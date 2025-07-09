#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property
from typing import Any

from lerobot.common.cameras.utils import make_cameras_from_configs
from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.common.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_so100_follower import SO100FollowerConfig

logger = logging.getLogger(__name__)


class SO100Follower(Robot):
    """
    [SO-100 Double Follower Arm](https://github.com/TheRobotStudio/SO-ARM100) designed by TheRobotStudio
    Dual arm configuration with left and right buses
    """

    config_class = SO100FollowerConfig
    name = "so100_follower"

    def __init__(self, config: SO100FollowerConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        
        # Left bus with motor IDs 1-6
        self.left_bus = FeetechMotorsBus(
            port=self.config.left_port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        
        # Right bus with motor IDs 7-12
        self.right_bus = FeetechMotorsBus(
            port=self.config.right_port,
            motors={
                "shoulder_pan": Motor(7, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(8, "sts3215", norm_mode_body),
                "elbow_flex": Motor(9, "sts3215", norm_mode_body),
                "wrist_flex": Motor(10, "sts3215", norm_mode_body),
                "wrist_roll": Motor(11, "sts3215", norm_mode_body),
                "gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        
        # Keep the original bus reference for backward compatibility
        self.bus = self.left_bus
        
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        # Combine motors from both buses
        left_motors = {f"left_{motor}.pos": float for motor in self.left_bus.motors}
        right_motors = {f"right_{motor}.pos": float for motor in self.right_bus.motors}
        return {**left_motors, **right_motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return (self.left_bus.is_connected and self.right_bus.is_connected and 
                all(cam.is_connected for cam in self.cameras.values()))

    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.left_bus.connect()
        self.right_bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.left_bus.is_calibrated and self.right_bus.is_calibrated

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        full_turn_motor = "wrist_roll"
        unknown_range_motors = [motor for motor in self.bus.motors if motor != full_turn_motor]
        print(
            f"Move all joints except '{full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        range_mins[full_turn_motor] = 0
        range_maxes[full_turn_motor] = 4095

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        with self.bus.torque_disabled():
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
                self.bus.write("P_Coefficient", motor, 16)
                # Set I_Coefficient and D_Coefficient to default value 0 and 32
                self.bus.write("I_Coefficient", motor, 0)
                self.bus.write("D_Coefficient", motor, 32)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read left arm position
        start = time.perf_counter()
        left_obs_dict = self.left_bus.sync_read("Present_Position")
        left_obs_dict = {f"left_{motor}.pos": val for motor, val in left_obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read left arm state: {dt_ms:.1f}ms")

        # Read right arm position
        start = time.perf_counter()
        right_obs_dict = self.right_bus.sync_read("Present_Position")
        right_obs_dict = {f"right_{motor}.pos": val for motor, val in right_obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read right arm state: {dt_ms:.1f}ms")

        # Combine observations
        obs_dict = {**left_obs_dict, **right_obs_dict}

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command both arms to move to target joint configurations.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            the action sent to the motors, potentially clipped.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Separate left and right arm actions
        left_goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() 
                        if key.startswith("left_") and key.endswith(".pos")}
        right_goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() 
                         if key.startswith("right_") and key.endswith(".pos")}

        # Cap goal positions when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            # Handle left arm
            if left_goal_pos:
                left_present_pos = self.left_bus.sync_read("Present_Position")
                left_goal_present_pos = {key: (g_pos, left_present_pos[key]) for key, g_pos in left_goal_pos.items()}
                left_goal_pos = ensure_safe_goal_position(left_goal_present_pos, self.config.max_relative_target)
            
            # Handle right arm
            if right_goal_pos:
                right_present_pos = self.right_bus.sync_read("Present_Position")
                right_goal_present_pos = {key: (g_pos, right_present_pos[key]) for key, g_pos in right_goal_pos.items()}
                right_goal_pos = ensure_safe_goal_position(right_goal_present_pos, self.config.max_relative_target)

        # Send goal positions to both arms
        if left_goal_pos:
            self.left_bus.sync_write("Goal_Position", left_goal_pos)
        if right_goal_pos:
            self.right_bus.sync_write("Goal_Position", right_goal_pos)
        
        # Return the combined action that was actually sent
        left_action = {f"left_{motor}.pos": val for motor, val in left_goal_pos.items()}
        right_action = {f"right_{motor}.pos": val for motor, val in right_goal_pos.items()}
        return {**left_action, **right_action}

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.left_bus.disconnect(self.config.disable_torque_on_disconnect)
        self.right_bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
