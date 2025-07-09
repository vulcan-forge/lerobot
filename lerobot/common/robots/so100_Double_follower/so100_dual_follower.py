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
from .config_so100_dual_follower import SO100DualFollowerConfig

logger = logging.getLogger(__name__)


class SO100DualFollower(Robot):
    """
    [SO-100 Dual Follower Arm](https://github.com/TheRobotStudio/SO-ARM100) designed by TheRobotStudio
    Dual arm configuration with left and right buses
    """

    config_class = SO100DualFollowerConfig
    name = "so100_dual_follower"

    def __init__(self, config: SO100DualFollowerConfig):
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
        """Calibrate both arms."""
        logger.info(f"\nRunning calibration of {self}")
        
        # Calibrate left arm
        logger.info("Calibrating left arm...")
        self.left_bus.disable_torque()
        for motor in self.left_bus.motors:
            self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move left arm to the middle of its range of motion and press ENTER....")
        left_homing_offsets = self.left_bus.set_half_turn_homings()

        full_turn_motor = "wrist_roll"
        unknown_range_motors = [motor for motor in self.left_bus.motors if motor != full_turn_motor]
        print(
            f"Move all left arm joints except '{full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        left_range_mins, left_range_maxes = self.left_bus.record_ranges_of_motion(unknown_range_motors)
        left_range_mins[full_turn_motor] = 0
        left_range_maxes[full_turn_motor] = 4095

        # Calibrate right arm
        logger.info("Calibrating right arm...")
        self.right_bus.disable_torque()
        for motor in self.right_bus.motors:
            self.right_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move right arm to the middle of its range of motion and press ENTER....")
        right_homing_offsets = self.right_bus.set_half_turn_homings()

        print(
            f"Move all right arm joints except '{full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        right_range_mins, right_range_maxes = self.right_bus.record_ranges_of_motion(unknown_range_motors)
        right_range_mins[full_turn_motor] = 0
        right_range_maxes[full_turn_motor] = 4095

        # Save calibration for both arms
        self.calibration = {}
        for motor, m in self.left_bus.motors.items():
            self.calibration[f"left_{motor}"] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=left_homing_offsets[motor],
                range_min=left_range_mins[motor],
                range_max=left_range_maxes[motor],
            )
        
        for motor, m in self.right_bus.motors.items():
            self.calibration[f"right_{motor}"] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=right_homing_offsets[motor],
                range_min=right_range_mins[motor],
                range_max=right_range_maxes[motor],
            )

        self.left_bus.write_calibration({k.replace("left_", ""): v for k, v in self.calibration.items() if k.startswith("left_")})
        self.right_bus.write_calibration({k.replace("right_", ""): v for k, v in self.calibration.items() if k.startswith("right_")})
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def disconnect(self) -> None:
        """Disconnect both arms and cameras."""
        if self.config.disable_torque_on_disconnect:
            self.left_bus.disable_torque()
            self.right_bus.disable_torque()
        
        self.left_bus.disconnect()
        self.right_bus.disconnect()
        
        for cam in self.cameras.values():
            cam.disconnect()
        
        logger.info(f"{self} disconnected.")

    def configure(self) -> None:
        """Configure both arms."""
        # Configure left arm
        with self.left_bus.torque_disabled():
            self.left_bus.configure_motors()
            for motor in self.left_bus.motors:
                self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
                self.left_bus.write("P_Coefficient", motor, 16)
                # Set I_Coefficient and D_Coefficient to default value 0 and 32
                self.left_bus.write("I_Coefficient", motor, 0)
                self.left_bus.write("D_Coefficient", motor, 32)
        
        # Configure right arm
        with self.right_bus.torque_disabled():
            self.right_bus.configure_motors()
            for motor in self.right_bus.motors:
                self.right_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
                self.right_bus.write("P_Coefficient", motor, 16)
                # Set I_Coefficient and D_Coefficient to default value 0 and 32
                self.right_bus.write("I_Coefficient", motor, 0)
                self.right_bus.write("D_Coefficient", motor, 32)

    def setup_motors(self) -> None:
        """Setup motors for both arms."""
        # Setup left arm motors
        for motor in reversed(self.left_bus.motors):
            input(f"Connect the controller board to the left arm '{motor}' motor only and press enter.")
            self.left_bus.setup_motor(motor)
            print(f"Left arm '{motor}' motor id set to {self.left_bus.motors[motor].id}")
        
        # Setup right arm motors
        for motor in reversed(self.right_bus.motors):
            input(f"Connect the controller board to the right arm '{motor}' motor only and press enter.")
            self.right_bus.setup_motor(motor)
            print(f"Right arm '{motor}' motor id set to {self.right_bus.motors[motor].id}")

    def get_observation(self) -> dict[str, Any]:
        """Get observation from both arms and cameras."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        observation = {}
        
        # Read arm positions
        start = time.perf_counter()
        left_obs_dict = self.left_bus.sync_read("Present_Position")
        right_obs_dict = self.right_bus.sync_read("Present_Position")
        
        # Combine states with prefixes
        for motor, val in left_obs_dict.items():
            observation[f"left_{motor}.pos"] = val
        
        for motor, val in right_obs_dict.items():
            observation[f"right_{motor}.pos"] = val
        
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            observation[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
        
        return observation

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

        # Split action into left and right arm actions
        left_goal_pos = {}
        right_goal_pos = {}
        
        for key, value in action.items():
            if key.startswith("left_") and key.endswith(".pos"):
                motor_name = key.replace("left_", "").replace(".pos", "")
                left_goal_pos[motor_name] = value
            elif key.startswith("right_") and key.endswith(".pos"):
                motor_name = key.replace("right_", "").replace(".pos", "")
                right_goal_pos[motor_name] = value
        
        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            left_present_pos = self.left_bus.sync_read("Present_Position")
            right_present_pos = self.right_bus.sync_read("Present_Position")
            
            left_goal_present_pos = {key: (g_pos, left_present_pos[key]) for key, g_pos in left_goal_pos.items()}
            right_goal_present_pos = {key: (g_pos, right_present_pos[key]) for key, g_pos in right_goal_pos.items()}
            
            left_goal_pos = ensure_safe_goal_position(left_goal_present_pos, self.config.max_relative_target)
            right_goal_pos = ensure_safe_goal_position(right_goal_present_pos, self.config.max_relative_target)
        
        # Send goal positions to both arms
        self.left_bus.sync_write("Goal_Position", left_goal_pos)
        self.right_bus.sync_write("Goal_Position", right_goal_pos)
        
        # Combine sent actions with prefixes
        sent_action = {}
        for motor, val in left_goal_pos.items():
            sent_action[f"left_{motor}.pos"] = val
        for motor, val in right_goal_pos.items():
            sent_action[f"right_{motor}.pos"] = val
        
        return sent_action 