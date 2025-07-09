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
from .config_so100_double_follower import SO100DoubleFollowerConfig

logger = logging.getLogger(__name__)


class SO100DoubleFollower(Robot):
    """
    [SO-100 Double Follower Arm](https://github.com/TheRobotStudio/SO-ARM100) designed by TheRobotStudio
    """

    config_class = SO100DoubleFollowerConfig
    name = "so100_double_follower"

    def __init__(self, config: SO100DoubleFollowerConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.left_bus = FeetechMotorsBus(
            port=self.config.left_port,
            motors={
                "left_shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "left_shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "left_elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "left_wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "left_wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "left_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration={k: v for k, v in self.calibration.items() if k.startswith("left")},
        )
        self.right_bus = FeetechMotorsBus(
            port=self.config.right_port,
            motors={
                "right_shoulder_pan": Motor(7, "sts3215", norm_mode_body),
                "right_shoulder_lift": Motor(8, "sts3215", norm_mode_body),
                "right_elbow_flex": Motor(9, "sts3215", norm_mode_body),
                "right_wrist_flex": Motor(10, "sts3215", norm_mode_body),
                "right_wrist_roll": Motor(11, "sts3215", norm_mode_body),
                "right_gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration={k: v for k, v in self.calibration.items() if k.startswith("right")},
        )
        self.left_motors = [motor for motor in self.left_bus.motors]
        self.right_motors = [motor for motor in self.right_bus.motors]
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            **{f"{motor}.pos": float for motor in self.left_bus.motors},
            **{f"{motor}.pos": float for motor in self.right_bus.motors},
        }

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
        return self.left_bus.is_connected and self.right_bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

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
        self.left_bus.disable_torque()
        self.right_bus.disable_torque()
        for motor in self.left_bus.motors:
            self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        for motor in self.right_bus.motors:
            self.right_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move left arm to the middle of its range of motion and press ENTER....")
        left_homing_offsets = self.left_bus.set_half_turn_homings(self.left_motors)
        

        left_full_turn_motor = "left_wrist_roll"
        left_unknown_range_motors = [motor for motor in self.left_motors if motor != left_full_turn_motor]
        print(
            f"Move all joints except '{left_full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        left_range_mins, left_range_maxes = self.left_bus.record_ranges_of_motion(left_unknown_range_motors)
        for name in left_full_turn_motor:
            left_range_mins[name] = 0
            left_range_maxes[name] = 4095

        input(f"Move right arm to the middle of its range of motion and press ENTER....")
        right_homing_offsets = self.right_bus.set_half_turn_homings(self.right_motors)
        
        right_full_turn_motor = "right_wrist_roll"
        right_unknown_range_motors = [motor for motor in self.right_motors if motor != right_full_turn_motor]

        print(
            f"Move all joints except '{right_full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        right_range_mins, right_range_maxes = self.right_bus.record_ranges_of_motion(right_unknown_range_motors)
        for name in right_full_turn_motor:
            right_range_mins[name] = 0
            right_range_maxes[name] = 4095

        self.left_bus_calibration = {}
        for name, motor in self.left_bus.motors.items():
            self.left_bus_calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=left_homing_offsets[name],
                range_min=left_range_mins[name],
                range_max=left_range_maxes[name],
            )

        self.right_bus_calibration = {}
        for name, motor in self.right_bus.motors.items():
            drive_mode = 1 if name == "right_gripper" else 0
            self.right_bus_calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=drive_mode,
                homing_offset=right_homing_offsets[name],
                range_min=right_range_mins[name],
                range_max=right_range_maxes[name],
            )

        self.left_bus.write_calibration(self.left_bus_calibration)
        self.right_bus.write_calibration(self.right_bus_calibration)

        self.calibration = {**self.left_bus_calibration, **self.right_bus_calibration}
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)


    def configure(self) -> None:
        with self.left_bus.torque_disabled():
            self.left_bus.configure_motors()
            for motor in self.left_bus.motors:
                self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
                self.left_bus.write("P_Coefficient", motor, 16)
                # Set I_Coefficient and D_Coefficient to default value 0 and 32
                self.left_bus.write("I_Coefficient", motor, 0)
                self.left_bus.write("D_Coefficient", motor, 32)

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
        for motor in reversed(self.left_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.left_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.left_bus.motors[motor].id}")

        for motor in reversed(self.right_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.right_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.right_bus.motors[motor].id}")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()
        left_obs_dict = self.left_bus.sync_read("Present_Position")
        left_obs_dict = {f"{motor}.pos": val for motor, val in left_obs_dict.items()}

        right_obs_dict = self.right_bus.sync_read("Present_Position")
        right_obs_dict = {f"{motor}.pos": val for motor, val in right_obs_dict.items()}

        obs_dict = {**left_obs_dict, **right_obs_dict}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command arm to move to a target joint configuration.

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

        left_goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos") and key.startswith("left_")}
        right_goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos") and key.startswith("right_")}

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            left_present_pos = self.left_bus.sync_read("Present_Position")
            left_goal_present_pos = {key: (g_pos, left_present_pos[key]) for key, g_pos in left_goal_pos.items()}
            left_goal_pos = ensure_safe_goal_position(left_goal_present_pos, self.config.max_relative_target)
            
            right_present_pos = self.right_bus.sync_read("Present_Position")
            right_goal_present_pos = {key: (g_pos, right_present_pos[key]) for key, g_pos in right_goal_pos.items()}            
            right_goal_pos = ensure_safe_goal_position(right_goal_present_pos, self.config.max_relative_target)

        # Send goal position to the arm
        self.left_bus.sync_write("Goal_Position", left_goal_pos)
        self.right_bus.sync_write("Goal_Position", right_goal_pos)
        return {**left_goal_pos, **right_goal_pos}

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.left_bus.disconnect(self.config.disable_torque_on_disconnect)
        self.right_bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
