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

from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.common.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..teleoperator import Teleoperator
from .config_so100_double_leader import SO100DoubleLeaderConfig

logger = logging.getLogger(__name__)


class SO100DoubleLeader(Teleoperator):
    """
    [SO-100 Double Leader Arm](https://github.com/TheRobotStudio/SO-ARM100) designed by TheRobotStudio
    """

    config_class = SO100DoubleLeaderConfig
    name = "so100_double_leader"

    def __init__(self, config: SO100DoubleLeaderConfig):
        super().__init__(config)
        self.config = config
        self.left_bus = FeetechMotorsBus(
            port=self.config.left_port,
            motors={
                "left_shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.right_bus = FeetechMotorsBus(
            port=self.config.right_port,
            motors={
                "right_shoulder_pan": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_shoulder_lift": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_elbow_flex": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_wrist_flex": Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_wrist_roll": Motor(11, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
    @property
    def action_features(self) -> dict[str, type]:
        return {
            **{f"{motor}.pos": float for motor in self.left_bus.motors},
            **{f"{motor}.pos": float for motor in self.right_bus.motors}
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_bus.is_connected and self.right_bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.left_bus.connect()
        self.right_bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        # Check if the teleoperator calibration file exists and has been loaded
        is_calibrated = self.calibration_fpath.is_file() and len(self.calibration) > 0
        print(f"DEBUG: Teleoperator calibrated: {is_calibrated}")
        print(f"DEBUG: Calibration file path: {self.calibration_fpath}")
        print(f"DEBUG: Calibration file exists: {self.calibration_fpath.is_file()}")
        print(f"DEBUG: Calibration dict keys: {list(self.calibration.keys())}")
        return is_calibrated

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration of {self}")
        self.left_bus.disable_torque()
        self.right_bus.disable_torque()
        for motor in self.left_bus.motors:
            self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        for motor in self.right_bus.motors:
            self.right_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move left teleoperator to the middle of its range of motion and press ENTER....")
        left_homing_offsets = self.left_bus.set_half_turn_homings()

        left_full_turn_motor = "left_wrist_roll"
        left_unknown_range_motors = [motor for motor in self.left_bus.motors if motor != left_full_turn_motor]
        print(
            f"Move all joints except '{left_full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        left_range_mins, left_range_maxes = self.left_bus.record_ranges_of_motion(left_unknown_range_motors)
        left_range_mins[left_full_turn_motor] = 0
        left_range_maxes[left_full_turn_motor] = 4095

        input(f"Move right teleoperator to the middle of its range of motion and press ENTER....")
        right_homing_offsets = self.right_bus.set_half_turn_homings()

        right_full_turn_motor = "right_wrist_roll"
        right_unknown_range_motors = [motor for motor in self.right_bus.motors if motor != right_full_turn_motor]
        print(
            f"Move all joints except '{right_full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        right_range_mins, right_range_maxes = self.right_bus.record_ranges_of_motion(right_unknown_range_motors)
        right_range_mins[right_full_turn_motor] = 0
        right_range_maxes[right_full_turn_motor] = 4095

        self.left_calibration = {}
        for name, motor in self.left_bus.motors.items():
            self.left_calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=left_homing_offsets[name],
                range_min=left_range_mins[name],
                range_max=left_range_maxes[name],
            )

        self.right_calibration = {}
        for name, motor in self.right_bus.motors.items():
            self.right_calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=right_homing_offsets[name],
                range_min=right_range_mins[name],
                range_max=right_range_maxes[name],
            )

        self.left_bus.write_calibration(self.left_calibration)
        self.right_bus.write_calibration(self.right_calibration)

        self.calibration = {**self.left_calibration, **self.right_calibration}
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.left_bus.disable_torque()
        self.left_bus.configure_motors()
        for motor in self.left_bus.motors:
            self.left_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def setup_motors(self) -> None:
        for motor in reversed(self.left_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.left_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.left_bus.motors[motor].id}")

        for motor in reversed(self.right_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.right_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.right_bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        left_action = self.left_bus.sync_read("Present_Position")
        left_action = {f"{motor}.pos": val for motor, val in left_action.items()}

        right_action = self.right_bus.sync_read("Present_Position")
        right_action = {f"{motor}.pos": val for motor, val in right_action.items()}

        action = {**left_action, **right_action}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")

        self.left_bus.disconnect()
        self.right_bus.disconnect()
        logger.info(f"{self} disconnected.")
