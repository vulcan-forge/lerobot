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

from ...teleoperator import Teleoperator
from .config_sourccey_v2beta_teleop import SourcceyV2BetaTeleopConfig

logger = logging.getLogger(__name__)


class SourcceyV2BetaTeleop(Teleoperator):
    """
    [Sourccey V2 Beta Teleoperator] designed by Vulcan
    """

    config_class = SourcceyV2BetaTeleopConfig
    name = "sourccey_v2beta_teleop"

    def __init__(self, config: SourcceyV2BetaTeleopConfig):
        super().__init__(config)
        self.config = config
        self.left_arm_bus = FeetechMotorsBus(
            port=self.config.left_arm_port,
            motors={
                "left_arm_shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_arm_shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_arm_elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_arm_wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_arm_wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "left_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=self.calibration,
        )
        self.right_arm_bus = FeetechMotorsBus(
            port=self.config.right_arm_port,
            motors={
                "right_arm_shoulder_pan": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_arm_shoulder_lift": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_arm_elbow_flex": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_arm_wrist_flex": Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_arm_wrist_roll": Motor(11, "sts3215", MotorNormMode.RANGE_M100_100),
                "right_arm_gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
            },
        )

    @property
    def action_features(self) -> dict[str, type]:
        return {
            **{f"{motor}.pos": float for motor in self.left_arm_bus.motors},
            **{f"{motor}.pos": float for motor in self.right_arm_bus.motors}
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_arm_bus.is_connected and self.right_arm_bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.left_arm_bus.connect()
        self.right_arm_bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm_bus.is_calibrated and self.right_arm_bus.is_calibrated

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration of {self}")
        self.left_arm_bus.disable_torque()
        self.right_arm_bus.disable_torque()
        for motor in self.left_arm_bus.motors:
            self.left_arm_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        for motor in self.right_arm_bus.motors:
            self.right_arm_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move Left Arm Teleoperator to the middle of its range of motion and press ENTER....")
        homing_offsets = self.left_arm_bus.set_half_turn_homings()
        input(f"Move Right Arm Teleoperator to the middle of its range of motion and press ENTER....")
        homing_offsets = self.right_arm_bus.set_half_turn_homings()

        full_turn_motor = "wrist_roll"
        unknown_left_arm_range_motors = [motor for motor in self.left_arm_bus.motors if motor != full_turn_motor]
        unknown_right_arm_range_motors = [motor for motor in self.right_arm_bus.motors if motor != full_turn_motor]
        print(
            f"Move all joints except '{full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        left_arm_range_mins, left_arm_range_maxes = self.left_arm_bus.record_ranges_of_motion(unknown_left_arm_range_motors)
        right_arm_range_mins, right_arm_range_maxes = self.right_arm_bus.record_ranges_of_motion(unknown_right_arm_range_motors)
        left_arm_range_mins[full_turn_motor] = 0
        left_arm_range_maxes[full_turn_motor] = 4095
        right_arm_range_mins[full_turn_motor] = 0
        right_arm_range_maxes[full_turn_motor] = 4095

        self.calibration = {}
        for motor, m in self.left_arm_bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=left_arm_range_mins[motor],
                range_max=left_arm_range_maxes[motor],
            )
        for motor, m in self.right_arm_bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=right_arm_range_mins[motor],
                range_max=right_arm_range_maxes[motor],
            )

        self.left_arm_bus.write_calibration(self.calibration)
        self.right_arm_bus.write_calibration(self.calibration)
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.left_arm_bus.disable_torque()
        self.right_arm_bus.disable_torque()
        self.left_arm_bus.configure_motors()
        self.right_arm_bus.configure_motors()
        for motor in self.left_arm_bus.motors:
            self.left_arm_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
        for motor in self.right_arm_bus.motors:
            self.right_arm_bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def setup_motors(self) -> None:
        for motor in reversed(self.left_arm_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.left_arm_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.left_arm_bus.motors[motor].id}")
        for motor in reversed(self.right_arm_bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.right_arm_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.right_arm_bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        action = self.left_arm_bus.sync_read("Present_Position")
        action = {f"{motor}.pos": val for motor, val in action.items()}
        action = self.right_arm_bus.sync_read("Present_Position")
        action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")

        self.left_arm_bus.disconnect()
        self.right_arm_bus.disconnect()
        logger.info(f"{self} disconnected.")
