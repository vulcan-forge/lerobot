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
            calibration={k: v for k, v in self.calibration.items() if k.startswith("left_arm")},
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
            calibration={k: v for k, v in self.calibration.items() if k.startswith("right_arm")},
        )
        self.left_arm_motors = [motor for motor in self.left_arm_bus.motors]
        self.right_arm_motors = [motor for motor in self.right_arm_bus.motors]

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

        input(f"Move left arm teleoperator to the middle of its range of motion and press ENTER....")
        left_arm_homing_offsets = self.left_arm_bus.set_half_turn_homings(self.left_arm_motors)

        left_arm_full_turn_motor = ["left_arm_wrist_roll"]
        left_arm_unknown_range_motors = [motor for motor in self.left_arm_bus.motors if motor != left_arm_full_turn_motor]

        print(
            f"Move all joints except '{left_arm_full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        left_arm_range_mins, left_arm_range_maxes = self.left_arm_bus.record_ranges_of_motion(left_arm_unknown_range_motors)
        for name in left_arm_full_turn_motor:
            left_arm_range_mins[name] = 0
            left_arm_range_maxes[name] = 4095

        input("Move right arm teleoperator to the middle of its range of motion and press ENTER....")
        right_arm_homing_offsets = self.right_arm_bus.set_half_turn_homings(self.right_arm_motors)

        right_arm_full_turn_motor = ["right_arm_wrist_roll"]
        right_arm_unknown_range_motors = [motor for motor in self.right_arm_motors if motor not in right_arm_full_turn_motor]

        print(
            f"Move all arm joints except '{right_arm_full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        right_arm_range_mins, right_arm_range_maxes = self.right_arm_bus.record_ranges_of_motion(right_arm_unknown_range_motors)
        for name in right_arm_full_turn_motor:
            right_arm_range_mins[name] = 0
            right_arm_range_maxes[name] = 4095

        self.left_arm_calibration = {}
        for name, motor in self.left_arm_bus.motors.items():
            self.left_arm_calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=left_arm_homing_offsets[name],
                range_min=left_arm_range_mins[name],
                range_max=left_arm_range_maxes[name],
            )

        self.right_arm_calibration = {}
        for name, motor in self.right_arm_bus.motors.items():
            self.right_arm_calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=0,
                homing_offset=right_arm_homing_offsets[name],
                range_min=right_arm_range_mins[name],
                range_max=right_arm_range_maxes[name],
            )

        self.left_arm_bus.write_calibration(self.left_arm_calibration)
        self.right_arm_bus.write_calibration(self.right_arm_calibration)

        self.calibration = {**self.left_arm_calibration, **self.right_arm_calibration}
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

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
        left_arm_action = self.left_arm_bus.sync_read("Present_Position")
        left_arm_action = {f"{motor}.pos": val for motor, val in left_arm_action.items()}
        right_arm_action = self.right_arm_bus.sync_read("Present_Position")
        right_arm_action = {f"{motor}.pos": val for motor, val in right_arm_action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return {**left_arm_action, **right_arm_action}

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")

        self.left_arm_bus.disconnect()
        self.right_arm_bus.disconnect()
        logger.info(f"{self} disconnected.")
