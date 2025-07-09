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
from functools import cached_property
from typing import Any

from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.motors import Motor, MotorNormMode, MotorCalibration
from lerobot.common.motors.feetech import FeetechMotorsBus

from ..teleoperator import Teleoperator
from .config_so100_dual_leader import SO100DualLeaderConfig

logger = logging.getLogger(__name__)


class SO100DualLeader(Teleoperator):
    """
    [SO-100 Dual Leader Arm](https://github.com/TheRobotStudio/SO-ARM100) designed by TheRobotStudio
    Dual arm teleoperator configuration with left and right buses
    """

    config_class = SO100DualLeaderConfig
    name = "so100_dual_leader"

    def __init__(self, config: SO100DualLeaderConfig):
        super().__init__(config)
        self.config = config
        
        # Left leader bus with motor IDs 1-6
        self.left_bus = FeetechMotorsBus(
            port=self.config.left_port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
        )
        
        # Right leader bus with motor IDs 7-12
        self.right_bus = FeetechMotorsBus(
            port=self.config.right_port,
            motors={
                "shoulder_pan": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(11, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
            },
        )

    @property
    def _motors_ft(self) -> dict[str, type]:
        # Combine motors from both buses
        left_motors = {f"left_{motor}.pos": float for motor in self.left_bus.motors}
        right_motors = {f"right_{motor}.pos": float for motor in self.right_bus.motors}
        return {**left_motors, **right_motors}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def feedback_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.left_bus.is_connected and self.right_bus.is_connected

    @property
    def is_calibrated(self) -> bool:
        return True  # Leader arms don't need calibration

    def connect(self, calibrate: bool = True) -> None:
        """Connect to both leader arms."""
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.left_bus.connect()
        self.right_bus.connect()
        self.configure()
        logger.info(f"{self} connected.")

    def disconnect(self) -> None:
        """Disconnect from both leader arms."""
        self.left_bus.disconnect()
        self.right_bus.disconnect()
        logger.info(f"{self} disconnected.")

    def calibrate(self) -> None:
        """Calibrate both leader arms."""
        logger.info(f"Calibrating {self}...")
        
        # Calibrate left arm
        logger.info("Calibrating left leader arm...")
        self.left_bus.disable_torque()
        for motor in self.left_bus.motors:
            self.left_bus.write("Operating_Mode", motor, 3)  # Position control mode

        input(f"Move left leader arm to the middle of its range of motion and press ENTER....")
        left_homing_offsets = self.left_bus.set_half_turn_homings()

        full_turn_motor = "wrist_roll"
        unknown_range_motors = [motor for motor in self.left_bus.motors if motor != full_turn_motor]
        print(
            f"Move all left leader arm joints except '{full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        left_range_mins, left_range_maxes = self.left_bus.record_ranges_of_motion(unknown_range_motors)
        left_range_mins[full_turn_motor] = 0
        left_range_maxes[full_turn_motor] = 4095

        # Calibrate right arm
        logger.info("Calibrating right leader arm...")
        self.right_bus.disable_torque()
        for motor in self.right_bus.motors:
            self.right_bus.write("Operating_Mode", motor, 3)  # Position control mode

        input(f"Move right leader arm to the middle of its range of motion and press ENTER....")
        right_homing_offsets = self.right_bus.set_half_turn_homings()

        print(
            f"Move all right leader arm joints except '{full_turn_motor}' sequentially through their "
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
                homing_offset=int(left_homing_offsets[motor]),
                range_min=int(left_range_mins[motor]),
                range_max=int(left_range_maxes[motor]),
            )
        
        for motor, m in self.right_bus.motors.items():
            self.calibration[f"right_{motor}"] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=int(right_homing_offsets[motor]),
                range_min=int(right_range_mins[motor]),
                range_max=int(right_range_maxes[motor]),
            )

        self.left_bus.write_calibration({k.replace("left_", ""): v for k, v in self.calibration.items() if k.startswith("left_")})
        self.right_bus.write_calibration({k.replace("right_", ""): v for k, v in self.calibration.items() if k.startswith("right_")})
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self) -> None:
        """Configure both leader arms."""
        # Configure left arm
        with self.left_bus.torque_disabled():
            self.left_bus.configure_motors()
            for motor in self.left_bus.motors:
                self.left_bus.write("Operating_Mode", motor, 3)  # Position control mode
                # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
                self.left_bus.write("P_Coefficient", motor, 16)
                # Set I_Coefficient and D_Coefficient to default value 0 and 32
                self.left_bus.write("I_Coefficient", motor, 0)
                self.left_bus.write("D_Coefficient", motor, 32)
        
        # Configure right arm
        with self.right_bus.torque_disabled():
            self.right_bus.configure_motors()
            for motor in self.right_bus.motors:
                self.right_bus.write("Operating_Mode", motor, 3)  # Position control mode
                # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
                self.right_bus.write("P_Coefficient", motor, 16)
                # Set I_Coefficient and D_Coefficient to default value 0 and 32
                self.right_bus.write("I_Coefficient", motor, 0)
                self.right_bus.write("D_Coefficient", motor, 32)

    def get_action(self) -> dict[str, float]:
        """Get action from both leader arms."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read positions from both arms
        left_positions = self.left_bus.sync_read("Present_Position")
        right_positions = self.right_bus.sync_read("Present_Position")
        
        # Combine positions with prefixes
        action = {}
        for motor, pos in left_positions.items():
            action[f"left_{motor}.pos"] = pos
        
        for motor, pos in right_positions.items():
            action[f"right_{motor}.pos"] = pos
        
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Send feedback to both leader arms."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Split feedback into left and right arm actions
        left_feedback = {}
        right_feedback = {}
        
        for key, value in feedback.items():
            if key.startswith("left_") and key.endswith(".pos"):
                motor_name = key.replace("left_", "").replace(".pos", "")
                left_feedback[motor_name] = value
            elif key.startswith("right_") and key.endswith(".pos"):
                motor_name = key.replace("right_", "").replace(".pos", "")
                right_feedback[motor_name] = value
        
        # Send feedback to both arms
        self.left_bus.sync_write("Goal_Position", left_feedback)
        self.right_bus.sync_write("Goal_Position", right_feedback) 