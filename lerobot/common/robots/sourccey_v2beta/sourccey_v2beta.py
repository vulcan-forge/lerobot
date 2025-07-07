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
from itertools import chain
from typing import Any

import numpy as np

from lerobot.common.cameras.utils import make_cameras_from_configs
from lerobot.common.constants import OBS_IMAGES, OBS_STATE
from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.common.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from lerobot.common.robots.robot import Robot
from lerobot.common.robots.utils import ensure_safe_goal_position
from .config_sourccey_v2beta import SourcceyV2BetaConfig

logger = logging.getLogger(__name__)


class SourcceyV2Beta(Robot):
    """
    The robot includes a four mecanum wheel mobile base and 2 remote follower arms.
    The leader arm is connected locally (on the laptop) and its joint positions are recorded and then
    forwarded to the remote follower arm (after applying a safety clamp).
    In parallel, keyboard teleoperation is used to generate raw velocity commands for the wheels.
    """

    config_class = SourcceyV2BetaConfig
    name = "sourccey_v2beta"

    def __init__(self, config: SourcceyV2BetaConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.left_arm_bus = FeetechMotorsBus(
            port=self.config.left_arm_port,
            motors={
                "left_arm_shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "left_arm_shoulder_lift": Motor(2, "sts3235", norm_mode_body, gear_ratio=3.0),
                "left_arm_elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "left_arm_wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "left_arm_wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "left_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration={k: v for k, v in self.calibration.items() if k.startswith("left_arm")},
        )
        self.right_arm_bus = FeetechMotorsBus(
            port=config.right_arm_port,
            motors={
                "right_arm_shoulder_pan": Motor(7, "sts3215", norm_mode_body),
                "right_arm_shoulder_lift": Motor(8, "sts3235", norm_mode_body, gear_ratio=3.0),
                "right_arm_elbow_flex": Motor(9, "sts3215", norm_mode_body),
                "right_arm_wrist_flex": Motor(10, "sts3215", norm_mode_body),
                "right_arm_wrist_roll": Motor(11, "sts3215", norm_mode_body),
                "right_arm_gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration={k: v for k, v in self.calibration.items() if k.startswith("right_arm")},
        )
        self.left_arm_motors = [motor for motor in self.left_arm_bus.motors]
        self.right_arm_motors = [motor for motor in self.right_arm_bus.motors]
        self.cameras = make_cameras_from_configs(config.cameras)
    @property
    def _state_ft(self) -> dict[str, type]:
        return dict.fromkeys(
            (
                "left_arm_shoulder_pan.pos",
                "left_arm_shoulder_lift.pos",
                "left_arm_elbow_flex.pos",
                "left_arm_wrist_flex.pos",
                "left_arm_wrist_roll.pos",
                "left_arm_gripper.pos",
                "right_arm_shoulder_pan.pos",
                "right_arm_shoulder_lift.pos",
                "right_arm_elbow_flex.pos",
                "right_arm_wrist_flex.pos",
                "right_arm_wrist_roll.pos",
                "right_arm_gripper.pos",
            ),
            float,
        )

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        return self.left_arm_bus.is_connected and self.right_arm_bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.left_arm_bus.connect()
        self.right_arm_bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

        if self.config.home_on_start:
            self.home_motors()

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm_bus.is_calibrated and self.right_arm_bus.is_calibrated

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration of {self}")

        self.left_arm_bus.disable_torque(self.left_arm_motors)
        self.right_arm_bus.disable_torque(self.right_arm_motors)
        for name in self.left_arm_motors:
            self.left_arm_bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
        for name in self.right_arm_motors:
            self.right_arm_bus.write("Operating_Mode", name, OperatingMode.POSITION.value)

        input("Move left arm of the robot to the middle of its range of motion and press ENTER....")
        left_arm_homing_offsets = self.left_arm_bus.set_half_turn_homings(self.left_arm_motors)
        left_arm_homing_offsets["left_arm_shoulder_lift"] = 0 # Set offset to 0 for multi turn motors

        left_arm_full_turn_motor = ["left_arm_wrist_roll"]
        left_arm_multi_turn_motors = ["left_arm_shoulder_lift"]
        left_arm_unknown_range_motors = [motor for motor in self.left_arm_motors if motor not in left_arm_full_turn_motor and motor not in left_arm_multi_turn_motors]

        print(
            f"Move all arm joints except '{left_arm_full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        left_arm_range_mins, left_arm_range_maxes = self.left_arm_bus.record_ranges_of_motion(left_arm_unknown_range_motors)
        for name in left_arm_full_turn_motor:
            left_arm_range_mins[name] = 0
            left_arm_range_maxes[name] = 4095

        for name in left_arm_multi_turn_motors:
            gear_ratio = self.left_arm_bus.motors[name].gear_ratio
            left_arm_range_mins[name] = 0
            left_arm_range_maxes[name] = int((4096 * gear_ratio) - 1)

        input("Move right arm of the robot to the middle of its range of motion and press ENTER....")
        right_arm_homing_offsets = self.right_arm_bus.set_half_turn_homings(self.right_arm_motors)
        right_arm_homing_offsets["right_arm_shoulder_lift"] = 0 # Set offset to 0 for multi turn motors

        right_arm_full_turn_motor = ["right_arm_wrist_roll"]
        right_arm_multi_turn_motors = ["right_arm_shoulder_lift"]
        right_arm_unknown_range_motors = [motor for motor in self.right_arm_motors if motor not in right_arm_full_turn_motor and motor not in right_arm_multi_turn_motors]

        print(
            f"Move all arm joints except '{right_arm_full_turn_motor}' sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        right_arm_range_mins, right_arm_range_maxes = self.right_arm_bus.record_ranges_of_motion(right_arm_unknown_range_motors)
        for name in right_arm_full_turn_motor:
            right_arm_range_mins[name] = 0
            right_arm_range_maxes[name] = 4095

        for name in right_arm_multi_turn_motors:
            gear_ratio = self.right_arm_bus.motors[name].gear_ratio
            right_arm_range_mins[name] = 0
            right_arm_range_maxes[name] = int((4096 * gear_ratio) - 1)

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
            drive_mode = 1 if name == "right_arm_gripper" else 0
            self.right_arm_calibration[name] = MotorCalibration(
                id=motor.id,
                drive_mode=drive_mode,
                homing_offset=right_arm_homing_offsets[name],
                range_min=right_arm_range_mins[name],
                range_max=right_arm_range_maxes[name],
            )

        self.left_arm_bus.write_calibration(self.left_arm_calibration)
        self.right_arm_bus.write_calibration(self.right_arm_calibration)

        self.calibration = {**self.left_arm_calibration, **self.right_arm_calibration}
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def update_profile(self) -> None:
        # Get the positions of the motors
        left_arm_pos = self.left_arm_bus.sync_read("Present_Position", self.left_arm_motors)
        right_arm_pos = self.right_arm_bus.sync_read("Present_Position", self.right_arm_motors)
        arm_pos = {**left_arm_pos, **right_arm_pos}

        # Any geared down, multi-turn motor instead needs to have a min = -1 or max = 1 value
        # to determine if homing will go to it's min or max position.
        geared_down_multi_turn_motors = ["left_arm_shoulder_lift", "right_arm_shoulder_lift"]
        profile = {}
        motor_homings = {}
        for motor, pos in arm_pos.items():
            full_rotate = 1 if motor in geared_down_multi_turn_motors else 0
            motor_homings[motor] = {
                "pos": pos,
                "full_rotate": full_rotate,
            }

        profile = { "motor_homings": motor_homings, "home_on_start": True }
        self.profile = profile
        self._save_profile()
        print("Profile saved to", self.profile_fpath)

    def configure(self):
        # Set-up arm actuators (position mode)
        # We assume that at connection time, arm is in a rest position,
        # and torque can be safely disabled to run calibration.
        self.left_arm_bus.disable_torque()
        self.right_arm_bus.disable_torque()
        self.left_arm_bus.configure_motors()
        self.right_arm_bus.configure_motors()
        for name in self.left_arm_motors:
            self.left_arm_bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.left_arm_bus.write("P_Coefficient", name, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.left_arm_bus.write("I_Coefficient", name, 0)
            self.left_arm_bus.write("D_Coefficient", name, 32)

        for name in self.right_arm_motors:
            self.right_arm_bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.right_arm_bus.write("P_Coefficient", name, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.right_arm_bus.write("I_Coefficient", name, 0)
            self.right_arm_bus.write("D_Coefficient", name, 32)

        self.left_arm_bus.enable_torque()
        self.right_arm_bus.enable_torque()

    def setup_motors(self) -> None:
        for motor in chain(reversed(self.left_arm_motors)):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.left_arm_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.left_arm_bus.motors[motor].id}")

        for motor in chain(reversed(self.right_arm_motors)):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.right_arm_bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.right_arm_bus.motors[motor].id}")

    def home_motors(self) -> None:
        print("Homing Motors 1")
        if not self.profile:
            return None

        print("Homing Motors 2")
        home_on_start = self.profile["home_on_start"]
        if not home_on_start:
            return None

        print("Homing Motors 3")
        motor_homings = self.profile["motor_homings"]
        print("Homing Motors 4", motor_homings)
        geared_down_multi_turn_motors = ["left_arm_shoulder_lift", "right_arm_shoulder_lift"]
        print("Homing Motors 5",  motor_homings.items())
        home_motor_positions = {k: v["pos"] for k, v in motor_homings.items() if k not in geared_down_multi_turn_motors}
        print("Homing Motors 6", home_motor_positions)

        # For the geared down, multi-turn motors, rotate until max or min or until send_action prevents the motor
        # from turning further.
        geared_down_home_motor_positions = {}
        for motor in geared_down_multi_turn_motors:
            full_rotate = motor_homings[motor]["full_rotate"]
            if full_rotate == 1:
                geared_down_home_motor_positions[motor] = self.calibration[motor].range_max
            elif full_rotate == -1:
                geared_down_home_motor_positions[motor] = self.calibration[motor].range_min

        full_home_motor_positions = {**home_motor_positions, **geared_down_home_motor_positions}
        print("Homing Motors 7", full_home_motor_positions)
        # self.send_action(full_home_motor_positions)
        print("Homing Motors 8 Commented out")
        logger.info(f"{self} homing motors for 10 seconds.")
        time.sleep(10)

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read actuators position for arm and vel for base
        start = time.perf_counter()
        left_arm_pos = self.left_arm_bus.sync_read("Present_Position", self.left_arm_motors)
        right_arm_pos = self.right_arm_bus.sync_read("Present_Position", self.right_arm_motors)
        arm_pos = {**left_arm_pos, **right_arm_pos}
        # base_wheel_vel = self.bus.sync_read("Present_Velocity", self.base_motors)

        base_vel = {}
        # base_vel = self._wheel_raw_to_body(
        #     base_wheel_vel["base_left_wheel"],
        #     base_wheel_vel["base_back_wheel"],
        #     base_wheel_vel["base_right_wheel"],
        # )

        arm_state = {f"{k}.pos": v for k, v in arm_pos.items()}

        obs_dict = {**arm_state, **base_vel}

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
        """Command SourcceyV2Beta to move to a target joint configuration.

        The relative action magnitude may be clipped depending on the configuration parameter
        `max_relative_target`. In this case, the action sent differs from original action.
        Thus, this function always returns the action actually sent.

        Raises:
            RobotDeviceNotConnectedError: if robot is not connected.

        Returns:
            np.ndarray: the action sent to the motors, potentially clipped.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_arm_goal_pos = {k: v for k, v in action.items() if k.startswith("left_arm_") and k.endswith(".pos")}
        right_arm_goal_pos = {k: v for k, v in action.items() if k.startswith("right_arm_") and k.endswith(".pos")}
        base_goal_vel = {k: v for k, v in action.items() if k.endswith(".vel")}

        base_wheel_goal_vel = {}
        # base_wheel_goal_vel = self._body_to_wheel_raw(
        #     base_goal_vel["x.vel"], base_goal_vel["y.vel"], base_goal_vel["theta.vel"]
        # )
        # Check for NaN values and skip sending actions if any are found
        if any(np.isnan(v) for v in left_arm_goal_pos.values()) or any(np.isnan(v) for v in right_arm_goal_pos.values()):
            logger.warning("NaN values detected in left arm goal positions. Skipping action execution.")
            return {**left_arm_goal_pos, **right_arm_goal_pos, **base_goal_vel}

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        left_arm_present_pos = self.left_arm_bus.sync_read("Present_Position", self.left_arm_motors)
        right_arm_present_pos = self.right_arm_bus.sync_read("Present_Position", self.right_arm_motors)
        if self.config.max_relative_target is not None:
            left_arm_goal_present_pos = {key: (g_pos, left_arm_present_pos[key]) for key, g_pos in left_arm_goal_pos.items()}
            left_arm_adjusted_goal_present_pos = self._apply_minimum_action(left_arm_goal_present_pos)
            left_arm_safe_goal_pos = ensure_safe_goal_position(left_arm_adjusted_goal_present_pos, self.config.max_relative_target)
            left_arm_goal_pos = left_arm_safe_goal_pos

            right_arm_goal_present_pos = {key: (g_pos, right_arm_present_pos[key]) for key, g_pos in right_arm_goal_pos.items()}
            right_arm_adjusted_goal_present_pos = self._apply_minimum_action(right_arm_goal_present_pos)
            right_arm_safe_goal_pos = ensure_safe_goal_position(right_arm_adjusted_goal_present_pos, self.config.max_relative_target)
            right_arm_goal_pos = right_arm_safe_goal_pos

        # Send goal position to the actuators
        left_arm_goal_pos_raw = {k.replace(".pos", ""): v for k, v in left_arm_goal_pos.items()}
        self.left_arm_bus.sync_write("Goal_Position", left_arm_goal_pos_raw)
        right_arm_goal_pos_raw = {k.replace(".pos", ""): v for k, v in right_arm_goal_pos.items()}
        self.right_arm_bus.sync_write("Goal_Position", right_arm_goal_pos_raw)
        # self.bus.sync_write("Goal_Velocity", base_wheel_goal_vel)

        # Check safety after sending goals
        is_safe, overcurrent_motors = self._check_current_safety()
        if not is_safe:
            left_arm_goal_pos, right_arm_goal_pos = self._handle_overcurrent_motors(
                left_arm_present_pos,
                right_arm_present_pos,
                left_arm_goal_pos,
                right_arm_goal_pos,
                overcurrent_motors
            )

        return {**left_arm_goal_pos, **right_arm_goal_pos, **base_goal_vel}

    def _apply_minimum_action(self, goal_present_pos: dict[str, tuple[float, float]]) -> dict[str, tuple[float, float]]:
        """Apply a minimum action to the robot's geared down motors.

        This function ensures that geared-down motors receive a minimum movement threshold
        to overcome friction and backlash. If the desired movement is below the threshold,
        it's amplified to the minimum threshold while preserving direction.

        Args:
            goal_present_pos (dict[str, tuple[float, float]]): Dictionary mapping motor names to
                tuples of (goal_position, present_position).

        Returns:
            dict[str, tuple[float, float]]: Dictionary mapping motor names to tuples of
                (adjusted_goal_position, present_position) for all motors.
        """
        # Define geared down motors and their minimum action thresholds
        geared_down_motors = ["left_arm_shoulder_lift", "right_arm_shoulder_lift"]
        min_action_threshold = 0.5  # Minimum movement threshold in normalized units

        adjusted_goal_present_pos = {}

        for key, (goal_pos, present_pos) in goal_present_pos.items():
            motor_name = key.replace(".pos", "")

            if motor_name in geared_down_motors:
                desired_movement = goal_pos - present_pos
                movement_magnitude = abs(desired_movement)

                # If movement is below threshold, apply minimum action
                if movement_magnitude > 0 and movement_magnitude < min_action_threshold:
                    direction = 1 if desired_movement > 0 else -1
                    adjusted_movement = direction * min_action_threshold
                    adjusted_goal_pos = present_pos + adjusted_movement
                    adjusted_goal_present_pos[key] = (adjusted_goal_pos, present_pos)
                else:
                    adjusted_goal_present_pos[key] = (goal_pos, present_pos)
            else:
                adjusted_goal_present_pos[key] = (goal_pos, present_pos)

        print()
        print("goal_present_pos:")
        print(goal_present_pos)
        print()
        print("adjusted_goal_present_pos:")
        print(adjusted_goal_present_pos)
        print()

        return adjusted_goal_present_pos

    def _check_current_safety(self) -> tuple[bool, list[str]]:
        """
        Check if any motor is over current limit and return safety status.

        Returns:
            tuple: (is_safe, overcurrent_motors)
            - is_safe: True if all motors are under current limit
            - overcurrent_motors: List of motor names that are over current
        """
        # Read current from all motors
        left_arm_current = self.left_arm_bus.sync_read("Present_Current", self.left_arm_motors)
        right_arm_current = self.right_arm_bus.sync_read("Present_Current", self.right_arm_motors)
        all_currents = {**left_arm_current, **right_arm_current}

        # Hardcoded safety threshold
        CURRENT_SAFETY_THRESHOLD = 500  # 500mA

        # Check if any motor is over the current limit
        overcurrent_motors = []
        for motor_name, current in all_currents.items():
            if current > CURRENT_SAFETY_THRESHOLD:
                overcurrent_motors.append(motor_name)
                logger.warning(f"Safety triggered: {motor_name} current {current}mA > {CURRENT_SAFETY_THRESHOLD}mA")

        if overcurrent_motors:
            logger.warning(f"Emergency stop triggered for motors: {overcurrent_motors}")
            return False, overcurrent_motors

        return True, []

    def _handle_overcurrent_motors(
        self,
        left_arm_present_pos: dict[str, float],
        right_arm_present_pos: dict[str, float],
        left_arm_goal_pos: dict[str, float],
        right_arm_goal_pos: dict[str, float],
        overcurrent_motors: list[str]
    ) -> tuple[dict[str, float], dict[str, float]]:
        """
        Handle overcurrent motors by replacing their goal positions with present positions.

        Args:
            left_arm_goal_pos: Dictionary of left arm goal positions with keys like "left_arm_shoulder_pan.pos"
            right_arm_goal_pos: Dictionary of right arm goal positions with keys like "right_arm_shoulder_pan.pos"
            overcurrent_motors: List of motor names that are over current (e.g., ["left_arm_shoulder_pan", "right_arm_elbow_flex"])

        Returns:
            tuple: (modified_left_arm_goal_pos, modified_right_arm_goal_pos) with present positions for overcurrent motors
        """
        if not overcurrent_motors:
            return left_arm_goal_pos, right_arm_goal_pos

        # Create copies of the goal positions to modify
        modified_left_arm_goal_pos = left_arm_goal_pos.copy()
        modified_right_arm_goal_pos = right_arm_goal_pos.copy()

        # Replace goal positions with present positions for overcurrent motors
        for motor_name in overcurrent_motors:
            if motor_name.startswith("left_arm_"):
                # Convert motor name to goal position key format
                goal_key = f"{motor_name}.pos"
                if goal_key in modified_left_arm_goal_pos:
                    modified_left_arm_goal_pos[goal_key] = left_arm_present_pos[motor_name]
                    logger.warning(f"Replaced goal position for {motor_name} with present position: {left_arm_present_pos[motor_name]}")
            elif motor_name.startswith("right_arm_"):
                # Convert motor name to goal position key format
                goal_key = f"{motor_name}.pos"
                if goal_key in modified_right_arm_goal_pos:
                    modified_right_arm_goal_pos[goal_key] = right_arm_present_pos[motor_name]
                    logger.warning(f"Replaced goal position for {motor_name} with present position: {right_arm_present_pos[motor_name]}")

        # Sync write the modified goal positions
        left_arm_goal_pos_raw = {k.replace(".pos", ""): v for k, v in modified_left_arm_goal_pos.items()}
        self.left_arm_bus.sync_write("Goal_Position", left_arm_goal_pos_raw)
        right_arm_goal_pos_raw = {k.replace(".pos", ""): v for k, v in modified_right_arm_goal_pos.items()}
        self.right_arm_bus.sync_write("Goal_Position", right_arm_goal_pos_raw)

        return modified_left_arm_goal_pos, modified_right_arm_goal_pos

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.left_arm_bus.disconnect(self.config.disable_torque_on_disconnect)
        self.right_arm_bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")

