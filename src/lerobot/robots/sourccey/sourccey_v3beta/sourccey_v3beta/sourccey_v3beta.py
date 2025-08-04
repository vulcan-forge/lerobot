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
from typing import Any, Optional

import numpy as np
import threading

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.constants import OBS_IMAGES, OBS_STATE
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dc_pwm.dc_pwm import PWMDCMotorsController, PWMProtocolHandler
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.motors.dc_motors_controller import DCMotor, MotorNormMode

from lerobot.robots.robot import Robot
from lerobot.robots.sourccey.sourccey_v3beta.sourccey_v3beta_follower.config_sourccey_v3beta_follower import SourcceyV3BetaFollowerConfig
from lerobot.robots.sourccey.sourccey_v3beta.sourccey_v3beta_follower.sourccey_v3beta_follower import SourcceyV3BetaFollower
from lerobot.robots.utils import ensure_safe_goal_position
from .config_sourccey_v3beta import SourcceyV3BetaConfig

logger = logging.getLogger(__name__)


class SourcceyV3Beta(Robot):
    """
    The robot includes a four mecanum wheel mobile base, 1 DC actuator, and 2 remote follower arms.
    The leader arm is connected locally (on the laptop) and its joint positions are recorded and then
    forwarded to the remote follower arm (after applying a safety clamp).
    In parallel, keyboard teleoperation is used to generate raw velocity commands for the wheels.
    """

    config_class = SourcceyV3BetaConfig
    name = "sourccey_v3beta"

    def __init__(self, config: SourcceyV3BetaConfig):
        super().__init__(config)
        self.config = config

        left_arm_config = SourcceyV3BetaFollowerConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_port,
            disable_torque_on_disconnect=config.left_arm_disable_torque_on_disconnect,
            max_relative_target=config.left_arm_max_relative_target,
            use_degrees=config.left_arm_use_degrees,
            cameras={},
        )
        right_arm_config = SourcceyV3BetaFollowerConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_port,
            orientation="right",
            disable_torque_on_disconnect=config.right_arm_disable_torque_on_disconnect,
            max_relative_target=config.right_arm_max_relative_target,
            use_degrees=config.right_arm_use_degrees,
            cameras={},
        )

        self.left_arm = SourcceyV3BetaFollower(left_arm_config)
        self.right_arm = SourcceyV3BetaFollower(right_arm_config)
        self.cameras = make_cameras_from_configs(config.cameras)
        self.dc_motors_controller = PWMDCMotorsController(
            config=self.config.dc_motors
        )

    @property
    def _state_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.left_arm.bus.motors} | {
            f"{motor}.pos": float for motor in self.right_arm.bus.motors
        }

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
        return (
            self.left_arm.is_connected and
            self.right_arm.is_connected and
            self.dc_motors_controller.is_connected and
            all(cam.is_connected for cam in self.cameras.values())
        )

    def connect(self, calibrate: bool = True) -> None:
        self.left_arm.connect(calibrate)
        self.right_arm.connect(calibrate)

        # Connect DC motors
        self.dc_motors_controller.connect()

        for cam in self.cameras.values():
            cam.connect()

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def auto_calibrate(self, full_reset: bool = False) -> None:
        """
        Auto-calibrate both arms simultaneously using threading.
        """
        # Create threads for each arm
        left_thread = threading.Thread(
            target=self.left_arm.auto_calibrate,
            kwargs={"reversed": False, "full_reset": full_reset}
        )
        right_thread = threading.Thread(
            target=self.right_arm.auto_calibrate,
            kwargs={"reversed": True, "full_reset": full_reset}
        )

        # Start both threads
        left_thread.start()
        right_thread.start()

        # Wait for both threads to complete
        left_thread.join()
        right_thread.join()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        self.left_arm.setup_motors()
        self.right_arm.setup_motors()

    def get_observation(self) -> dict[str, Any]:
        try:
            obs_dict = {}

            left_obs = self.left_arm.get_observation()
            obs_dict.update({f"left_{key}": value for key, value in left_obs.items()})

            right_obs = self.right_arm.get_observation()
            obs_dict.update({f"right_{key}": value for key, value in right_obs.items()})

            for cam_key, cam in self.cameras.items():
                obs_dict[cam_key] = cam.async_read()

            return obs_dict
        except Exception as e:
            print(f"Error getting observation: {e}")
            return {}

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        try:
            left_action = {
                key.removeprefix("left_"): value for key, value in action.items() if key.startswith("left_")
            }
            right_action = {
                key.removeprefix("right_"): value for key, value in action.items() if key.startswith("right_")
            }

            send_action_left = self.left_arm.send_action(left_action)
            send_action_right = self.right_arm.send_action(right_action)

            prefixed_send_action_left = {f"left_{key}": value for key, value in send_action_left.items()}
            prefixed_send_action_right = {f"right_{key}": value for key, value in send_action_right.items()}

            return {**prefixed_send_action_left, **prefixed_send_action_right}
        except Exception as e:
            print(f"Error sending action: {e}")
            return {}

    def disconnect(self):
        self.left_arm.disconnect()
        self.right_arm.disconnect()
        self.dc_motors_controller.disconnect()

        for cam in self.cameras.values():
            cam.disconnect()

    # High precision wheel control methods
    def set_mecanum_velocity(self, linear_x: float, linear_y: float, angular_z: float):
        """
        Set mecanum wheel velocities for holonomic movement (high precision).

        Args:
            linear_x: Forward/backward velocity (-1 to 1)
            linear_y: Left/right velocity (-1 to 1)
            angular_z: Rotational velocity (-1 to 1)
        """
        # Mecanum wheel kinematic equations
        wheel_radius = 0.05  # 5cm wheel radius
        wheelbase = 0.2      # 20cm between wheels
        track_width = 0.2    # 20cm track width

        # Calculate individual wheel velocities
        v_fl = linear_x + linear_y + angular_z * (wheelbase + track_width) / 2
        v_fr = linear_x - linear_y - angular_z * (wheelbase + track_width) / 2
        v_rl = linear_x - linear_y + angular_z * (wheelbase + track_width) / 2
        v_rr = linear_x + linear_y - angular_z * (wheelbase + track_width) / 2

        # Normalize to [-1, 1] range
        max_vel = max(abs(v_fl), abs(v_fr), abs(v_rl), abs(v_rr))
        if max_vel > 1.0:
            v_fl /= max_vel
            v_fr /= max_vel
            v_rl /= max_vel
            v_rr /= max_vel

        # Set wheel velocities (hardware PWM - high precision)
        self.dc_motors_controller.set_velocity("front_left", v_fl)
        self.dc_motors_controller.set_velocity("front_right", v_fr)
        self.dc_motors_controller.set_velocity("rear_left", v_rl)
        self.dc_motors_controller.set_velocity("rear_right", v_rr)

    # Lower precision actuator control methods
    def set_actuator_velocity(self, velocity: float):
        """
        Set actuator velocity (software PWM - lower precision).

        Args:
            velocity: Actuator velocity (-1 to 1, negative=retract, positive=extend)
        """
        self.dc_motors_controller.set_velocity("actuator", velocity)

    def set_actuator_position(self, position: float):
        """
        Set actuator position (0 to 1, simplified control).

        Args:
            position: Actuator position (0=fully retracted, 1=fully extended)
        """
        self.dc_motors_controller.set_position("actuator", position)

    def get_actuator_position(self) -> float:
        """Get current actuator position estimate."""
        return self.dc_motors_controller.get_position("actuator") or 0.0

    def stop_motors(self):
        """Stop all motors."""
        # Stop wheels
        self.dc_motors_controller.set_velocity("front_left", 0.0)
        self.dc_motors_controller.set_velocity("front_right", 0.0)
        self.dc_motors_controller.set_velocity("rear_left", 0.0)
        self.dc_motors_controller.set_velocity("rear_right", 0.0)
        # Stop actuator
        self.dc_motors_controller.set_velocity("actuator", 0.0)

    def stop_wheels_only(self):
        """Stop only the wheels, leave actuator running."""
        self.dc_motors_controller.set_velocity("front_left", 0.0)
        self.dc_motors_controller.set_velocity("front_right", 0.0)
        self.dc_motors_controller.set_velocity("rear_left", 0.0)
        self.dc_motors_controller.set_velocity("rear_right", 0.0)

