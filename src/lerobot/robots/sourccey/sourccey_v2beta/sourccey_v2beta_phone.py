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

from lerobot.common.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.common.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_sourccey_v2beta_phone import SourcceyV2BetaPhoneConfig

logger = logging.getLogger(__name__)


class SourcceyV2BetaPhone(Robot):
    """
    Sourccey V2Beta Phone Teleoperation Robot
    Direct connection to Sourccey arm for phone teleoperation
    Uses motors 7-12
    """

    config_class = SourcceyV2BetaPhoneConfig
    name = "sourccey_v2beta"

    def __init__(self, config: SourcceyV2BetaPhoneConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
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

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @cached_property
    def observation_features(self) -> dict[str, type]:
        return self._motors_ft

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        """
        We assume that at connection time, arm is in a rest position,
        and torque can be safely disabled to run calibration.
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        if not self.is_calibrated and calibrate:
            self.calibrate()

        self.configure()
        
        # Safety check: Move motors to midpoint if they're outside calibration range
        self._safety_move_to_midpoint()
        
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

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

        # Read arm position
        start = time.perf_counter()
        obs_dict = self.bus.sync_read("Present_Position")
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

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

        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # DEBUG: Log what we're trying to send
        logger.info(f"🎯 Robot trying to send goal_pos: {goal_pos}")
        
        # DEBUG: Log calibration info
        if hasattr(self, 'bus') and hasattr(self.bus, 'calibration'):
            logger.info(f"🔧 Motor calibration ranges:")
            for motor, cal in self.bus.calibration.items():
                logger.info(f"  {motor}: min={cal.range_min}, max={cal.range_max}, mid={(cal.range_min + cal.range_max) / 2}")
        
        # DEBUG: Log raw present positions
        if hasattr(self, 'bus'):
            try:
                raw_positions = self.bus.sync_read("Present_Position", normalize=False)
                logger.info(f"🔧 Raw encoder positions: {raw_positions}")
            except Exception as e:
                logger.warning(f"Could not read raw positions: {e}")

        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal positions to motors
        start = time.perf_counter()
        self.bus.sync_write("Goal_Position", goal_pos)
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} write state: {dt_ms:.1f}ms")

        # Return the action that was actually sent
        return {f"{motor}.pos": goal_pos[motor] for motor in goal_pos}

    def disconnect(self) -> None:
        """Disconnect from the robot."""
        if self.config.disable_torque_on_disconnect:
            try:
                self.bus.disable_torque()
            except Exception as e:
                logger.warning(f"Failed to disable torque on disconnect: {e}")

        try:
            self.bus.disconnect()
        except Exception as e:
            logger.warning(f"Failed to disconnect bus: {e}")

        logger.info(f"{self} disconnected.")

    def _safety_move_to_midpoint(self) -> None:
        """Safety check: Move motors to midpoint if they're outside calibration range."""
        if not hasattr(self, 'bus') or not hasattr(self.bus, 'calibration'):
            logger.warning("No calibration available for safety check")
            return
        
        logger.info("🔒 Running safety check: Moving motors to midpoint if out of range...")
        
        try:
            # Read current raw positions
            raw_positions = self.bus.sync_read("Present_Position", normalize=False)
            
            motors_to_move = []
            midpoints = {}
            
            # Check each motor
            for motor_name, motor in self.bus.motors.items():
                if motor_name not in self.bus.calibration:
                    logger.warning(f"No calibration for {motor_name}, skipping")
                    continue
                
                cal = self.bus.calibration[motor_name]
                current_pos = raw_positions.get(motor_name)
                
                if current_pos is None:
                    logger.warning(f"Could not read position for {motor_name}")
                    continue
                
                # Check if position is outside calibration range
                if current_pos < cal.range_min or current_pos > cal.range_max:
                    logger.warning(
                        f"⚠️  {motor_name} is out of range: "
                        f"position={current_pos}, range=[{cal.range_min}, {cal.range_max}]"
                    )
                    
                    # Calculate midpoint
                    midpoint = (cal.range_min + cal.range_max) // 2
                    motors_to_move.append(motor_name)
                    midpoints[motor_name] = midpoint
                    
                    logger.info(f"  → Will move {motor_name} to midpoint: {midpoint}")
                else:
                    logger.info(f"✅ {motor_name} is within range: {current_pos}")
            
            # Move out-of-range motors to midpoint
            if motors_to_move:
                logger.info(f"🔧 Moving {len(motors_to_move)} motors to midpoint...")
                
                # Create goal positions dict
                goal_positions = {}
                for motor_name in motors_to_move:
                    goal_positions[motor_name] = midpoints[motor_name]
                
                # Move motors to midpoint
                self.bus.sync_write("Goal_Position", goal_positions)
                
                # Wait a moment for motors to move
                time.sleep(2.0)
                
                # Verify the move
                new_positions = self.bus.sync_read("Present_Position", normalize=False)
                for motor_name in motors_to_move:
                    new_pos = new_positions.get(motor_name)
                    if new_pos is not None:
                        logger.info(f"✅ {motor_name} moved to: {new_pos}")
                    else:
                        logger.warning(f"❌ Could not verify position for {motor_name}")
                
                logger.info("🔒 Safety check complete!")
            else:
                logger.info("✅ All motors are within calibration range")
                
        except Exception as e:
            logger.error(f"❌ Safety check failed: {e}")
            logger.warning("Continuing without safety correction...") 