#!/usr/bin/env python3
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

"""
Helper to calibrate a 2-motor leader-follower setup.

This script calibrates two motors where:
- Motor ID 1 is the leader (teleop)
- Motor ID 2 is the follower (robot)

The script automatically detects if motors are single or double shaft and calibrates accordingly.
For double shaft motors, calibration allows up to 7 rotations using extended position tracking.
For single shaft motors, calibration is limited to 1 rotation.

The follower motor movement is proportional to the leader motor movement.

Example:

```shell
# Windows
python start/colin/calibrate_2_motors.py \
    --port=COM13 \
    --leader_model=sts3215 \
    --follower_model=sts3235

# Linux/Mac
python start/colin/calibrate_2_motors.py \
    --port=COM13 \
    --leader_model=sts3215 \
    --follower_model=sts3235
```
"""

import logging
import time
from dataclasses import asdict, dataclass
from pprint import pformat
from typing import Literal

import draccus

from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.common.motors.feetech import FeetechMotorsBus
from lerobot.common.utils.utils import init_logging, enter_pressed


@dataclass
class TwoMotorCalibrateConfig:
    port: str
    leader_model: str
    follower_model: str
    motor_type: Literal["dynamixel", "feetech"] = "feetech"
    leader_id: int = 1
    follower_id: int = 2
    baudrate: int = 115200

    def __post_init__(self):
        if self.leader_id == self.follower_id:
            raise ValueError("Leader and follower motor IDs must be different.")


def detect_motor_type(motor_id: int, bus, motor_model: str) -> Literal["single", "double"]:
    """
    Detect if a motor is single or double shaft based on its model and capabilities.
    """
    # Known multi-shaft models
    multi_shaft_models = ["sts3235", "sts3250", "sts2057"]

    # Check if it's a known multi-shaft model
    if motor_model.lower() in multi_shaft_models:
        return "double"

    # For Feetech motors, we rely on the model name since they handle multi-turn naturally
    # in their 16-bit position range (0-65535)
    logging.info(f"Motor {motor_id} ({motor_model}) detected as single shaft (Feetech motors handle multi-turn naturally)")
    return "single"


def get_calibration_range(motor_type: Literal["single", "double"]) -> int:
    """
    Get the calibration range in degrees for a motor type.
    """
    if motor_type == "single":
        return 360  # One full rotation
    else:
        return 2520  # 7 full rotations (7 * 360)


def proportional_movement(leader_position: float, leader_range: int, follower_range: int) -> float:
    """
    Calculate proportional follower position based on leader position.
    """
    leader_percentage = leader_position / leader_range
    return leader_percentage * follower_range


class FeetechMultiTurnTracker:
    """
    Handles multi-turn position tracking for Feetech motors.

    Feetech motors use 16-bit position values (0-65535) and can track multiple rotations.
    For multi-shaft motors, we need to use extended position tracking to handle
    positions beyond the basic 12-bit encoder range (0-4095).
    """

    def __init__(self, bus, motor_name: str):
        self.bus = bus
        self.motor_name = motor_name
        self.motor_id = bus.motors[motor_name].id
        self.model = bus.motors[motor_name].model

        # Feetech motor resolution (12-bit encoder, but can track multiple rotations)
        self.max_resolution = 4095  # 12-bit encoder max value
        self.max_extended = 28665  # 7 rotations * 4095 = 28665 (maximum allowed)

        # Track the number of full rotations
        self.rotation_count = 0
        self.last_position = 0
        self.position_wraps = 0

        # Initialize position tracking
        self._initialize_position_tracking()

    def _initialize_position_tracking(self):
        """Initialize position tracking by reading current position."""
        self.last_position = self.bus.read("Present_Position", self.motor_name, normalize=False)
        logging.info(f"Initialized {self.motor_name} position tracking at: {self.last_position}")

    def read_extended_position(self) -> int:
        """
        Read the current extended position that accounts for multiple rotations.

        Returns:
            int: Extended position value that can go beyond 4095 for multi-turn motors
        """
        current_position = self.bus.read("Present_Position", self.motor_name, normalize=False)

        # Detect position wrapping (when position jumps from high to low or vice versa)
        position_diff = current_position - self.last_position

        # If the position difference is large, it might be a wrap
        if abs(position_diff) > 3000:  # Threshold for detecting wraps
            if position_diff > 0:
                # Wrapped from high to low (e.g., 4095 -> 0)
                self.position_wraps -= 1
            else:
                # Wrapped from low to high (e.g., 0 -> 4095)
                self.position_wraps += 1

            logging.debug(f"{self.motor_name} position wrap detected: {self.last_position} -> {current_position}, wraps: {self.position_wraps}")

        # Calculate extended position
        extended_position = current_position + (self.position_wraps * 4096)

        # Handle wrapping: if position goes above 28665, wrap back to 0
        if extended_position > 28665:
            extended_position = extended_position % 28666  # Wrap back to 0-28665 range

        # Ensure non-negative values
        if extended_position < 0:
            extended_position = 0

        # Update last position
        self.last_position = current_position

        return extended_position

    def write_extended_position(self, extended_position: int):
        """
        Write an extended position that can go beyond 4095.

        Args:
            extended_position: Target position that can be beyond 4095
        """
        # For Feetech motors, we need to handle the position wrapping
        # The motor can only accept positions in the 0-65535 range

        # Ensure the position is within the valid range (0 to 4095*7 = 28665)
        if extended_position < 0:
            extended_position = 0
        elif extended_position > self.max_extended:
            logging.warning(f"Requested position {extended_position} exceeds maximum {self.max_extended}, limiting to maximum")
            extended_position = self.max_extended

        # Write the position directly (Feetech motors handle the wrapping internally)
        self.bus.write("Goal_Position", self.motor_name, extended_position, normalize=False)

        logging.debug(f"Wrote extended position {extended_position} to {self.motor_name}")

    def get_position_info(self) -> dict:
        """
        Get detailed position information for debugging.

        Returns:
            dict: Position information including raw, extended, and wrap count
        """
        raw_position = self.bus.read("Present_Position", self.motor_name, normalize=False)
        extended_position = self.read_extended_position()

        return {
            "raw_position": raw_position,
            "extended_position": extended_position,
            "wraps": self.position_wraps,
            "rotation_count": extended_position // 4096
        }


@draccus.wrap()
def calibrate_2_motors(cfg: TwoMotorCalibrateConfig):
    init_logging()
    logging.info("Starting 2-motor calibration")
    logging.info(pformat(asdict(cfg)))

    # Create motor configurations
    motors = {
        "leader": Motor(
            id=cfg.leader_id,
            model=cfg.leader_model,
            norm_mode=MotorNormMode.RANGE_0_100
        ),
        "follower": Motor(
            id=cfg.follower_id,
            model=cfg.follower_model,
            norm_mode=MotorNormMode.RANGE_0_100
        )
    }

    # Create motor bus - ONE port, TWO motors
    bus = FeetechMotorsBus(port=cfg.port, motors=motors)

    try:
        # Connect to motors
        logging.info("Connecting to motors...")
        bus.connect()
        logging.info("Successfully connected to motors")

        # Enable torque on both motors initially
        bus.enable_torque(["leader", "follower"])
        logging.info("Torque enabled on both motors")

        # Detect motor types
        logging.info("Detecting motor types...")
        leader_type = detect_motor_type(cfg.leader_id, bus, cfg.leader_model)
        follower_type = detect_motor_type(cfg.follower_id, bus, cfg.follower_model)

        logging.info(f"Leader motor (ID {cfg.leader_id}): {leader_type} shaft")
        logging.info(f"Follower motor (ID {cfg.follower_id}): {follower_type} shaft")

        # Get calibration ranges
        leader_range = get_calibration_range(leader_type)
        follower_range = get_calibration_range(follower_type)

        logging.info(f"Leader motor range: {leader_range} degrees")
        logging.info(f"Follower motor range: {follower_range} degrees")

        # Move to home position (0%) - use unnormalized positions since no calibration yet
        logging.info("Moving to home position...")
        bus.write("Goal_Position", "leader", 2048, normalize=False)
        if follower_type == "double":
            # For multi-shaft motors, start at 4095*3.5 = 14332
            bus.write("Goal_Position", "follower", 14332, normalize=False)
        else:
            bus.write("Goal_Position", "follower", 2048, normalize=False)  # Center position
        time.sleep(2)

        # Disable torque to allow manual movement
        logging.info("Disabling torque to allow manual movement...")
        bus.disable_torque(["leader", "follower"])

        # Set operating modes for motors
        logging.info("Setting operating modes...")
        logging.info("Setting both motors to Position Mode (Feetech motors handle multi-turn naturally)")
        bus.write("Operating_Mode", "leader", 0)  # Position Mode
        bus.write("Operating_Mode", "follower", 0)  # Position Mode

        # Set homing offsets to center the motors
        logging.info("Setting homing offsets to center the motors...")
        input("Move both motors to the middle of their ranges and press Enter...")
        homing_offsets = bus.set_half_turn_homings()
        logging.info(f"Homing offsets set: {homing_offsets}")

        # Initialize multi-turn trackers for extended position tracking
        leader_tracker = FeetechMultiTurnTracker(bus, "leader")
        follower_tracker = FeetechMultiTurnTracker(bus, "follower")

        # Record ranges of motion using the proper method
        logging.info("Recording ranges of motion...")
        logging.info("Move the leader motor through its entire range of motion")
        logging.info("The system will automatically track min/max values")
        logging.info("Press Enter when done...")

        leader_mins, leader_maxes = bus.record_ranges_of_motion(["leader"], display_values=True)
        leader_min = leader_mins["leader"]
        leader_max = leader_maxes["leader"]

        logging.info(f"Leader range recorded: {leader_min} to {leader_max}")

        # For multi-shaft motors, set proper position limits and test extended range
        if follower_type == "double":
            logging.info("Setting multi-shaft follower motor for extended position tracking...")

            # Set position limits to allow full range
            # Feetech motors typically use 12-bit resolution (0-4095) for position limits
            # But can track multiple rotations within their 16-bit range
            bus.write("Min_Position_Limit", "follower", 0, normalize=False)
            bus.write("Max_Position_Limit", "follower", 4095, normalize=False)  # Standard 12-bit range

            # Verify the limits were set correctly
            min_limit = bus.read("Min_Position_Limit", "follower", normalize=False)
            max_limit = bus.read("Max_Position_Limit", "follower", normalize=False)
            logging.info(f"Follower motor limits set: {min_limit} to {max_limit}")

            # Test extended position movement
            logging.info("Testing extended position movement...")
            current_info = follower_tracker.get_position_info()
            logging.info(f"Current follower position info: {current_info}")

            # Test movement to extended positions
            test_positions = [8190, 12285, 16380]  # 2, 3, 4 rotations
            for test_pos in test_positions:
                logging.info(f"Testing movement to position {test_pos} ({test_pos//4096} rotations)...")
                follower_tracker.write_extended_position(test_pos)
                time.sleep(2)

                new_info = follower_tracker.get_position_info()
                logging.info(f"Follower moved to: {new_info}")

                if new_info["extended_position"] >= 4095:
                    logging.info(f"✅ Extended position movement confirmed for {test_pos//4096} rotations!")
                else:
                    logging.warning(f"⚠️ Extended position movement may not be working for {test_pos//4096} rotations")

        logging.info("Now move the follower motor through its ENTIRE multi-turn range of motion")
        if follower_type == "double":
            logging.info("For multi-shaft motor: move through multiple full rotations in both directions")
            logging.info("The motor can now track positions beyond 4095 for multiple rotations")
        logging.info("Press Enter when done...")

        # Disable torque on follower motor to allow manual movement during calibration
        logging.info("Disabling torque on follower motor for manual calibration...")
        bus.disable_torque(["follower"])

        # Use the multi-turn tracker to record extended ranges
        follower_min = float('inf')
        follower_max = float('-inf')

        print("\n" + "-" * 42)
        print("NAME            |    MIN |    POS |    MAX")

        while True:
            try:
                # Read extended positions
                leader_extended = leader_tracker.read_extended_position()
                follower_extended = follower_tracker.read_extended_position()

                # Update min/max for follower (ensure non-negative values)
                if follower_extended >= 0:  # Only record non-negative positions
                    follower_min = min(follower_min, follower_extended)
                    follower_max = max(follower_max, follower_extended)

                # Display current positions
                print(f"{'follower':<15} | {follower_min:>6} | {follower_extended:>6} | {follower_max:<6}")

                # Check if user pressed Enter
                if enter_pressed():
                    break

                time.sleep(0.1)  # Small delay to avoid overwhelming the display

            except KeyboardInterrupt:
                break

        logging.info(f"Follower range recorded: {follower_min} to {follower_max}")

        # Check if the follower actually used its full range
        if follower_type == "double" and follower_max < 4095:
            logging.warning(f"⚠️ Multi-shaft follower only reached {follower_max}, expected up to 28665")
            logging.warning("This may indicate the motor wasn't moved through its full range")
            logging.warning("Or the extended position tracking isn't working properly")

        # Validate ranges
        leader_range = leader_max - leader_min
        follower_range = follower_max - follower_min

        logging.info(f"Leader range: {leader_range}")
        logging.info(f"Follower range: {follower_range}")

        if leader_range == 0:
            raise ValueError(f"Leader motor has no movement range: min={leader_min}, max={leader_max}")

        if follower_range <= 0:
            logging.warning(f"Follower motor has no movement range: min={follower_min}, max={follower_max}")
            logging.warning("This may indicate the follower motor is stuck or not moving")
            logging.warning("Proceeding with follower range = 1 to avoid division by zero")
            follower_max = follower_min + 1  # Add 1 to avoid division by zero
            follower_range = follower_max - follower_min  # Update the range

        # For multi-shaft motors, adjust the range to center around 4095*3.5 (14,332)
        if follower_type == "double":
            logging.info("Adjusting multi-shaft follower range to center around 4095*3.5...")

            # Calculate the desired center position (4095*3.5 = 14,332)
            desired_center = 14332

            # Calculate the current range and center
            current_center = (follower_min + follower_max) / 2
            current_range = follower_max - follower_min

            # Ensure the range doesn't exceed 4095*7 (28,665) total range
            max_total_range = 28665  # 4095 * 7
            if current_range > max_total_range:
                logging.warning(f"Follower range {current_range} exceeds maximum {max_total_range}, limiting to maximum")
                current_range = max_total_range

            # Calculate new min/max centered around 4095*3.5
            new_min = max(0, int(desired_center - current_range / 2))  # Ensure non-negative
            new_max = min(max_total_range, int(desired_center + current_range / 2))  # Ensure within max range

            # Double-check that we don't have negative values
            if new_min < 0:
                logging.warning(f"Calculated minimum {new_min} is negative, setting to 0")
                new_min = 0

            logging.info(f"Original range: {follower_min} to {follower_max} (center: {current_center})")
            logging.info(f"Adjusted range: {new_min} to {new_max} (center: {desired_center})")

            # Update the follower range values
            follower_min = new_min
            follower_max = new_max
            follower_range = follower_max - follower_min

        # Re-enable torque for automatic movement testing
        logging.info("Re-enabling torque for automatic movement testing...")
        bus.enable_torque(["leader", "follower"])

        logging.info(f"Leader maximum: {leader_max:.2f}%")
        logging.info(f"Follower maximum: {follower_max:.2f}%")

        # Test proportional movement using extended positions
        logging.info("Testing proportional movement...")
        test_positions = [25, 50, 75]

        for pos in test_positions:
            logging.info(f"Testing {pos}% position...")

            # Calculate unnormalized positions based on min/max ranges
            leader_pos_raw = int(leader_min + (pos / 100.0) * leader_range)
            follower_pos_raw = int(follower_min + (pos / 100.0) * follower_range)

            # Move leader to test position
            bus.write("Goal_Position", "leader", leader_pos_raw, normalize=False)
            time.sleep(1)

            # Move follower to proportional position using extended tracking
            if follower_type == "double":
                follower_tracker.write_extended_position(follower_pos_raw)
            else:
                bus.write("Goal_Position", "follower", follower_pos_raw, normalize=False)

            # Read actual positions
            actual_leader = bus.read("Present_Position", "leader", normalize=False)
            if follower_type == "double":
                actual_follower = follower_tracker.read_extended_position()
            else:
                actual_follower = bus.read("Present_Position", "follower", normalize=False)

            # Convert to percentages for display
            leader_pct = ((actual_leader - leader_min) / leader_range) * 100
            follower_pct = ((actual_follower - follower_min) / follower_range) * 100

            logging.info(f"  Leader: {leader_pct:.2f}% -> Follower: {follower_pct:.2f}%")
            time.sleep(2)

        # Return to home
        logging.info("Returning to home position...")
        bus.write("Goal_Position", "leader", 2048, normalize=False)  # Center position
        if follower_type == "double":
            follower_tracker.write_extended_position(2048)
        else:
            bus.write("Goal_Position", "follower", 2048, normalize=False)  # Center position
        time.sleep(2)

        # Ensure follower position limits are non-negative (motors don't support negative limits)
        if follower_min < 0:
            logging.warning(f"Follower minimum position {follower_min} is negative, adjusting to 0")
            follower_min = 0
            follower_range = follower_max - follower_min  # Recalculate range

        # Create calibration data
        calibration = {
            "leader": MotorCalibration(
                id=cfg.leader_id,
                drive_mode=0,  # Default drive mode
                homing_offset=int(homing_offsets["leader"]),
                range_min=int(leader_min),
                range_max=int(leader_max)
            ),
            "follower": MotorCalibration(
                id=cfg.follower_id,
                drive_mode=0,  # Default drive mode
                homing_offset=int(homing_offsets["follower"]),
                range_min=int(follower_min),
                range_max=int(follower_max)
            )
        }

        # Save calibration
        logging.info("Saving calibration...")
        bus.write_calibration(calibration)
        logging.info("Calibration saved successfully!")

        logging.info("Calibration summary:")
        logging.info(f"  Leader motor (ID {cfg.leader_id}): {leader_type} shaft, range {leader_min:.1f}% - {leader_max:.1f}%")
        logging.info(f"  Follower motor (ID {cfg.follower_id}): {follower_type} shaft, range {follower_min:.1f}% - {follower_max:.1f}%")
        logging.info("  Proportional movement: follower moves proportionally to leader")

        # Manual test mode
        logging.info("\n" + "="*60)
        logging.info("MANUAL TEST MODE")
        logging.info("="*60)
        logging.info("Now you can test manual movement:")
        logging.info("1. Leader motor torque will be disabled (you can move it manually)")
        logging.info("2. Follower motor will follow proportionally")
        if follower_type == "double":
            logging.info("3. Multi-shaft follower uses extended position tracking (0-28665)")
            logging.info("4. Position values can go beyond 4095 for multiple rotations (max 7 rotations)")
            logging.info("5. Center position is at 4095×3.5 = 14332")
            logging.info("6. Positions wrap back to 0 when exceeding 28665")
        logging.info("7. Press Enter to read positions, type 'quit' to exit")
        logging.info("="*60)

        # Disable leader torque, enable follower torque
        bus.disable_torque(["leader"])
        bus.enable_torque(["follower"])

        while True:
            try:
                user_input = input("\nPress Enter to read positions (or type 'quit'): ").strip().lower()

                if user_input == 'quit':
                    break

                # Read current positions using extended tracking
                leader_pos = bus.read("Present_Position", "leader", normalize=False)
                if follower_type == "double":
                    follower_pos = follower_tracker.read_extended_position()
                    follower_info = follower_tracker.get_position_info()
                else:
                    follower_pos = bus.read("Present_Position", "follower", normalize=False)
                    follower_info = {"raw_position": follower_pos, "extended_position": follower_pos, "wraps": 0, "rotation_count": 0}

                # Calculate leader percentage
                leader_pct = ((leader_pos - leader_min) / (leader_max - leader_min)) * 100
                leader_pct = max(0, min(100, leader_pct))

                # Calculate target follower position based on leader percentage
                target_follower_pos = int(follower_min + (leader_pct / 100.0) * (follower_max - follower_min))

                # Calculate current follower percentage based on adjusted range
                follower_pct = ((follower_pos - follower_min) / (follower_max - follower_min)) * 100
                follower_pct = max(0, min(100, follower_pct))

                logging.info(f"Leader: {leader_pos} ({leader_pct:.1f}%)")
                if follower_type == "double":
                    logging.info(f"Follower: {follower_pos} ({follower_pct:.1f}%) [raw: {follower_info['raw_position']}, wraps: {follower_info['wraps']}, rotations: {follower_info['rotation_count']}]")
                else:
                    logging.info(f"Follower: {follower_pos} ({follower_pct:.1f}%)")

                # Move follower to proportional position
                if abs(target_follower_pos - follower_pos) > 2:
                    logging.info(f"Moving follower to: {target_follower_pos}")
                    if follower_type == "double":
                        follower_tracker.write_extended_position(target_follower_pos)
                    else:
                        bus.write("Goal_Position", "follower", target_follower_pos, normalize=False)
                    time.sleep(0.1)

                    # Read new position to verify movement
                    if follower_type == "double":
                        new_follower_pos = follower_tracker.read_extended_position()
                        new_follower_info = follower_tracker.get_position_info()
                        logging.info(f"Follower moved to: {new_follower_pos} [raw: {new_follower_info['raw_position']}, wraps: {new_follower_info['wraps']}]")
                    else:
                        new_follower_pos = bus.read("Present_Position", "follower", normalize=False)
                        logging.info(f"Follower moved to: {new_follower_pos}")
                else:
                    logging.info(f"Follower already at target position")

            except KeyboardInterrupt:
                break
            except Exception as e:
                logging.error(f"Error: {e}")

        logging.info("Manual test completed!")

    except Exception as e:
        logging.error(f"Calibration failed: {e}")
        raise
    finally:
        # Disable torque and disconnect
        logging.info("Disconnecting...")
        bus.disable_torque(["leader", "follower"])
        bus.disconnect()


if __name__ == "__main__":
    calibrate_2_motors()
