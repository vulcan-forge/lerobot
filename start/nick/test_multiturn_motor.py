#!/usr/bin/env python3
"""
Simple test script to move a single motor between positions 1000, 1500, and back to 1000.
"""

import time
import sys
from pathlib import Path

from lerobot.common.motors.motors_bus import MotorCalibration

# Add the project root to the path so we can import lerobot
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.feetech import FeetechMotorsBus, OperatingMode

def read_motor_position():
    """Simple function to read the current position of the motor."""

    # Motor configuration for STS3235
    motor_id = 1
    motor_model = "sts3235"
    gear_ratio = 1.0
    port = "COM36"

    motors = {
        "test_multiturn_motor": Motor(motor_id, motor_model, MotorNormMode.RANGE_M100_100, gear_ratio=gear_ratio),
    }

    bus = FeetechMotorsBus(
        port=port,
        motors=motors,
        calibration={
            "test_multiturn_motor": MotorCalibration(
                id=motor_id,
                drive_mode=0,
                homing_offset=0,
                range_min=0,
                range_max=28672
            )
        }
    )

    try:
        # Connect to the motor
        print("Connecting to motor...")
        bus.connect(handshake=False)
        print("✓ Connected successfully")

        # max_res = 4096 * 3
        # min_range = 0
        # max_range = max_res - 1
        # bus.write("Homing_Offset", "test_multiturn_motor", 0)
        # bus.write("Min_Position_Limit", "test_multiturn_motor", min_range)
        # bus.write("Max_Position_Limit", "test_multiturn_motor", max_range)
        bus.write("Homing_Offset", "test_multiturn_motor", 0)
        bus.write("Min_Position_Limit", "test_multiturn_motor", 0)
        bus.write("Max_Position_Limit", "test_multiturn_motor", 12287)

        # Read current position

        position = bus.read("Present_Position", "test_multiturn_motor", normalize=False)
        print()
        print(f"Current position 1: {position}")
        print()

        bus.write("Goal_Position", "test_multiturn_motor", 1000, normalize=False)
        time.sleep(2)

        position = bus.read("Present_Position", "test_multiturn_motor", normalize=False)
        print()
        print(f"Current position 2: {position}")
        print()

        bus.write("Homing_Offset", "test_multiturn_motor", 3048)

        position = bus.read("Present_Position", "test_multiturn_motor", normalize=False)
        print()
        print(f"Current position 3: {position}")
        print()

        return position

    except Exception as e:
        print(f"✗ Error reading position: {e}")
        return None

    finally:
        # Clean up
        try:
            bus.disconnect()
            print("✓ Disconnected successfully")
        except:
            pass

if __name__ == "__main__":
    print("Reading motor position...")
    position = read_motor_position()
