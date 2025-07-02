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

def test_multiturn_continuous_rotation():
    """Test continuous rotation behavior of the STS3235 multi-turn motor."""

    # Motor configuration for STS3235
    motor_id = 2
    motor_model = "sts3235"
    gear_ratio = 1.0
    port = "COM13"

    motors = {
        "test_multiturn_motor": Motor(motor_id, motor_model, MotorNormMode.RANGE_M100_100, gear_ratio=gear_ratio),
    }

    print(f"Testing STS3235 continuous rotation")
    print(f"Motor ID: {motor_id}")
    print(f"Motor Model: {motor_model}")
    print(f"Gear Ratio: {gear_ratio}")
    print(f"Port: {port}")
    print("-" * 50)

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
        print("Connecting to multi-turn motor...")
        bus.connect(handshake=False)
        print("✓ Connected successfully")

        # Test ping
        print(f"Testing ping on motor ID {motor_id}...")
        try:
            model_number = bus.ping(motor_id)
            print(f"✓ Motor responded with model number: {model_number}")
        except Exception as e:
            print(f"✗ Ping failed: {e}")
            return False

        # Read current configuration
        print("\n=== READING CURRENT CONFIGURATION ===")
        current_operating_mode = bus.read("Operating_Mode", "test_multiturn_motor", normalize=False)
        current_min_limit = bus.read("Min_Position_Limit", "test_multiturn_motor", normalize=False)
        current_max_limit = bus.read("Max_Position_Limit", "test_multiturn_motor", normalize=False)

        print(f"Current Operating Mode: {current_operating_mode}")
        print(f"Current Min Position Limit: {current_min_limit}")
        print(f"Current Max Position Limit: {current_max_limit}")

        # Configure motor for multi-turn operation
        print("\n=== CONFIGURING MOTOR FOR MULTI-TURN ===")

        # Set operating mode to Position Mode (0)
        print("Setting Operating Mode to Position Mode (0)...")
        bus.write("Operating_Mode", "test_multiturn_motor", OperatingMode.POSITION.value, normalize=False)
        time.sleep(1)

        # Set position limits to allow full multi-turn range
        print("Setting position limits for multi-turn operation...")
        bus.write("Min_Position_Limit", "test_multiturn_motor", 0, normalize=False)
        bus.write("Max_Position_Limit", "test_multiturn_motor", 28672, normalize=False)  # 7 rotations * 4096
        time.sleep(1)

        # Verify configuration
        new_operating_mode = bus.read("Operating_Mode", "test_multiturn_motor", normalize=False)
        new_min_limit = bus.read("Min_Position_Limit", "test_multiturn_motor", normalize=False)
        new_max_limit = bus.read("Max_Position_Limit", "test_multiturn_motor", normalize=False)

        print(f"New Operating Mode: {new_operating_mode}")
        print(f"New Min Position Limit: {new_min_limit}")
        print(f"New Max Position Limit: {new_max_limit}")

        # Read initial position
        initial_pos = bus.read("Present_Position", "test_multiturn_motor", normalize=False)
        print(f"\nInitial position: {initial_pos} ({initial_pos / 4096 * gear_ratio:.2f} physical turns)")

        # Test continuous rotation by moving to full rotation positions
        print("\n=== TESTING FULL ROTATION POSITIONS ===")

        # Test positions at 4096, 8192, 12288, etc. (full rotations)
        full_rotation_positions = [0, 28672] #[0, 4096, 8192, 12288, 16384, 20480, 24576, 28672]

        for i, target_pos in enumerate(full_rotation_positions):
            physical_turns = target_pos / 4096 * gear_ratio

            print(f"\n--- Rotation {i+1}: Position {target_pos} ({physical_turns:.0f} full turns) ---")

            # Move to target position
            bus.write("Goal_Position", "test_multiturn_motor", target_pos, normalize=False)

            # Wait for movement (longer for full rotations)
            time.sleep(20)

            # Read actual position
            actual_pos = bus.read("Present_Position", "test_multiturn_motor", normalize=False)
            actual_turns = actual_pos / 4096 * gear_ratio
            moving = bus.read("Moving", "test_multiturn_motor", normalize=False)

            print(f"  Target: {target_pos} ({physical_turns:.0f} turns)")
            print(f"  Actual: {actual_pos} ({actual_turns:.2f} turns)")
            print(f"  Moving: {moving}")

            if abs(actual_pos - target_pos) <= 50:  # Larger tolerance for full rotations
                print("  ✓ Movement successful")
            else:
                print("  ⚠ Movement may not have completed")

        print("\n=== FULL ROTATION TEST COMPLETED ===")

        return True

    except Exception as e:
        print(f"✗ Error during continuous rotation test: {e}")
        return False

    finally:
        # Clean up
        try:
            print("\nDisconnecting...")
            bus.disconnect()
            print("✓ Disconnected successfully")
        except:
            pass

if __name__ == "__main__":
    print("STS3235 Multi-Turn Motor Test Suite")
    print("=" * 50)

    success = test_multiturn_continuous_rotation()
    if success:
        print("\n🎉 Test completed successfully!")
        sys.exit(0)
    else:
        print("\n❌ Test failed!")
        sys.exit(1)
