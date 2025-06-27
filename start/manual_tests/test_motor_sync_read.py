#!/usr/bin/env python3
"""
Manual test script for Feetech motor sync read on all 12 motors using /dev/ttyUSB0
This mimics the exact setup used in Sourccey V2 Beta teleoperation
"""

import time
import sys
from pathlib import Path

# Add the project root to the path so we can import lerobot
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.feetech import FeetechMotorsBus


def test_feetech_motor_sync_read():
    """Test sync read on all 12 Feetech motors using /dev/ttyUSB0"""

    # Define the motor configuration - EXACTLY as used in teleoperation
    motors = {
        "left_arm_shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
        "left_arm_shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
        "left_arm_elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
        "left_arm_wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
        "left_arm_wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
        "left_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        "right_arm_shoulder_pan": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
        "right_arm_shoulder_lift": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
        "right_arm_elbow_flex": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
        "right_arm_wrist_flex": Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
        "right_arm_wrist_roll": Motor(11, "sts3215", MotorNormMode.RANGE_M100_100),
        "right_arm_gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
    }

    # Create the motors bus
    port = "/dev/ttyUSB0"
    bus = FeetechMotorsBus(port=port, motors=motors)

    print(f"Testing Feetech motor sync read on {port}")
    print(f"Testing all 12 motors (IDs 1-12)")
    print(f"Motor Model: sts3215")
    print("-" * 50)

    try:
        # Connect to the motors
        print("Connecting to motors...")
        bus.connect(handshake=False)
        print("✓ Connected successfully")

        # Test ping on each motor individually
        print("\nTesting ping on each motor...")
        for motor_name, motor in motors.items():
            try:
                model_number = bus.ping(motor.id)
                print(f"  ✓ {motor_name} (ID {motor.id}): Model {model_number}")
            except Exception as e:
                print(f"  ✗ {motor_name} (ID {motor.id}): Failed - {e}")

        # Test sync read of Present_Position on all motors
        print("\nTesting sync read of Present_Position on all motors...")
        for i in range(5):
            try:
                positions = bus.sync_read("Present_Position", normalize=False)
                print(f"  Read {i+1}: Success - {len(positions)} motors responded")
                for motor_name, position in positions.items():
                    print(f"    {motor_name}: {position}")
                time.sleep(0.1)
            except Exception as e:
                print(f"  Read {i+1}: Failed - {e}")
                time.sleep(0.1)

        # Test sync read with different motor subsets
        print("\nTesting sync read with different motor subsets...")

        # Test left arm only (IDs 1-6)
        left_arm_motors = ["left_arm_shoulder_pan", "left_arm_shoulder_lift", "left_arm_elbow_flex",
                          "left_arm_wrist_flex", "left_arm_wrist_roll", "left_arm_gripper"]
        try:
            positions = bus.sync_read("Present_Position", left_arm_motors, normalize=False)
            print(f"  ✓ Left arm (6 motors): Success - {len(positions)} motors responded")
        except Exception as e:
            print(f"  ✗ Left arm (6 motors): Failed - {e}")

        # Test right arm only (IDs 7-12)
        right_arm_motors = ["right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
                           "right_arm_wrist_flex", "right_arm_wrist_roll", "right_arm_gripper"]
        try:
            positions = bus.sync_read("Present_Position", right_arm_motors, normalize=False)
            print(f"  ✓ Right arm (6 motors): Success - {len(positions)} motors responded")
        except Exception as e:
            print(f"  ✗ Right arm (6 motors): Failed - {e}")

        # Test individual motors
        print("\nTesting sync read on individual motors...")
        for motor_name in motors.keys():
            try:
                positions = bus.sync_read("Present_Position", [motor_name], normalize=False)
                position = positions[motor_name]
                print(f"  ✓ {motor_name}: {position}")
            except Exception as e:
                print(f"  ✗ {motor_name}: Failed - {e}")

    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return False

    finally:
        # Clean up
        try:
            bus.close()
            print("\n✓ Connection closed")
        except:
            pass

    print("\n✓ Test completed successfully")
    return True


if __name__ == "__main__":
    print("Feetech Motor Sync Read Test - All 12 Motors")
    print("=" * 50)

    success = test_feetech_motor_sync_read()

    if success:
        print("\n🎉 All tests passed!")
        sys.exit(0)
    else:
        print("\n❌ Tests failed!")
        sys.exit(1)
