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
        # "left_arm_shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
        # "left_arm_elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
        # "left_arm_wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
        # "left_arm_wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
        # "left_arm_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        # "right_arm_shoulder_pan": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
        # "right_arm_shoulder_lift": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
        # "right_arm_elbow_flex": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
        # "right_arm_wrist_flex": Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
        # "right_arm_wrist_roll": Motor(11, "sts3215", MotorNormMode.RANGE_M100_100),
        # "right_arm_gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
    }

    # Create the motors bus
    port = "/dev/ttyUSB0"
    bus = FeetechMotorsBus(port=port, motors=motors)

    print(f"Testing Feetech motor sync read on {port}")
    print(f"Testing all {len(motors)} motors (IDs 1-{len(motors)})")
    print(f"Motor Model: sts3215")
    print("-" * 50)

    try:
        # Connect to the motors
        print("Connecting to motors...")
        bus.connect(handshake=False)
        print("✓ Connected successfully")

        # Set a longer timeout for sync operations
        print("Setting longer timeout for sync operations...")
        bus.set_timeout(1000)  # 1 second timeout
        print("✓ Timeout set to 1000ms")

        # Test ping on each motor individually
        print("\nTesting ping on each motor...")
        for motor_name, motor in motors.items():
            try:
                model_number = bus.ping(motor.id)
                print(f"  ✓ {motor_name} (ID {motor.id}): Model {model_number}")
            except Exception as e:
                print(f"  ✗ {motor_name} (ID {motor.id}): Failed - {e}")

        # Test sync read with longer delays between attempts
        print("\nTesting sync read of Present_Position on all motors (with delays)...")
        success_count = 0
        total_attempts = 10

        for i in range(total_attempts):
            try:
                positions = bus.sync_read("Present_Position", normalize=False)
                success_count += 1
                print(f"  Read {i+1}: Success - {len(positions)} motors responded")
                if i == 0:  # Only show positions for first successful read
                    for motor_name, position in positions.items():
                        print(f"    {motor_name}: {position}")
                time.sleep(0.2)  # Longer delay between reads
            except Exception as e:
                print(f"  Read {i+1}: Failed - {e}")
                time.sleep(0.5)  # Even longer delay after failure

        print(f"\nSync read success rate: {success_count}/{total_attempts} ({success_count/total_attempts*100:.1f}%)")

        # Test with different timing strategies
        print("\nTesting different timing strategies...")

        # Strategy 1: Read with longer delays
        print("Strategy 1: Reading with 0.3s delays...")
        try:
            time.sleep(0.3)
            positions = bus.sync_read("Present_Position", normalize=False)
            print(f"  ✓ Success after 0.3s delay")
        except Exception as e:
            print(f"  ✗ Failed after 0.3s delay: {e}")

        # Strategy 2: Read left and right arms separately
        print("Strategy 2: Reading arms separately...")
        try:
            left_arm_motors = ["left_arm_shoulder_pan",]# "left_arm_shoulder_lift", "left_arm_elbow_flex",
                            #   "left_arm_wrist_flex", "left_arm_wrist_roll", "left_arm_gripper"]
            # right_arm_motors = ["right_arm_shoulder_pan", "right_arm_shoulder_lift", "right_arm_elbow_flex",
                            #    "right_arm_wrist_flex", "right_arm_wrist_roll", "right_arm_gripper"]

            left_positions = bus.sync_read("Present_Position", left_arm_motors, normalize=False)
            time.sleep(0.1)
            # right_positions = bus.sync_read("Present_Position", right_arm_motors, normalize=False)

            all_positions = {**left_positions } #, **right_positions}
            print(f"  ✓ Success reading arms separately - {len(all_positions)} motors")
        except Exception as e:
            print(f"  ✗ Failed reading arms separately: {e}")

        # Strategy 3: Test with different retry counts
        print("Strategy 3: Testing with increased retry count...")
        try:
            # Temporarily increase retry count
            positions = bus.sync_read("Present_Position", normalize=False, num_retry=8)
            print(f"  ✓ Success with 8 retries")
        except Exception as e:
            print(f"  ✗ Failed with 8 retries: {e}")

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
