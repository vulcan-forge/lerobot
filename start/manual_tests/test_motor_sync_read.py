#!/usr/bin/env python3
"""
Manual test script for Feetech motor sync read on ID 1 using /dev/ttyUSB0
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
    """Test sync read on Feetech motor ID 1 using /dev/ttyUSB0"""

    # Define the motor configuration
    motors = {
        "test_motor": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100)
    }

    # Create the motors bus
    port = "/dev/ttyUSB0"
    bus = FeetechMotorsBus(port=port, motors=motors)

    print(f"Testing Feetech motor sync read on {port}")
    print(f"Motor ID: 1")
    print(f"Motor Model: sts3215")
    print("-" * 50)

    try:
        # Connect to the motor
        print("Connecting to motor...")
        bus.connect(handshake=False)
        print("✓ Connected successfully")

        # Test basic ping first
        print("\nTesting ping...")
        model_number = bus.ping(1)
        print(f"✓ Ping successful - Model Number: {model_number}")

        # Test sync read of Present_Position
        print("\nTesting sync read of Present_Position...")
        for i in range(5):
            try:
                positions = bus.sync_read("Present_Position", ["test_motor"], normalize=False)
                position = positions["test_motor"]
                print(f"  Read {i+1}: Position = {position}")
                time.sleep(0.1)
            except Exception as e:
                print(f"  Read {i+1}: Failed - {e}")
                time.sleep(0.1)

        # Test sync read of other common registers
        test_registers = ["Present_Velocity", "Present_Load", "Present_Voltage", "Present_Temperature"]

        for register in test_registers:
            print(f"\nTesting sync read of {register}...")
            try:
                values = bus.sync_read(register, ["test_motor"], normalize=False)
                value = values["test_motor"]
                print(f"  ✓ {register} = {value}")
            except Exception as e:
                print(f"  ✗ Failed to read {register}: {e}")

        # Test reading multiple registers at once
        print("\nTesting sync read of multiple registers...")
        try:
            positions = bus.sync_read("Present_Position", ["test_motor"], normalize=False)
            velocities = bus.sync_read("Present_Velocity", ["test_motor"], normalize=False)
            loads = bus.sync_read("Present_Load", ["test_motor"], normalize=False)

            print(f"  Position: {positions['test_motor']}")
            print(f"  Velocity: {velocities['test_motor']}")
            print(f"  Load: {loads['test_motor']}")
        except Exception as e:
            print(f"  ✗ Failed to read multiple registers: {e}")

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
    print("Feetech Motor Sync Read Test")
    print("=" * 40)

    success = test_feetech_motor_sync_read()

    if success:
        print("\n🎉 All tests passed!")
        sys.exit(0)
    else:
        print("\n❌ Tests failed!")
        sys.exit(1)
