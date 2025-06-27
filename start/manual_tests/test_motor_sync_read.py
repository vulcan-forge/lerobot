#!/usr/bin/env python3
"""Test motor sync_read for given motor IDs."""

import sys
from lerobot.common.motors.feetech import FeetechMotorsBus
from lerobot.common.motors import Motor, MotorNormMode


def test_motor_sync_read():
    """Test sync_read for specified motor IDs."""
    
    # Get motor IDs from user
    print("Enter motor IDs to test (comma-separated, e.g., 1,2,3):")
    try:
        motor_ids = [int(x.strip()) for x in input().split(',')]
    except ValueError:
        print("Invalid input. Please enter numbers separated by commas.")
        return
    
    # Get port from user
    print("Enter USB port (e.g., /dev/ttyACM0, COM5):")
    port = input().strip()
    
    # Create motor dictionary
    motors = {}
    for motor_id in motor_ids:
        motors[f"motor_{motor_id}"] = Motor(motor_id, "sts3215", MotorNormMode.RANGE_M100_100)
    
    # Create bus and test
    try:
        bus = FeetechMotorsBus(port=port, motors=motors)
        print(f"\nConnecting to {port}...")
        bus.connect()
        print("✓ Connected successfully")
        
        # Test sync_read
        print(f"\nTesting sync_read for motors: {motor_ids}")
        positions = bus.sync_read("Present_Position")
        
        print("✓ Sync_read successful!")
        print("Motor positions:")
        for motor_name, position in positions.items():
            print(f"  {motor_name}: {position}")
            
        bus.disconnect()
        print("\n✓ Disconnected successfully")
        
    except Exception as e:
        print(f"✗ Error: {e}")
        print("\nTroubleshooting tips:")
        print("- Check if the port is correct")
        print("- Make sure motors are powered on")
        print("- Verify USB connection")
        print("- Check if motors are responding")


if __name__ == "__main__":
    test_motor_sync_read() 