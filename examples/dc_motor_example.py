#!/usr/bin/env python

"""
Example script demonstrating how to use DC motors with potentiometer encoding
as a replacement for feetech motors in the LeRobot framework.

This example shows how to:
1. Set up a DC motor with potentiometer feedback
2. Use it as a drop-in replacement for feetech motors
3. Read position from the potentiometer
4. Control the motor position
"""

import time
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dc_motor import DCMotorsBus

def main():
    """
    Main function demonstrating DC motor usage.
    """
    print("DC Motor with Potentiometer Example")
    print("===================================")
    
    # Define the DC motor configuration
    # This mimics the shoulder_lift motor from SO100 robot
    motors = {
        "shoulder_lift": Motor(2, "shoulder_lift_dc", MotorNormMode.DEGREES),
    }
    
    # Create the DC motor bus
    # Note: Use a real serial port for actual hardware
    # For testing, you can use a virtual serial port or modify the DCMotorController
    # to work without actual hardware
    port = "/dev/ttyUSB0"  # Change this to your actual port
    
    try:
        # Initialize the motor bus
        bus = DCMotorsBus(
            port=port,
            motors=motors,
            calibration=None,  # Will be created during calibration
        )
        
        print(f"Initialized DC motor bus on port {port}")
        
        # Connect to the motors
        print("Connecting to motors...")
        bus.connect()
        print("Connected successfully!")
        
        # Configure motors for position control
        print("Configuring motors...")
        bus.configure_motors()
        
        # Enable torque
        print("Enabling torque...")
        bus.enable_torque()
        
        # Read current position
        print("\nReading current position...")
        current_pos = bus.read("Present_Position", "shoulder_lift", normalize=True)
        print(f"Current position: {current_pos:.2f} degrees")
        
        # Move to a new position
        print("\nMoving to 45 degrees...")
        bus.write("Goal_Position", "shoulder_lift", 45.0, normalize=True)
        
        # Wait for movement to complete
        time.sleep(2)
        
        # Read position again
        new_pos = bus.read("Present_Position", "shoulder_lift", normalize=True)
        print(f"New position: {new_pos:.2f} degrees")
        
        # Move back to center
        print("\nMoving back to center (0 degrees)...")
        bus.write("Goal_Position", "shoulder_lift", 0.0, normalize=True)
        
        # Wait for movement to complete
        time.sleep(2)
        
        # Read final position
        final_pos = bus.read("Present_Position", "shoulder_lift", normalize=True)
        print(f"Final position: {final_pos:.2f} degrees")
        
        # Demonstrate sync read
        print("\nDemonstrating sync read...")
        positions = bus.sync_read("Present_Position", normalize=True)
        for motor_name, pos in positions.items():
            print(f"{motor_name}: {pos:.2f} degrees")
        
        # Disable torque before disconnecting
        print("\nDisabling torque...")
        bus.disable_torque()
        
    except Exception as e:
        print(f"Error: {e}")
        print("\nNote: This example requires actual hardware or a virtual serial port.")
        print("To test without hardware, you can modify the DCMotorController class")
        print("to simulate the serial communication.")
        
    finally:
        # Disconnect
        if 'bus' in locals():
            print("Disconnecting...")
            bus.disconnect()
            print("Disconnected.")

def calibration_example():
    """
    Example showing how to calibrate the DC motor.
    """
    print("\nCalibration Example")
    print("===================")
    
    motors = {
        "shoulder_lift": Motor(2, "shoulder_lift_dc", MotorNormMode.DEGREES),
    }
    
    port = "/dev/ttyUSB0"  # Change this to your actual port
    
    try:
        bus = DCMotorsBus(port=port, motors=motors)
        bus.connect()
        
        print("Starting calibration...")
        print("1. Move the motor to the middle of its range of motion")
        print("2. Press Enter when ready...")
        input()
        
        # Set homing offsets
        homing_offsets = bus.set_half_turn_homings()
        print(f"Homing offsets: {homing_offsets}")
        
        # Record range of motion
        print("\nNow move the motor through its entire range of motion")
        print("Press Enter when done...")
        input()
        
        range_mins, range_maxes = bus.record_ranges_of_motion()
        print(f"Range minimums: {range_mins}")
        print(f"Range maximums: {range_maxes}")
        
        # Save calibration
        calibration = bus.read_calibration()
        print(f"Calibration saved: {calibration}")
        
    except Exception as e:
        print(f"Calibration error: {e}")
    finally:
        if 'bus' in locals():
            bus.disconnect()

if __name__ == "__main__":
    main()
    calibration_example() 