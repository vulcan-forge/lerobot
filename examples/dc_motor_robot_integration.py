#!/usr/bin/env python

"""
Example showing how to integrate DC motors with potentiometer encoding
into an existing robot configuration as a replacement for feetech motors.

This example demonstrates:
1. How to modify an existing robot configuration to use DC motors
2. How to replace specific joints with DC motors
3. How to maintain compatibility with the existing robot interface
"""

import time
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dc_motor import DCMotorsBus
from lerobot.common.motors.feetech import FeetechMotorsBus


class HybridRobot:
    """
    Example robot that uses a mix of Feetech and DC motors.
    This demonstrates how to integrate DC motors into existing robot configurations.
    """
    
    def __init__(self, port_feetech="/dev/ttyUSB0", port_dc="/dev/ttyUSB1"):
        self.port_feetech = port_feetech
        self.port_dc = port_dc
        
        # Define motors - mixing Feetech and DC motors
        self.feetech_motors = {
            "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
            "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        }
        
        # DC motor for shoulder lift (replacing feetech motor)
        self.dc_motors = {
            "shoulder_lift": Motor(2, "shoulder_lift_dc", MotorNormMode.DEGREES),
        }
        
        # Initialize motor buses
        self.feetech_bus = None
        self.dc_bus = None
        
    def connect(self):
        """Connect to all motors."""
        print("Connecting to Feetech motors...")
        self.feetech_bus = FeetechMotorsBus(
            port=self.port_feetech,
            motors=self.feetech_motors,
            calibration=None,
        )
        self.feetech_bus.connect()
        
        print("Connecting to DC motors...")
        self.dc_bus = DCMotorsBus(
            port=self.port_dc,
            motors=self.dc_motors,
            calibration=None,
        )
        self.dc_bus.connect()
        
        print("All motors connected!")
        
    def configure(self):
        """Configure all motors."""
        if self.feetech_bus:
            print("Configuring Feetech motors...")
            self.feetech_bus.configure_motors()
        
        if self.dc_bus:
            print("Configuring DC motors...")
            self.dc_bus.configure_motors()
        
        print("All motors configured!")
        
    def enable_torque(self):
        """Enable torque on all motors."""
        if self.feetech_bus:
            print("Enabling torque on Feetech motors...")
            self.feetech_bus.enable_torque()
        
        if self.dc_bus:
            print("Enabling torque on DC motors...")
            self.dc_bus.enable_torque()
        
        print("All motors enabled!")
        
    def disable_torque(self):
        """Disable torque on all motors."""
        if self.feetech_bus:
            print("Disabling torque on Feetech motors...")
            self.feetech_bus.disable_torque()
        
        if self.dc_bus:
            print("Disabling torque on DC motors...")
            self.dc_bus.disable_torque()
        
        print("All motors disabled!")
        
    def read_all_positions(self):
        """Read positions from all motors."""
        positions = {}
        
        # Read from Feetech motors
        if self.feetech_bus:
            feetech_positions = self.feetech_bus.sync_read("Present_Position", normalize=True)
            positions.update(feetech_positions)
        
        # Read from DC motors
        if self.dc_bus:
            dc_positions = self.dc_bus.sync_read("Present_Position", normalize=True)
            positions.update(dc_positions)
        
        return positions
        
    def move_to_positions(self, target_positions):
        """Move all motors to target positions."""
        # Separate positions by motor type
        feetech_targets = {}
        dc_targets = {}
        
        for motor_name, position in target_positions.items():
            if motor_name in self.feetech_motors:
                feetech_targets[motor_name] = position
            elif motor_name in self.dc_motors:
                dc_targets[motor_name] = position
            else:
                print(f"Warning: Unknown motor {motor_name}")
                
        # Move Feetech motors
        if feetech_targets and self.feetech_bus:
            self.feetech_bus.sync_write("Goal_Position", feetech_targets, normalize=True)
            
        # Move DC motors
        if dc_targets and self.dc_bus:
            self.dc_bus.sync_write("Goal_Position", dc_targets, normalize=True)
            
    def disconnect(self):
        """Disconnect from all motors."""
        if self.feetech_bus:
            self.feetech_bus.disconnect()
        if self.dc_bus:
            self.dc_bus.disconnect()
        print("All motors disconnected!")


def demonstrate_hybrid_robot():
    """
    Demonstrate the hybrid robot with mixed Feetech and DC motors.
    """
    print("Hybrid Robot Demo")
    print("=================")
    print("This demo shows a robot with:")
    print("- Feetech motors: shoulder_pan, elbow_flex, wrist_flex, wrist_roll, gripper")
    print("- DC motor: shoulder_lift (with potentiometer)")
    print()
    
    # Create the hybrid robot
    robot = HybridRobot()
    
    try:
        # Connect to motors
        robot.connect()
        
        # Configure motors
        robot.configure()
        
        # Enable torque
        robot.enable_torque()
        
        # Read current positions
        print("\nReading current positions...")
        positions = robot.read_all_positions()
        for motor_name, pos in positions.items():
            print(f"  {motor_name}: {pos:.2f} degrees")
            
        # Move to a test position
        print("\nMoving to test positions...")
        test_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 45.0,  # This will use the DC motor
            "elbow_flex": 90.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 50.0,
        }
        
        robot.move_to_positions(test_positions)
        
        # Wait for movement
        time.sleep(3)
        
        # Read positions again
        print("\nReading positions after movement...")
        new_positions = robot.read_all_positions()
        for motor_name, pos in new_positions.items():
            print(f"  {motor_name}: {pos:.2f} degrees")
            
        # Move back to center
        print("\nMoving back to center...")
        center_positions = {motor: 0.0 for motor in positions.keys()}
        robot.move_to_positions(center_positions)
        
        # Wait for movement
        time.sleep(3)
        
        # Read final positions
        print("\nReading final positions...")
        final_positions = robot.read_all_positions()
        for motor_name, pos in final_positions.items():
            print(f"  {motor_name}: {pos:.2f} degrees")
            
    except Exception as e:
        print(f"Error: {e}")
        print("\nNote: This demo requires actual hardware or virtual serial ports.")
        print("The DC motor implementation is designed to work with real hardware.")
        
    finally:
        # Disable torque and disconnect
        robot.disable_torque()
        robot.disconnect()


def show_migration_example():
    """
    Show how to migrate from all-Feetech to hybrid configuration.
    """
    print("\nMigration Example")
    print("=================")
    print("Original SO100 robot configuration:")
    print("```python")
    print("motors = {")
    print('    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),')
    print('    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),  # <-- This one')
    print('    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),')
    print('    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),')
    print('    "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),')
    print('    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),')
    print("}")
    print("```")
    print()
    print("After migration to hybrid configuration:")
    print("```python")
    print("# Feetech motors")
    print("feetech_motors = {")
    print('    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),')
    print('    "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),')
    print('    "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),')
    print('    "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),')
    print('    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),')
    print("}")
    print()
    print("# DC motor (replacement)")
    print("dc_motors = {")
    print('    "shoulder_lift": Motor(2, "shoulder_lift_dc", MotorNormMode.DEGREES),')
    print("}")
    print("```")
    print()
    print("Key benefits:")
    print("- Drop-in replacement: Same API as Feetech motors")
    print("- Cost effective: DC motors are cheaper than Feetech servos")
    print("- Customizable: Can use any DC motor with potentiometer")
    print("- Maintains compatibility: Existing code continues to work")


if __name__ == "__main__":
    demonstrate_hybrid_robot()
    show_migration_example() 