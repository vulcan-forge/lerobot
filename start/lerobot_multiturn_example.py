#!/usr/bin/env python3
"""
Example: Using STS3235 Multiturn Motor in LeRobot

This example shows how to properly configure and use a multiturn motor
with gear ratios in LeRobot. The multiturn capability allows for flexible
gear ratios while maintaining the range of motion needed.
"""

import sys
import time

# Add the parent directory to the path to import lerobot modules
sys.path.insert(0, '..')

from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.feetech import FeetechMotorsBus, OperatingMode


def main():
    """Example of using STS3235 multiturn motor in LeRobot."""
    
    # Configure the motor with gear ratio (e.g., 3:1 reduction for higher torque)
    gear_ratio = 3.0  # 3:1 gear reduction
    
    # Create motor configuration
    motor = Motor(
        id=2,  # Your motor ID
        model="sts3235",  # Multiturn motor
        norm_mode=MotorNormMode.RANGE_M100_100,
        gear_ratio=gear_ratio  # This handles the gear ratio in LeRobot
    )
    
    motors = {"shoulder_lift": motor}
    
    # Create motor bus
    bus = FeetechMotorsBus(port="COM5", motors=motors)
    
    try:
        # Connect to the motor
        bus.connect()
        
        # Configure the motor
        bus.disable_torque()
        bus.write("Operating_Mode", "shoulder_lift", OperatingMode.POSITION.value)
        
        # Set position limits for full multiturn range
        max_position = 4095 * 7  # 7 turns
        bus.write("Min_Position_Limit", "shoulder_lift", 0, normalize=False)
        bus.write("Max_Position_Limit", "shoulder_lift", max_position, normalize=False)
        
        # Set PID coefficients for smooth operation
        bus.write("P_Coefficient", "shoulder_lift", 16)
        bus.write("I_Coefficient", "shoulder_lift", 0)
        bus.write("D_Coefficient", "shoulder_lift", 32)
        
        bus.enable_torque()
        
        print("Motor configured successfully!")
        print(f"Gear ratio: {gear_ratio}:1")
        print(f"Full range: 0-{max_position} steps")
        print(f"With gear ratio, effective range: 0-{max_position/gear_ratio:.1f} steps")
        print("\nKey benefits of multiturn motor:")
        print("- Higher torque with gear reduction")
        print("- Maintains full range of motion")
        print("- Flexible gear ratios for different applications")
        print("- Position control works reliably in 0-28665 range")
        
        # Example: Move to different positions
        positions = [0, 5000, 15000, 25000]
        
        for pos in positions:
            print(f"\nMoving to position {pos}")
            
            # LeRobot handles the gear ratio automatically
            bus.write("Goal_Position", "shoulder_lift", pos, normalize=False)
            time.sleep(3)  # Wait for movement
            
            # Read current position
            goal_pos = bus.read("Goal_Position", "shoulder_lift", normalize=False)
            present_pos = bus.read("Present_Position", "shoulder_lift", normalize=False)
            
            try:
                multiturn_pos = bus.read("Goal_Position_2", "shoulder_lift", normalize=False)
                print(f"  Goal: {goal_pos}, Present: {present_pos}, Multiturn: {multiturn_pos}")
            except:
                print(f"  Goal: {goal_pos}, Present: {present_pos}")
        
        print("\nExample completed successfully!")
        print("Note: Present_Position is limited to 0-4095, but control works for full range")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if bus and bus.port_handler.is_open:
            bus.port_handler.closePort()


if __name__ == "__main__":
    main() 