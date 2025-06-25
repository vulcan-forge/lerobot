#!/usr/bin/env python3
"""
Test Script: Synchronized Motor Movement

This script synchronizes the movement of two motors based on percentage of their range.
Motor 2 will move the same percentage of its range as motor 1 moves of its range.

Example:
- Motor 1 (single turn): 0-4095, moves to 1000 (24.4% of range)
- Motor 2 (multiturn): 0-28665, moves to 7000 (24.4% of range)

Usage:
    python start/motor_sync_test.py --port COM5 --motor1-id 1 --motor1-type sts3215 --motor2-id 2 --motor2-type sts3235
"""

import argparse
import logging
import sys
import time
from typing import Dict, Optional

# Add the parent directory to the path to import lerobot modules
sys.path.insert(0, '..')

from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.common.motors.feetech.tables import MODEL_RESOLUTION, MODEL_NUMBER_TABLE

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class MotorSyncController:
    """Controls synchronized movement of two motors."""
    
    def __init__(self, port: str):
        self.port = port
        self.bus = None
        
        # Motor configurations
        self.motor1_config = None
        self.motor2_config = None
        self.motor1_range = None
        self.motor2_range = None
        
    def setup_motors(self, motor1_id: int, motor1_type: str, motor2_id: int, motor2_type: str) -> bool:
        """Setup both motors for synchronized movement."""
        logger.info("Setting up motors for synchronized movement")
        
        # Determine motor ranges
        self.motor1_range = self._get_motor_range(motor1_type)
        self.motor2_range = self._get_motor_range(motor2_type)
        
        logger.info(f"Motor 1 (ID {motor1_id}, {motor1_type}): range 0-{self.motor1_range}")
        logger.info(f"Motor 2 (ID {motor2_id}, {motor2_type}): range 0-{self.motor2_range}")
        
        # Create motor configurations
        motor1 = Motor(motor1_id, motor1_type, MotorNormMode.RANGE_M100_100)
        motor2 = Motor(motor2_id, motor2_type, MotorNormMode.RANGE_M100_100)
        
        motors = {
            "motor1": motor1,
            "motor2": motor2
        }
        
        try:
            # Create motor bus
            self.bus = FeetechMotorsBus(port=self.port, motors=motors)
            self.bus.connect()
            
            # Configure both motors
            for motor_name in ["motor1", "motor2"]:
                # Disable torque for safe configuration
                self.bus.disable_torque()
                
                # Set operating mode to position control
                self.bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
                
                # Set position limits
                motor_range = self.motor1_range if motor_name == "motor1" else self.motor2_range
                self.bus.write("Min_Position_Limit", motor_name, 0, normalize=False)
                self.bus.write("Max_Position_Limit", motor_name, motor_range, normalize=False)
                
                # Set PID coefficients for smooth operation
                self.bus.write("P_Coefficient", motor_name, 16)
                self.bus.write("I_Coefficient", motor_name, 0)
                self.bus.write("D_Coefficient", motor_name, 32)
                
                # Set acceleration
                self.bus.write("Acceleration", motor_name, 254, normalize=False)
                
                # Enable torque
                self.bus.enable_torque()
                
                logger.info(f"Configured {motor_name}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to setup motors: {e}")
            return False
    
    def _get_motor_range(self, motor_type: str) -> int:
        """Get the full range for a motor type."""
        # Check if it's a multiturn motor
        multiturn_motors = ["sts3235"]  # Add more multiturn motors as needed
        
        if motor_type in multiturn_motors:
            # Multiturn motor: 7 turns * 4095 steps per turn
            return 4095 * 7
        else:
            # Single turn motor: use resolution from table
            resolution = MODEL_RESOLUTION.get(motor_type, 4096)
            return resolution - 1
    
    def move_synchronized(self, motor1_position: int) -> bool:
        """Move both motors synchronously based on percentage of range."""
        try:
            # Ensure motor ranges are set
            if self.motor1_range is None or self.motor2_range is None:
                logger.error("Motor ranges not initialized")
                return False
            
            if self.bus is None:
                logger.error("Motor bus not initialized")
                return False
            
            # Calculate percentage of motor1's range
            percentage = motor1_position / self.motor1_range
            
            # Calculate corresponding position for motor2
            motor2_position = int(percentage * self.motor2_range)
            
            logger.info(f"Motor 1 moving to {motor1_position} ({percentage*100:.1f}% of range)")
            logger.info(f"Motor 2 moving to {motor2_position} ({percentage*100:.1f}% of range)")
            
            # Move both motors
            self.bus.write("Goal_Position", "motor1", motor1_position, normalize=False)
            self.bus.write("Goal_Position", "motor2", motor2_position, normalize=False)
            
            # Wait for movement
            time.sleep(2)
            
            # Read final positions
            final_pos1 = self.bus.read("Goal_Position", "motor1", normalize=False)
            final_pos2 = self.bus.read("Goal_Position", "motor2", normalize=False)
            
            logger.info(f"Final positions - Motor 1: {final_pos1}, Motor 2: {final_pos2}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to move motors synchronously: {e}")
            return False
    
    def interactive_control(self) -> bool:
        """Interactive control for synchronized movement."""
        logger.info("Starting interactive synchronized control")
        logger.info("Enter position for motor 1, or 'quit' to exit")
        
        if self.motor1_range is None or self.motor2_range is None:
            logger.error("Motor ranges not initialized")
            return False
            
        logger.info(f"Motor 1 range: 0-{self.motor1_range}")
        logger.info(f"Motor 2 will move proportionally in range: 0-{self.motor2_range}")
        
        while True:
            try:
                if self.bus is None:
                    logger.error("Motor bus not initialized")
                    break
                
                # Get current positions
                pos1 = self.bus.read("Goal_Position", "motor1", normalize=False)
                pos2 = self.bus.read("Goal_Position", "motor2", normalize=False)
                
                user_input = input(f"\nCurrent - Motor 1: {pos1}, Motor 2: {pos2}\nEnter motor 1 position or command: ").strip()
                
                if user_input.lower() == 'quit':
                    logger.info("Exiting synchronized control")
                    break
                elif user_input.lower() == 'home':
                    logger.info("Moving both motors to home (0)")
                    self.move_synchronized(0)
                    continue
                elif user_input.lower() == 'max':
                    logger.info(f"Moving both motors to maximum")
                    self.move_synchronized(self.motor1_range)
                    continue
                elif user_input.lower() == 'test':
                    logger.info("Running synchronized movement test")
                    test_positions = [0, 1000, 2000, 3000, 4000, self.motor1_range]
                    for pos in test_positions:
                        logger.info(f"Testing position {pos}")
                        self.move_synchronized(pos)
                        time.sleep(1)
                    continue
                
                # Try to parse as position value
                try:
                    position = int(user_input)
                    if position < 0 or position > self.motor1_range:
                        logger.warning(f"Position must be between 0 and {self.motor1_range}")
                        continue
                    
                    self.move_synchronized(position)
                    
                except ValueError:
                    logger.warning("Invalid input. Enter a number, 'quit', 'home', 'max', or 'test'")
                    
            except KeyboardInterrupt:
                logger.info("\nInterrupted by user")
                break
            except Exception as e:
                logger.error(f"Error during synchronized control: {e}")
                break
        
        return True
    
    def cleanup(self):
        """Clean up resources."""
        if self.bus and self.bus.port_handler.is_open:
            self.bus.port_handler.closePort()


def main():
    """Main function to handle command line arguments and execute synchronized movement."""
    parser = argparse.ArgumentParser(description="Synchronized Motor Movement Test")
    parser.add_argument("--port", required=True, help="Serial port (e.g., COM5)")
    parser.add_argument("--motor1-id", type=int, required=True, help="Motor 1 ID")
    parser.add_argument("--motor1-type", required=True, help="Motor 1 type (e.g., sts3215)")
    parser.add_argument("--motor2-id", type=int, required=True, help="Motor 2 ID")
    parser.add_argument("--motor2-type", required=True, help="Motor 2 type (e.g., sts3235)")
    
    args = parser.parse_args()
    
    # Create motor sync controller
    controller = MotorSyncController(args.port)
    
    try:
        # Setup motors
        success = controller.setup_motors(
            args.motor1_id, args.motor1_type,
            args.motor2_id, args.motor2_type
        )
        
        if not success:
            print("Failed to setup motors")
            sys.exit(1)
        
        # Start interactive control
        controller.interactive_control()
        
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.cleanup()


if __name__ == "__main__":
    main() 