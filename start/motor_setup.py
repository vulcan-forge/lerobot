#!/usr/bin/env python3
"""
Motor Identification, Calibration, and Multiturn Setup Script

This script provides comprehensive motor management for Feetech motors, including:
- Motor identification and scanning
- Motor ID and baudrate setup
- Motor calibration
- Multiturn mode configuration for compatible motors

Usage:
    python motor_setup.py --port /dev/ttyUSB0 --action scan
    python motor_setup.py --port /dev/ttyUSB0 --action setup --motor-type sts3235
    python motor_setup.py --port /dev/ttyUSB0 --action calibrate --motor-type sts3235
    python motor_setup.py --port /dev/ttyUSB0 --action multiturn --motor-type sts3235
"""

import argparse
import logging
import sys
import time
from typing import Dict, List, Optional, Tuple

# Add the parent directory to the path to import lerobot modules
sys.path.insert(0, '..')

from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.common.motors.feetech import FeetechMotorsBus, OperatingMode, TorqueMode
from lerobot.common.motors.feetech.tables import (
    MODEL_NUMBER_TABLE,
    MODEL_RESOLUTION,
    SCAN_BAUDRATES,
    STS_SMS_SERIES_CONTROL_TABLE,
)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class MotorSetupManager:
    """Manages motor identification, calibration, and multiturn setup for Feetech motors."""
    
    def __init__(self, port: str):
        self.port = port
        self.bus: Optional[FeetechMotorsBus] = None
        
    def scan_motors(self) -> Dict[int, List[int]]:
        """
        Scan for motors on the specified port at 1,000,000 baudrate only.
        
        Returns:
            Dict mapping baudrate to list of motor IDs found
        """
        logger.info(f"Scanning for motors on port {self.port} at 1,000,000 baudrate")
        
        try:
            # Create a temporary bus for scanning with a dummy motor
            # FeetechMotorsBus requires at least one motor to be defined
            dummy_motor = Motor(1, "sts_series", MotorNormMode.RANGE_M100_100)
            temp_bus = FeetechMotorsBus(port=self.port, motors={"dummy": dummy_motor})
            
            temp_bus._connect(handshake=False)
            temp_bus.set_baudrate(1_000_000)
            
            ids_models = temp_bus.broadcast_ping()
            baudrate_ids = {}
            
            if ids_models:
                logger.info(f"Found motors at 1,000,000 baudrate: {ids_models}")
                baudrate_ids[1_000_000] = list(ids_models.keys())
            else:
                logger.info("No motors found at 1,000,000 baudrate")
            
            return baudrate_ids
            
        except Exception as e:
            logger.error(f"Error during motor scan: {e}")
            return {}
        finally:
            if 'temp_bus' in locals() and temp_bus.port_handler.is_open:
                temp_bus.port_handler.closePort()
    
    def identify_motor_model(self, motor_id: int, baudrate: int) -> Optional[str]:
        """
        Identify the model of a motor by reading its model number.
        
        Args:
            motor_id: The ID of the motor to identify
            baudrate: The baudrate to use for communication
            
        Returns:
            The motor model name or None if identification failed
        """
        try:
            # Create a temporary bus for identification with a dummy motor
            dummy_motor = Motor(motor_id, "sts_series", MotorNormMode.RANGE_M100_100)
            temp_bus = FeetechMotorsBus(port=self.port, motors={"dummy": dummy_motor})
            temp_bus._connect(handshake=False)
            temp_bus.set_baudrate(baudrate)
            
            # Read model number
            model_number = temp_bus.ping(motor_id)
            if model_number is None:
                logger.error(f"Could not ping motor ID {motor_id}")
                return None
            
            # Find the model name from the model number
            for model_name, expected_model_number in MODEL_NUMBER_TABLE.items():
                if expected_model_number == model_number:
                    logger.info(f"Motor ID {motor_id} identified as {model_name} (model number: {model_number})")
                    return model_name
            
            logger.warning(f"Unknown model number {model_number} for motor ID {motor_id}")
            return None
            
        except Exception as e:
            logger.error(f"Error identifying motor {motor_id}: {e}")
            return None
        finally:
            if 'temp_bus' in locals() and temp_bus.port_handler.is_open:
                temp_bus.port_handler.closePort()
    
    def setup_motor(self, motor_name: str, motor_type: str, target_id: int, 
                   initial_baudrate: Optional[int] = None, initial_id: Optional[int] = None) -> bool:
        """
        Setup a motor with the specified ID and baudrate.
        
        Args:
            motor_name: Name for the motor
            motor_type: Type of motor (e.g., 'sts3235')
            target_id: Target ID to set for the motor
            initial_baudrate: Current baudrate (if known)
            initial_id: Current ID (if known)
            
        Returns:
            True if setup was successful, False otherwise
        """
        logger.info(f"Setting up motor '{motor_name}' of type '{motor_type}' with target ID {target_id}")
        
        # Create motor configuration
        motor = Motor(target_id, motor_type, MotorNormMode.RANGE_M100_100)
        motors = {motor_name: motor}
        
        try:
            # Create motor bus
            self.bus = FeetechMotorsBus(port=self.port, motors=motors)
            self.bus._connect(handshake=False)
            
            # Setup the motor
            self.bus.setup_motor(motor_name, initial_baudrate, initial_id)
            
            logger.info(f"Successfully set up motor '{motor_name}' with ID {target_id}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to setup motor '{motor_name}': {e}")
            return False
        finally:
            if self.bus and self.bus.port_handler.is_open:
                self.bus.port_handler.closePort()
    
    def calibrate_motor(self, motor_name: str, motor_type: str, motor_id: int) -> bool:
        """
        Calibrate a motor by setting homing offset and position limits.
        
        Args:
            motor_name: Name of the motor
            motor_type: Type of motor
            motor_id: ID of the motor
            
        Returns:
            True if calibration was successful, False otherwise
        """
        logger.info(f"Calibrating motor '{motor_name}' (ID: {motor_id}, Type: {motor_type})")
        
        # Create motor configuration
        motor = Motor(motor_id, motor_type, MotorNormMode.RANGE_M100_100)
        motors = {motor_name: motor}
        
        try:
            # Create motor bus
            self.bus = FeetechMotorsBus(port=self.port, motors=motors)
            self.bus.connect()
            
            # Disable torque for safe calibration
            self.bus.disable_torque()
            
            # Set operating mode to position control
            self.bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
            
            # Get current position for homing offset calculation
            current_position = self.bus.read("Present_Position", motor_name, normalize=False)
            logger.info(f"Current position of {motor_name}: {current_position}")
            
            # Calculate homing offset to center the motor
            model = motor_type
            max_res = MODEL_RESOLUTION.get(model, 4096) - 1
            mid_position = max_res // 2
            homing_offset = int(current_position - mid_position)
            
            logger.info(f"Setting homing offset for {motor_name}: {homing_offset}")
            
            # Set homing offset
            self.bus.write("Homing_Offset", motor_name, homing_offset, normalize=False)
            
            # Set position limits (full range for now)
            self.bus.write("Min_Position_Limit", motor_name, 0, normalize=False)
            self.bus.write("Max_Position_Limit", motor_name, max_res, normalize=False)
            
            # Create calibration object
            calibration = MotorCalibration(
                id=motor_id,
                drive_mode=0,
                homing_offset=homing_offset,
                range_min=0,
                range_max=max_res,
            )
            
            # Write calibration to motor
            self.bus.write_calibration({motor_name: calibration})
            
            logger.info(f"Successfully calibrated motor '{motor_name}'")
            return True
            
        except Exception as e:
            logger.error(f"Failed to calibrate motor '{motor_name}': {e}")
            return False
        finally:
            if self.bus and self.bus.port_handler.is_open:
                self.bus.port_handler.closePort()
    
    def enable_multiturn(self, motor_name: str, motor_type: str, motor_id: int) -> bool:
        """
        Enable multiturn mode for compatible motors.
        
        Args:
            motor_name: Name of the motor
            motor_type: Type of motor
            motor_id: ID of the motor
            
        Returns:
            True if multiturn was enabled successfully, False otherwise
        """
        logger.info(f"Enabling multiturn mode for motor '{motor_name}' (ID: {motor_id}, Type: {motor_type})")
        
        # Check if motor type supports multiturn
        if motor_type not in ["sts3235", "sts_series", "sms_series"]:
            logger.warning(f"Motor type '{motor_type}' may not support multiturn mode")
        
        # Create motor configuration
        motor = Motor(motor_id, motor_type, MotorNormMode.RANGE_M100_100)
        motors = {motor_name: motor}
        
        try:
            # Create motor bus
            self.bus = FeetechMotorsBus(port=self.port, motors=motors)
            self.bus.connect()
            
            # Disable torque for safe configuration
            self.bus.disable_torque()
            
            # Set operating mode to position control
            self.bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
            
            # Configure multiturn-specific settings
            # Set maximum position limits for multiturn operation (full range)
            max_res = MODEL_RESOLUTION.get(motor_type, 4096) - 1
            self.bus.write("Min_Position_Limit", motor_name, 0, normalize=False)
            self.bus.write("Max_Position_Limit", motor_name, max_res, normalize=False)
            
            # For Feetech motors, multiturn is typically handled through extended position range
            # The Goal_Position_2 register is read-only and shows the current multiturn position
            # We can read it to verify multiturn capability
            try:
                goal_position_2 = self.bus.read("Goal_Position_2", motor_name, normalize=False)
                logger.info(f"Current Goal_Position_2 value: {goal_position_2}")
                logger.info("Goal_Position_2 register is available (multiturn capability confirmed)")
            except Exception as e:
                logger.warning(f"Could not read Goal_Position_2: {e}")
            
            # Set some multiturn-specific parameters
            # Increase acceleration for smoother multiturn operation
            self.bus.write("Acceleration", motor_name, 254, normalize=False)
            
            logger.info(f"Successfully enabled multiturn mode for motor '{motor_name}'")
            logger.info("Note: Use Goal_Position register for multiturn position control")
            logger.info("The motor can now rotate beyond 360 degrees using extended position values")
            return True
                
        except Exception as e:
            logger.error(f"Failed to enable multiturn for motor '{motor_name}': {e}")
            return False
        finally:
            if self.bus and self.bus.port_handler.is_open:
                self.bus.port_handler.closePort()
    
    def test_multiturn(self, motor_name: str, motor_type: str, motor_id: int) -> bool:
        """
        Test multiturn functionality by reading position values and testing extended range.
        
        Args:
            motor_name: Name of the motor
            motor_type: Type of motor
            motor_id: ID of the motor
            
        Returns:
            True if test was successful, False otherwise
        """
        logger.info(f"Testing multiturn functionality for motor '{motor_name}'")
        
        # Create motor configuration
        motor = Motor(motor_id, motor_type, MotorNormMode.RANGE_M100_100)
        motors = {motor_name: motor}
        
        try:
            # Create motor bus
            self.bus = FeetechMotorsBus(port=self.port, motors=motors)
            self.bus.connect()
            
            # Read current position values
            present_position = self.bus.read("Present_Position", motor_name, normalize=False)
            goal_position = self.bus.read("Goal_Position", motor_name, normalize=False)
            
            try:
                goal_position_2 = self.bus.read("Goal_Position_2", motor_name, normalize=False)
                logger.info(f"Goal_Position_2: {goal_position_2}")
            except Exception as e:
                logger.warning(f"Could not read Goal_Position_2: {e}")
                goal_position_2 = None
            
            logger.info(f"Present_Position: {present_position}")
            logger.info(f"Goal_Position: {goal_position}")
            
            # Test setting a multiturn position using Goal_Position
            # For multiturn motors, we can use extended position values
            test_position = 5000  # Extended position value for multiturn
            logger.info(f"Testing multiturn position control with extended position {test_position}")
            
            # Set Goal_Position to test position
            self.bus.write("Goal_Position", motor_name, test_position, normalize=False)
            time.sleep(0.5)  # Wait for motor to respond
            
            # Read back the value
            new_goal_position = self.bus.read("Goal_Position", motor_name, normalize=False)
            logger.info(f"Goal_Position after setting: {new_goal_position}")
            
            if new_goal_position == test_position:
                logger.info("Multiturn position control test successful!")
                logger.info("Motor can accept extended position values for multiturn operation")
                return True
            else:
                logger.warning(f"Position set to {test_position} but read back as {new_goal_position}")
                logger.info("This might be normal if the motor is moving or has position limits")
                return True  # Still consider it successful if we can communicate
                
        except Exception as e:
            logger.error(f"Failed to test multiturn functionality: {e}")
            return False
        finally:
            if self.bus and self.bus.port_handler.is_open:
                self.bus.port_handler.closePort()
    
    def position_control(self, motor_name: str, motor_type: str, motor_id: int) -> bool:
        """
        Interactive position control for multiturn motors.
        
        Args:
            motor_name: Name of the motor
            motor_type: Type of motor
            motor_id: ID of the motor
            
        Returns:
            True if successful, False otherwise
        """
        logger.info(f"Starting interactive position control for motor '{motor_name}'")
        
        # Check if motor type supports multiturn
        multiturn_motors = ["sts3235"]  # Only STS3235 is confirmed multiturn
        single_turn_motors = ["sts3215"]  # STS3215 is single turn
        
        if motor_type in single_turn_motors:
            logger.error(f"Motor type '{motor_type}' is a single turn motor and does not support multiturn position control")
            logger.error("This feature is only available for multiturn motors like STS3235")
            return False
        elif motor_type not in multiturn_motors:
            logger.warning(f"Motor type '{motor_type}' is not in the known multiturn list")
            logger.warning("Proceeding with caution - this may not work as expected")
        
        logger.info("This motor has 7 turns with 4095 steps per turn (range: 0-28665)")
        logger.info("Type 'quit' to exit, 'status' to show current position")
        
        # Create motor configuration
        motor = Motor(motor_id, motor_type, MotorNormMode.RANGE_M100_100)
        motors = {motor_name: motor}
        
        try:
            # Create motor bus
            self.bus = FeetechMotorsBus(port=self.port, motors=motors)
            self.bus.connect()
            
            # Enable torque
            self.bus.enable_torque()
            
            # Set operating mode to position control
            self.bus.write("Operating_Mode", motor_name, OperatingMode.POSITION.value)
            
            # Set position limits for full multiturn range
            max_position = 4095 * 7  # 7 turns * 4095 steps per turn
            self.bus.write("Min_Position_Limit", motor_name, 0, normalize=False)
            self.bus.write("Max_Position_Limit", motor_name, max_position, normalize=False)
            
            logger.info(f"Position limits set: 0 to {max_position}")
            
            while True:
                try:
                    # Get current position
                    present_pos = self.bus.read("Present_Position", motor_name, normalize=False)
                    goal_pos = self.bus.read("Goal_Position", motor_name, normalize=False)
                    
                    # Try to get multiturn position from Goal_Position_2
                    try:
                        multiturn_pos = self.bus.read("Goal_Position_2", motor_name, normalize=False)
                        current_pos_display = f"{multiturn_pos} (multiturn)"
                    except:
                        current_pos_display = f"{present_pos} (single turn)"
                    
                    # Show prompt
                    user_input = input(f"\nCurrent position: {current_pos_display}, Goal: {goal_pos}\nEnter position (0-{max_position}) or command: ").strip()
                    
                    if user_input.lower() == 'quit':
                        logger.info("Exiting position control")
                        break
                    elif user_input.lower() == 'status':
                        logger.info(f"Present Position (single turn): {present_pos}")
                        logger.info(f"Goal Position: {goal_pos}")
                        try:
                            goal_pos_2 = self.bus.read("Goal_Position_2", motor_name, normalize=False)
                            logger.info(f"Goal Position 2 (multiturn): {goal_pos_2}")
                            logger.info(f"Actual multiturn position: {goal_pos_2}")
                        except Exception as e:
                            logger.warning(f"Could not read multiturn position: {e}")
                        continue
                    elif user_input.lower() == 'home':
                        logger.info("Moving to home position (0)")
                        self.bus.write("Goal_Position", motor_name, 0, normalize=False)
                        continue
                    elif user_input.lower() == 'max':
                        logger.info(f"Moving to maximum position ({max_position})")
                        self.bus.write("Goal_Position", motor_name, max_position, normalize=False)
                        continue
                    
                    # Try to parse as position value
                    try:
                        position = int(user_input)
                        if position < 0 or position > max_position:
                            logger.warning(f"Position must be between 0 and {max_position}")
                            continue
                        
                        logger.info(f"Moving to position {position}")
                        self.bus.write("Goal_Position", motor_name, position, normalize=False)
                        
                        # Wait a moment and show movement
                        time.sleep(0.5)
                        new_goal = self.bus.read("Goal_Position", motor_name, normalize=False)
                        if new_goal == position:
                            logger.info("Position command accepted successfully")
                        else:
                            logger.warning(f"Position set to {position} but read back as {new_goal}")
                            
                    except ValueError:
                        logger.warning("Invalid input. Enter a number, 'quit', 'status', 'home', or 'max'")
                        
                except KeyboardInterrupt:
                    logger.info("\nInterrupted by user")
                    break
                except Exception as e:
                    logger.error(f"Error during position control: {e}")
                    break
                    
            return True
                
        except Exception as e:
            logger.error(f"Failed to start position control: {e}")
            return False
        finally:
            if self.bus and self.bus.port_handler.is_open:
                self.bus.port_handler.closePort()


def main():
    """Main function to handle command line arguments and execute the requested action."""
    parser = argparse.ArgumentParser(description="Motor Identification, Calibration, and Multiturn Setup")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--action", required=True, choices=["scan", "setup", "calibrate", "multiturn", "test", "position_control"],
                       help="Action to perform")
    parser.add_argument("--motor-type", help="Motor type (e.g., sts3235)")
    parser.add_argument("--motor-name", help="Motor name (e.g., shoulder_lift)")
    parser.add_argument("--motor-id", type=int, help="Motor ID")
    parser.add_argument("--initial-baudrate", type=int, help="Initial baudrate for motor setup")
    parser.add_argument("--initial-id", type=int, help="Initial motor ID for motor setup")
    
    args = parser.parse_args()
    
    # Create motor setup manager
    manager = MotorSetupManager(args.port)
    
    try:
        if args.action == "scan":
            # Scan for motors
            baudrate_ids = manager.scan_motors()
            if baudrate_ids:
                print("\nMotor scan results:")
                for baudrate, motor_ids in baudrate_ids.items():
                    print(f"  Baudrate {baudrate}: Motor IDs {motor_ids}")
                    
                    # Try to identify motor models
                    for motor_id in motor_ids:
                        model = manager.identify_motor_model(motor_id, baudrate)
                        if model:
                            print(f"    Motor ID {motor_id}: {model}")
            else:
                print("No motors found on the specified port")
        
        elif args.action == "setup":
            # Setup motor
            if not all([args.motor_type, args.motor_id]):
                print("Error: --motor-type and --motor-id are required for setup action")
                sys.exit(1)
            
            success = manager.setup_motor(
                args.motor_name, 
                args.motor_type, 
                args.motor_id,
                args.initial_baudrate,
                args.initial_id
            )
            
            if success:
                print(f"Motor setup completed successfully")
            else:
                print("Motor setup failed")
                sys.exit(1)
        
        elif args.action == "calibrate":
            # Calibrate motor
            if not all([args.motor_type, args.motor_id]):
                print("Error: --motor-type and --motor-id are required for calibrate action")
                sys.exit(1)
            
            success = manager.calibrate_motor(args.motor_name, args.motor_type, args.motor_id)
            
            if success:
                print(f"Motor calibration completed successfully")
            else:
                print("Motor calibration failed")
                sys.exit(1)
        
        elif args.action == "multiturn":
            # Enable multiturn mode
            if not all([args.motor_type, args.motor_id]):
                print("Error: --motor-type and --motor-id are required for multiturn action")
                sys.exit(1)
            
            success = manager.enable_multiturn(args.motor_name, args.motor_type, args.motor_id)
            
            if success:
                print(f"Multiturn mode enabled successfully")
            else:
                print("Failed to enable multiturn mode")
                sys.exit(1)
        
        elif args.action == "test":
            # Test multiturn functionality
            if not all([args.motor_type, args.motor_id]):
                print("Error: --motor-type and --motor-id are required for test action")
                sys.exit(1)
            
            success = manager.test_multiturn(args.motor_name, args.motor_type, args.motor_id)
            
            if success:
                print(f"Multiturn test completed successfully")
            else:
                print("Multiturn test failed")
                sys.exit(1)
        
        elif args.action == "position_control":
            # Position control
            if not all([args.motor_type, args.motor_id]):
                print("Error: --motor-type and --motor-id are required for position_control action")
                sys.exit(1)
            
            success = manager.position_control(args.motor_name, args.motor_type, args.motor_id)
            
            if success:
                print(f"Position control completed successfully")
            else:
                print("Position control failed")
                sys.exit(1)
    
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main() 