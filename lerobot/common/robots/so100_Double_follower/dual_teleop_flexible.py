#!/usr/bin/env python3

import time
import threading
import json
from pathlib import Path
from lerobot.common.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.common.motors import Motor, MotorNormMode, MotorCalibration

class FlexibleDualSO100Teleop:
    def __init__(self):
        self.running = False
        self.fps = 30
        
        # Get configuration from user
        self.config = self.get_configuration()
        
        # Load calibrations
        self.load_calibrations()
        
        # Initialize motor buses
        self.setup_motor_buses()
        
    def get_configuration(self):
        """Get configuration from user input"""
        print("=== FLEXIBLE DUAL SO100 TELEOP CONFIGURATION ===")
        print("This script supports flexible motor ID ranges (1-6 or 7-12) for each arm.")
        print()
        
        config = {}
        arm_names = ["left_follower", "right_follower", "left_leader", "right_leader"]
        
        for arm_name in arm_names:
            print(f"\n--- {arm_name.upper()} ---")
            
            # Get port
            port = input(f"Enter COM port for {arm_name} (e.g., COM3, /dev/ttyACM0): ").strip()
            
            # Get motor ID range
            while True:
                id_range = input(f"Enter motor ID range for {arm_name} (1-6 or 7-12): ").strip()
                if id_range in ["1-6", "7-12"]:
                    break
                print("Please enter either '1-6' or '7-12'")
            
            # Parse motor IDs
            if id_range == "1-6":
                motor_ids = [1, 2, 3, 4, 5, 6]
            else:
                motor_ids = [7, 8, 9, 10, 11, 12]
            
            config[arm_name] = {
                "port": port,
                "motor_ids": motor_ids,
                "id_range": id_range
            }
        
        return config
    
    def load_calibrations(self):
        """Load calibrations for each arm"""
        self.calibrations = {}
        cal_dir = Path.home() / ".lerobot" / "calibrations"
        
        if not cal_dir.exists():
            print("No calibration directory found. Please run calibration first.")
            return
        
        # Try to load individual calibration files
        for arm_name in self.config.keys():
            cal_file = cal_dir / f"{arm_name}_calibration.json"
            if cal_file.exists():
                try:
                    with open(cal_file, 'r') as f:
                        cal_data = json.load(f)
                    
                    # Convert back to MotorCalibration objects
                    calibration = {}
                    for motor_name, motor_data in cal_data.items():
                        if motor_name != "_metadata":
                            calibration[motor_name] = MotorCalibration(
                                id=motor_data["id"],
                                drive_mode=motor_data["drive_mode"],
                                homing_offset=motor_data["homing_offset"],
                                range_min=motor_data["range_min"],
                                range_max=motor_data["range_max"]
                            )
                    
                    self.calibrations[arm_name] = calibration
                    print(f"Loaded calibration for {arm_name}")
                    
                except Exception as e:
                    print(f"Error loading calibration for {arm_name}: {e}")
            else:
                print(f"No calibration file found for {arm_name}")
    
    def create_motor_bus(self, arm_name):
        """Create a motor bus for the specified arm"""
        config = self.config[arm_name]
        motor_ids = config["motor_ids"]
        
        motors = {
            "shoulder_pan": Motor(motor_ids[0], "sts3215", MotorNormMode.RANGE_M100_100),
            "shoulder_lift": Motor(motor_ids[1], "sts3215", MotorNormMode.RANGE_M100_100),
            "elbow_flex": Motor(motor_ids[2], "sts3215", MotorNormMode.RANGE_M100_100),
            "wrist_flex": Motor(motor_ids[3], "sts3215", MotorNormMode.RANGE_M100_100),
            "wrist_roll": Motor(motor_ids[4], "sts3215", MotorNormMode.RANGE_M100_100),
            "gripper": Motor(motor_ids[5], "sts3215", MotorNormMode.RANGE_0_100),
        }
        
        calibration = self.calibrations.get(arm_name, {})
        
        return FeetechMotorsBus(
            port=config["port"],
            motors=motors,
            calibration=calibration
        )
    
    def setup_motor_buses(self):
        """Setup all motor buses"""
        self.left_follower_bus = self.create_motor_bus("left_follower")
        self.right_follower_bus = self.create_motor_bus("right_follower")
        self.left_leader_bus = self.create_motor_bus("left_leader")
        self.right_leader_bus = self.create_motor_bus("right_leader")
    
    def connect_all(self):
        """Connect to all arms"""
        try:
            print("Connecting to left follower...")
            self.left_follower_bus.connect()
            
            print("Connecting to right follower...")
            self.right_follower_bus.connect()
            
            print("Connecting to left leader...")
            self.left_leader_bus.connect()
            
            print("Connecting to right leader...")
            self.right_leader_bus.connect()
            
            print("All arms connected successfully!")
            return True
            
        except Exception as e:
            print(f"Error connecting to arms: {e}")
            return False
    
    def disconnect_all(self):
        """Disconnect from all arms"""
        try:
            if hasattr(self, 'left_follower_bus') and self.left_follower_bus.is_connected:
                self.left_follower_bus.disconnect()
            if hasattr(self, 'right_follower_bus') and self.right_follower_bus.is_connected:
                self.right_follower_bus.disconnect()
            if hasattr(self, 'left_leader_bus') and self.left_leader_bus.is_connected:
                self.left_leader_bus.disconnect()
            if hasattr(self, 'right_leader_bus') and self.right_leader_bus.is_connected:
                self.right_leader_bus.disconnect()
        except Exception as e:
            print(f"Error disconnecting: {e}")
    
    def read_leader_positions(self):
        """Read positions from both leader arms"""
        try:
            left_positions = self.left_leader_bus.sync_read("Present_Position")
            right_positions = self.right_leader_bus.sync_read("Present_Position")
            return left_positions, right_positions
        except Exception as e:
            print(f"Error reading leader positions: {e}")
            return None, None
    
    def send_follower_positions(self, left_positions, right_positions):
        """Send positions to both follower arms"""
        try:
            if left_positions:
                self.left_follower_bus.sync_write("Goal_Position", left_positions)
            if right_positions:
                self.right_follower_bus.sync_write("Goal_Position", right_positions)
        except Exception as e:
            print(f"Error sending follower positions: {e}")
    
    def teleop_loop(self):
        """Main teleop loop"""
        print("Starting teleop loop...")
        print("Move the leader arms to control the follower arms.")
        print("Press Ctrl+C to stop.")
        
        while self.running:
            try:
                # Read leader positions
                left_positions, right_positions = self.read_leader_positions()
                
                # Send to followers
                self.send_follower_positions(left_positions, right_positions)
                
                # Control loop rate
                time.sleep(1.0 / self.fps)
                
            except KeyboardInterrupt:
                print("\nStopping teleop...")
                break
            except Exception as e:
                print(f"Error in teleop loop: {e}")
                break
    
    def start_teleop(self):
        """Start the teleop session"""
        if not self.connect_all():
            print("Failed to connect to all arms")
            return
        
        try:
            self.running = True
            self.teleop_loop()
        except KeyboardInterrupt:
            print("\nTeleop interrupted by user")
        finally:
            self.running = False
            self.disconnect_all()
            print("Teleop session ended")

def main():
    teleop = FlexibleDualSO100Teleop()
    teleop.start_teleop()

if __name__ == "__main__":
    main() 