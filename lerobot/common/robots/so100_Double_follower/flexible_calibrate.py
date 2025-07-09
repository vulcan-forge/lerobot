#!/usr/bin/env python3

import time
import json
import os
from pathlib import Path
from lerobot.common.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.common.motors import Motor, MotorNormMode, MotorCalibration

class FlexibleSO100Calibrator:
    def __init__(self):
        self.calibrations = {}
        
    def get_arm_config(self, arm_name):
        """Get configuration for a single arm"""
        print(f"\n=== {arm_name} CONFIGURATION ===")
        
        # Get port
        port = input(f"Enter the COM port for {arm_name} (e.g., COM3, /dev/ttyACM0): ").strip()
        
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
        
        return {
            "port": port,
            "motor_ids": motor_ids,
            "id_range": id_range
        }
    
    def create_motor_bus(self, port, motor_ids, arm_name):
        """Create a motor bus with the specified motor IDs"""
        motors = {
            "shoulder_pan": Motor(motor_ids[0], "sts3215", MotorNormMode.RANGE_M100_100),
            "shoulder_lift": Motor(motor_ids[1], "sts3215", MotorNormMode.RANGE_M100_100),
            "elbow_flex": Motor(motor_ids[2], "sts3215", MotorNormMode.RANGE_M100_100),
            "wrist_flex": Motor(motor_ids[3], "sts3215", MotorNormMode.RANGE_M100_100),
            "wrist_roll": Motor(motor_ids[4], "sts3215", MotorNormMode.RANGE_M100_100),
            "gripper": Motor(motor_ids[5], "sts3215", MotorNormMode.RANGE_0_100),
        }
        
        return FeetechMotorsBus(port=port, motors=motors)
    
    def calibrate_arm(self, bus, arm_name):
        """Calibrate a single arm"""
        print(f"\n=== CALIBRATING {arm_name} ===")
        print(f"Motor IDs: {[m.id for m in bus.motors.values()]}")
        print(f"Port: {bus.port}")
        
        try:
            # Connect to the bus
            bus.connect()
            print(f"Connected to {arm_name}")
            
            # Disable torque for safe calibration
            bus.disable_torque()
            for motor in bus.motors:
                bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            
            # Get homing offsets
            input(f"Move {arm_name} to the middle of its range of motion and press ENTER....")
            homing_offsets = bus.set_half_turn_homings()
            
            # Record ranges of motion
            full_turn_motor = "wrist_roll"
            unknown_range_motors = [motor for motor in bus.motors if motor != full_turn_motor]
            print(
                f"Move all joints except '{full_turn_motor}' sequentially through their "
                "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
            )
            range_mins, range_maxes = bus.record_ranges_of_motion(unknown_range_motors)
            range_mins[full_turn_motor] = 0
            range_maxes[full_turn_motor] = 4095
            
            # Create calibration
            calibration = {}
            for motor, m in bus.motors.items():
                calibration[motor] = MotorCalibration(
                    id=m.id,
                    drive_mode=0,
                    homing_offset=homing_offsets[motor],
                    range_min=range_mins[motor],
                    range_max=range_maxes[motor],
                )
            
            # Write calibration to motors
            bus.write_calibration(calibration)
            
            print(f"Calibration completed for {arm_name}")
            return calibration
            
        except Exception as e:
            print(f"Error calibrating {arm_name}: {e}")
            return None
        finally:
            if bus.is_connected:
                bus.disconnect()
    
    def save_calibrations(self, calibrations):
        """Save all calibrations to files"""
        cal_dir = Path.home() / ".lerobot" / "calibrations"
        cal_dir.mkdir(parents=True, exist_ok=True)
        
        # Save individual calibration files for each arm
        for arm_name, cal_data in calibrations.items():
            if cal_data["calibration"]:
                # Create serializable calibration
                serializable_cal = {}
                for motor_name, motor_cal in cal_data["calibration"].items():
                    serializable_cal[motor_name] = {
                        "id": motor_cal.id,
                        "drive_mode": motor_cal.drive_mode,
                        "homing_offset": motor_cal.homing_offset,
                        "range_min": motor_cal.range_min,
                        "range_max": motor_cal.range_max,
                    }
                
                # Add metadata
                serializable_cal["_metadata"] = {
                    "port": cal_data["port"],
                    "motor_ids": cal_data["motor_ids"],
                    "id_range": cal_data["id_range"],
                    "arm_name": arm_name
                }
                
                # Save to file
                cal_file = cal_dir / f"{arm_name}_calibration.json"
                with open(cal_file, 'w') as f:
                    json.dump(serializable_cal, f, indent=2)
                
                print(f"Calibration saved to: {cal_file}")
        
        # Save combined calibration file
        combined_cal = {
            "arms": calibrations,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        combined_file = cal_dir / "dual_so100_combined_calibration.json"
        with open(combined_file, 'w') as f:
            json.dump(combined_cal, f, indent=2, default=str)
        
        print(f"Combined calibration saved to: {combined_file}")
    
    def run_calibration(self):
        """Run the full flexible calibration process"""
        print("=== FLEXIBLE SO100 DUAL-ARM CALIBRATION ===")
        print("This script will calibrate up to 4 arms with flexible motor ID ranges.")
        print("You can calibrate 1-4 arms depending on your setup.")
        print()
        
        # Get number of arms to calibrate
        while True:
            try:
                num_arms = int(input("How many arms do you want to calibrate? (1-4): "))
                if 1 <= num_arms <= 4:
                    break
                print("Please enter a number between 1 and 4")
            except ValueError:
                print("Please enter a valid number")
        
        # Get configurations for each arm
        arm_configs = {}
        arm_names = ["left_follower", "right_follower", "left_leader", "right_leader"]
        
        for i in range(num_arms):
            arm_name = arm_names[i]
            arm_configs[arm_name] = self.get_arm_config(arm_name)
        
        # Calibrate each arm
        calibrations = {}
        for arm_name, config in arm_configs.items():
            print(f"\nPreparing to calibrate {arm_name}...")
            input("Press ENTER when ready to start calibration...")
            
            bus = self.create_motor_bus(config["port"], config["motor_ids"], arm_name)
            calibration = self.calibrate_arm(bus, arm_name)
            
            calibrations[arm_name] = {
                "port": config["port"],
                "motor_ids": config["motor_ids"],
                "id_range": config["id_range"],
                "calibration": calibration
            }
        
        # Save calibrations
        self.save_calibrations(calibrations)
        
        print("\n=== CALIBRATION COMPLETE ===")
        print("All arms have been calibrated successfully!")
        print("You can now run the dual-arm teleop with the updated script.")

def main():
    calibrator = FlexibleSO100Calibrator()
    calibrator.run_calibration()

if __name__ == "__main__":
    main() 