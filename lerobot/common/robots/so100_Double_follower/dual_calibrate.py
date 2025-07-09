#!/usr/bin/env python3

import time
import json
import os
from pathlib import Path
from lerobot.common.motors.feetech import FeetechMotorsBus
from lerobot.common.motors import Motor, MotorNormMode, MotorCalibration

class DualSO100Calibrator:
    def __init__(self, left_follower_port, right_follower_port, left_leader_port, right_leader_port):
        self.left_follower_port = left_follower_port
        self.right_follower_port = right_follower_port
        self.left_leader_port = left_leader_port
        self.right_leader_port = right_leader_port
        
        # Initialize motor buses
        self.setup_motor_buses()
        
    def setup_motor_buses(self):
        """Setup all motor buses"""
        # Left follower (motor IDs 1-6)
        self.left_follower_bus = FeetechMotorsBus(
            port=self.left_follower_port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            }
        )
        
        # Right follower (motor IDs 7-12)
        self.right_follower_bus = FeetechMotorsBus(
            port=self.right_follower_port,
            motors={
                "shoulder_pan": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(11, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
            }
        )
        
        # Left leader (motor IDs 1-6)
        self.left_leader_bus = FeetechMotorsBus(
            port=self.left_leader_port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            }
        )
        
        # Right leader (motor IDs 7-12)
        self.right_leader_bus = FeetechMotorsBus(
            port=self.right_leader_port,
            motors={
                "shoulder_pan": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "elbow_flex": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_flex": Motor(10, "sts3215", MotorNormMode.RANGE_M100_100),
                "wrist_roll": Motor(11, "sts3215", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(12, "sts3215", MotorNormMode.RANGE_0_100),
            }
        )
    
    def connect_all(self):
        """Connect to all arms"""
        print("Connecting to all arms...")
        
        try:
            self.left_follower_bus.connect()
            print("✓ Left follower connected")
        except Exception as e:
            print(f"✗ Left follower error: {e}")
            return False
            
        try:
            self.right_follower_bus.connect()
            print("✓ Right follower connected")
        except Exception as e:
            print(f"✗ Right follower error: {e}")
            return False
            
        try:
            self.left_leader_bus.connect()
            print("✓ Left leader connected")
        except Exception as e:
            print(f"✗ Left leader error: {e}")
            return False
            
        try:
            self.right_leader_bus.connect()
            print("✓ Right leader connected")
        except Exception as e:
            print(f"✗ Right leader error: {e}")
            return False
            
        print("All arms connected successfully!")
        return True
    
    def disconnect_all(self):
        """Disconnect all arms"""
        try:
            self.left_follower_bus.disconnect()
            self.right_follower_bus.disconnect()
            self.left_leader_bus.disconnect()
            self.right_leader_bus.disconnect()
            print("All arms disconnected.")
        except:
            pass
    
    def calibrate_pair(self, follower_bus, leader_bus, pair_name):
        """Calibrate a leader-follower pair"""
        print(f"\n=== Calibrating {pair_name} ===")
        
        # Disable torque on both arms
        follower_bus.disable_torque()
        leader_bus.disable_torque()
        
        # Set operating mode to position
        for motor in follower_bus.motors:
            follower_bus.write("Operating_Mode", motor, 3)  # Position mode
        for motor in leader_bus.motors:
            leader_bus.write("Operating_Mode", motor, 3)  # Position mode
        
        # Step 1: Move both arms to middle position
        input(f"Move both {pair_name} arms to the middle of their range of motion and press ENTER...")
        
        # Get homing offsets for both arms
        follower_homing = follower_bus.set_half_turn_homings()
        leader_homing = leader_bus.set_half_turn_homings()
        
        # Step 2: Record ranges of motion
        full_turn_motor = "wrist_roll"
        unknown_range_motors = [motor for motor in follower_bus.motors if motor != full_turn_motor]
        
        print(f"Move all joints except '{full_turn_motor}' on both {pair_name} arms through their entire ranges of motion.")
        print("Recording positions. Press ENTER to stop...")
        
        follower_ranges = follower_bus.record_ranges_of_motion(unknown_range_motors)
        leader_ranges = leader_bus.record_ranges_of_motion(unknown_range_motors)
        
        # Set full turn motor ranges
        follower_ranges[0][full_turn_motor] = 0
        follower_ranges[1][full_turn_motor] = 4095
        leader_ranges[0][full_turn_motor] = 0
        leader_ranges[1][full_turn_motor] = 4095
        
        # Create calibration objects
        follower_calibration = {}
        leader_calibration = {}
        
        for motor, m in follower_bus.motors.items():
            follower_calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=follower_homing[motor],
                range_min=follower_ranges[0][motor],
                range_max=follower_ranges[1][motor],
            )
        
        for motor, m in leader_bus.motors.items():
            leader_calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=leader_homing[motor],
                range_min=leader_ranges[0][motor],
                range_max=leader_ranges[1][motor],
            )
        
        # Write calibrations to motors
        follower_bus.write_calibration(follower_calibration)
        leader_bus.write_calibration(leader_calibration)
        
        return follower_calibration, leader_calibration
    
    def save_calibrations(self, left_follower_cal, left_leader_cal, right_follower_cal, right_leader_cal):
        """Save calibrations to files"""
        cal_dir = Path.home() / ".lerobot" / "calibrations"
        cal_dir.mkdir(parents=True, exist_ok=True)
        
        # Save calibrations
        calibrations = {
            "left_follower": left_follower_cal,
            "left_leader": left_leader_cal,
            "right_follower": right_follower_cal,
            "right_leader": right_leader_cal,
            "ports": {
                "left_follower": self.left_follower_port,
                "right_follower": self.right_follower_port,
                "left_leader": self.left_leader_port,
                "right_leader": self.right_leader_port,
            }
        }
        
        cal_file = cal_dir / f"dual_so100_calibration_{self.left_follower_port}_{self.right_follower_port}.json"
        
        # Convert to serializable format
        serializable_cal = {}
        for arm_name, cal in calibrations.items():
            if arm_name != "ports":
                serializable_cal[arm_name] = {}
                for motor_name, motor_cal in cal.items():
                    serializable_cal[arm_name][motor_name] = {
                        "id": motor_cal.id,
                        "drive_mode": motor_cal.drive_mode,
                        "homing_offset": motor_cal.homing_offset,
                        "range_min": motor_cal.range_min,
                        "range_max": motor_cal.range_max,
                    }
            else:
                serializable_cal[arm_name] = cal
        
        with open(cal_file, 'w') as f:
            json.dump(serializable_cal, f, indent=2)
        
        print(f"Calibrations saved to: {cal_file}")
    
    def run_calibration(self):
        """Run the full dual-arm calibration process"""
        try:
            if not self.connect_all():
                print("Failed to connect all arms")
                return False
            
            print("\n=== DUAL-ARM SO100 CALIBRATION ===")
            print("This will calibrate both leader-follower pairs.")
            print("Make sure both arms are in a safe position before starting.")
            input("Press ENTER to continue...")
            
            # Calibrate left pair
            left_follower_cal, left_leader_cal = self.calibrate_pair(
                self.left_follower_bus, self.left_leader_bus, "LEFT ARM"
            )
            
            # Calibrate right pair
            right_follower_cal, right_leader_cal = self.calibrate_pair(
                self.right_follower_bus, self.right_leader_bus, "RIGHT ARM"
            )
            
            # Save all calibrations
            self.save_calibrations(left_follower_cal, left_leader_cal, right_follower_cal, right_leader_cal)
            
            print("\n=== CALIBRATION COMPLETE ===")
            print("All arms have been calibrated successfully!")
            print("You can now run the dual-arm teleop.")
            
            return True
            
        except KeyboardInterrupt:
            print("\nCalibration interrupted by user.")
            return False
        except Exception as e:
            print(f"Error during calibration: {e}")
            return False
        finally:
            self.disconnect_all()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Dual SO100 Calibration")
    parser.add_argument("--left-follower-port", required=True, help="Left follower arm port")
    parser.add_argument("--right-follower-port", required=True, help="Right follower arm port")
    parser.add_argument("--left-leader-port", required=True, help="Left leader arm port")
    parser.add_argument("--right-leader-port", required=True, help="Right leader arm port")
    
    args = parser.parse_args()
    
    calibrator = DualSO100Calibrator(
        left_follower_port=args.left_follower_port,
        right_follower_port=args.right_follower_port,
        left_leader_port=args.left_leader_port,
        right_leader_port=args.right_leader_port
    )
    
    success = calibrator.run_calibration()
    if success:
        print("Calibration completed successfully!")
    else:
        print("Calibration failed.")

if __name__ == "__main__":
    main() 