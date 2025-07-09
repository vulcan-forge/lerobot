#!/usr/bin/env python3

import json
from pathlib import Path
from lerobot.common.motors import MotorCalibration

def test_calibration_loading():
    """Test loading calibration files for different motor ID ranges"""
    cal_dir = Path.home() / ".lerobot" / "calibrations"
    
    if not cal_dir.exists():
        print("No calibration directory found.")
        return
    
    print("=== TESTING CALIBRATION FILE LOADING ===")
    
    # Look for calibration files
    for cal_file in cal_dir.glob("*_calibration.json"):
        print(f"\nTesting file: {cal_file.name}")
        
        try:
            with open(cal_file, 'r') as f:
                cal_data = json.load(f)
            
            # Check if it has metadata
            if "_metadata" in cal_data:
                metadata = cal_data["_metadata"]
                print(f"  Arm name: {metadata.get('arm_name', 'Unknown')}")
                print(f"  Port: {metadata.get('port', 'Unknown')}")
                print(f"  Motor IDs: {metadata.get('motor_ids', 'Unknown')}")
                print(f"  ID range: {metadata.get('id_range', 'Unknown')}")
            
            # Check motor calibrations
            motor_count = 0
            for motor_name, motor_data in cal_data.items():
                if motor_name != "_metadata":
                    motor_count += 1
                    print(f"  Motor {motor_name}: ID {motor_data.get('id', 'Unknown')}")
            
            print(f"  Total motors: {motor_count}")
            
            # Test creating MotorCalibration objects
            calibrations = {}
            for motor_name, motor_data in cal_data.items():
                if motor_name != "_metadata":
                    try:
                        calibrations[motor_name] = MotorCalibration(
                            id=motor_data["id"],
                            drive_mode=motor_data["drive_mode"],
                            homing_offset=motor_data["homing_offset"],
                            range_min=motor_data["range_min"],
                            range_max=motor_data["range_max"]
                        )
                        print(f"  ✓ Successfully created calibration for {motor_name}")
                    except Exception as e:
                        print(f"  ✗ Error creating calibration for {motor_name}: {e}")
            
            print(f"  ✓ File loaded successfully")
            
        except Exception as e:
            print(f"  ✗ Error loading file: {e}")
    
    # Check for combined calibration file
    combined_file = cal_dir / "dual_so100_combined_calibration.json"
    if combined_file.exists():
        print(f"\nFound combined calibration file: {combined_file.name}")
        try:
            with open(combined_file, 'r') as f:
                combined_data = json.load(f)
            
            if "arms" in combined_data:
                print(f"  Contains {len(combined_data['arms'])} arm configurations")
                for arm_name, arm_data in combined_data['arms'].items():
                    print(f"    {arm_name}: {arm_data.get('id_range', 'Unknown')} on {arm_data.get('port', 'Unknown')}")
            
            print(f"  ✓ Combined file loaded successfully")
            
        except Exception as e:
            print(f"  ✗ Error loading combined file: {e}")

def create_sample_calibration():
    """Create a sample calibration file for testing"""
    cal_dir = Path.home() / ".lerobot" / "calibrations"
    cal_dir.mkdir(parents=True, exist_ok=True)
    
    # Sample calibration for motor IDs 7-12
    sample_cal = {
        "shoulder_pan": {
            "id": 7,
            "drive_mode": 0,
            "homing_offset": 2048,
            "range_min": 0,
            "range_max": 4095
        },
        "shoulder_lift": {
            "id": 8,
            "drive_mode": 0,
            "homing_offset": 2048,
            "range_min": 0,
            "range_max": 4095
        },
        "elbow_flex": {
            "id": 9,
            "drive_mode": 0,
            "homing_offset": 2048,
            "range_min": 0,
            "range_max": 4095
        },
        "wrist_flex": {
            "id": 10,
            "drive_mode": 0,
            "homing_offset": 2048,
            "range_min": 0,
            "range_max": 4095
        },
        "wrist_roll": {
            "id": 11,
            "drive_mode": 0,
            "homing_offset": 2048,
            "range_min": 0,
            "range_max": 4095
        },
        "gripper": {
            "id": 12,
            "drive_mode": 0,
            "homing_offset": 2048,
            "range_min": 0,
            "range_max": 4095
        },
        "_metadata": {
            "port": "COM3",
            "motor_ids": [7, 8, 9, 10, 11, 12],
            "id_range": "7-12",
            "arm_name": "right_follower"
        }
    }
    
    sample_file = cal_dir / "right_follower_calibration.json"
    with open(sample_file, 'w') as f:
        json.dump(sample_cal, f, indent=2)
    
    print(f"Created sample calibration file: {sample_file}")

if __name__ == "__main__":
    print("Choose an option:")
    print("1. Test loading existing calibration files")
    print("2. Create sample calibration file for testing")
    
    choice = input("Enter choice (1 or 2): ").strip()
    
    if choice == "1":
        test_calibration_loading()
    elif choice == "2":
        create_sample_calibration()
    else:
        print("Invalid choice") 