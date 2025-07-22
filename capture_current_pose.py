#!/usr/bin/env python3
"""
Capture Current Arm Pose
========================

This script reads the current motor positions and calculates the Cartesian coordinates
of the robot arm. Use this to set both the rest_pose and initial_position/initial_wxyz
in your phone teleoperator config.

Usage:
    python capture_current_pose.py

The script will output:
- Motor positions in degrees and radians (for rest_pose)
- Cartesian position and orientation (for initial_position and initial_wxyz)
"""

import time
import numpy as np
import json
from scipy.spatial.transform import Rotation as R
from lerobot.common.model.kinematics import RobotKinematics
from lerobot.common.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.common.motors import MotorCalibration


def capture_current_pose():
    """Capture the current pose of the robot arm."""
    
    print("=" * 60)
    print("🤖 CAPTURING CURRENT ARM POSE")
    print("=" * 60)
    print()
    
    # Load calibration file and copy to correct location
    print("📋 Loading calibration file...")
    try:
        # Load the calibration data
        with open("so100_follower_main.json", "r") as f:
            calib_data = json.load(f)
        
        # Import constants to get the correct calibration directory
        from lerobot.common.constants import HF_LEROBOT_CALIBRATION, ROBOTS
        
        # Create the calibration directory structure
        calib_dir = HF_LEROBOT_CALIBRATION / ROBOTS / "so100_follower"
        calib_dir.mkdir(parents=True, exist_ok=True)
        
        # Copy calibration file to the correct location
        calib_file = calib_dir / "so100_follower_main.json"
        import shutil
        shutil.copy("so100_follower_main.json", calib_file)
        print(f"✅ Calibration copied to: {calib_file}")
        
    except Exception as e:
        print(f"❌ Failed to load/copy calibration: {e}")
        return
    
    # Configuration for the SO100 follower robot (same as phone teleop example)
    robot_config = SO100FollowerConfig(
        id="so100_follower_main",
        port="COM11",  # Update this to your port
        use_degrees=True,
        max_relative_target=30.0,  # Safety limit in degrees
    )
    
    # Initialize robot (calibration will be loaded automatically from the correct location)
    print("📡 Connecting to robot...")
    try:
        robot = SO100Follower(robot_config)
        # Connect without calibration since we have the calibration file
        robot.connect(calibrate=False)
        print("✅ Connected successfully!")
    except Exception as e:
        print(f"❌ Failed to connect to robot: {e}")
        print("\n💡 Make sure:")
        print("   - Robot is powered on and connected")
        print("   - Correct port is specified (currently COM11)")
        print("   - Calibration file 'so100_follower_main.json' exists")
        return
    
    try:
        print("\n📊 Reading current motor positions...")
        
        # Get current observation
        observation = robot.get_observation()
        
        # Extract motor positions (in degrees)
        motor_keys = ["shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos", 
                     "wrist_flex.pos", "wrist_roll.pos", "gripper.pos"]
        
        current_joint_pos_deg = []
        motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
        
        print("\n🔧 MOTOR POSITIONS:")
        print("-" * 40)
        
        for i, (key, name) in enumerate(zip(motor_keys, motor_names)):
            pos_deg = observation.get(key, 0.0)
            pos_rad = np.deg2rad(pos_deg)
            current_joint_pos_deg.append(pos_deg)
            print(f"  {name:<15}: {pos_deg:>8.3f}° ({pos_rad:>9.6f} rad)")
        
        # Convert to radians for rest_pose format
        current_joint_pos_rad = np.deg2rad(current_joint_pos_deg)
        
        print("\n📋 REST_POSE FORMAT (for config file):")
        print("-" * 40)
        formatted_rad = ", ".join([f"{pos:.6f}" for pos in current_joint_pos_rad])
        print(f"rest_pose: tuple[float, ...] = ({formatted_rad})")
        
        # Calculate Cartesian coordinates using forward kinematics (same system as phone teleop)
        print("\n🧮 Calculating Cartesian coordinates...")
        
        try:
            # Use the same Pyroki + URDF system as phone teleop
            import pyroki as pk
            import yourdfpy
            from pathlib import Path
            import lerobot
            
            # Get the same URDF path that phone teleop uses
            lerobot_root = Path(lerobot.__file__).parent.parent
            urdf_path = str(lerobot_root / "lerobot" / "common" / "robots" / "so100_follower" / "model" / "so100.urdf")
            mesh_path = str(lerobot_root / "lerobot" / "common" / "robots" / "so100_follower" / "model" / "meshes")
            
            print(f"Loading URDF from: {urdf_path}")
            print(f"Mesh directory: {mesh_path}")
            
            # Load the same URDF model as phone teleop
            urdf = yourdfpy.URDF.load(urdf_path, mesh_dir=mesh_path)
            robot_model = pk.Robot.from_urdf(urdf)
            
            # Convert joint positions to radians (Pyroki expects radians)
            joint_angles_rad = np.array(current_joint_pos_rad, dtype=np.float32)
            
            print(f"Joint angles (radians): {joint_angles_rad}")
            
            # Use the same target link as phone teleop
            target_link_name = "Feetech-Servo-Motor-v1-5"
            
            print(f"Target link: {target_link_name}")
            
            # Get forward kinematics using Pyroki (same as phone teleop)
            T = robot_model.forward_kinematics(joint_angles_rad, target_link_name)
            
            print(f"Forward kinematics result shape: {T.shape}")
            
            # Extract position and orientation
            position = T[:3, 3]  # x, y, z in meters
            rot_matrix = T[:3, :3]
            
            # Convert rotation matrix to quaternion (w, x, y, z)
            rot = R.from_matrix(rot_matrix)
            q_xyzw = rot.as_quat()  # (x, y, z, w)
            quaternion_wxyz = (q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2])  # (w, x, y, z)
            
            print("\n📍 CARTESIAN COORDINATES:")
            print("-" * 40)
            print(f"  Position (meters):")
            print(f"    x: {position[0]:>10.6f}")
            print(f"    y: {position[1]:>10.6f}")
            print(f"    z: {position[2]:>10.6f}")
            print(f"  Orientation (quaternion w,x,y,z):")
            print(f"    w: {quaternion_wxyz[0]:>10.6f}")
            print(f"    x: {quaternion_wxyz[1]:>10.6f}")
            print(f"    y: {quaternion_wxyz[2]:>10.6f}")
            print(f"    z: {quaternion_wxyz[3]:>10.6f}")
            
            print("\n📋 INITIAL POSITION/ROTATION FORMAT (for config file):")
            print("-" * 55)
            pos_formatted = f"{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}"
            quat_formatted = f"{quaternion_wxyz[0]:.6f}, {quaternion_wxyz[1]:.6f}, {quaternion_wxyz[2]:.6f}, {quaternion_wxyz[3]:.6f}"
            
            print(f"initial_position: tuple[float, ...] = ({pos_formatted})  # meters")
            print(f"initial_wxyz: tuple[float, ...] = ({quat_formatted})  # quaternion (w,x,y,z)")
            
        except Exception as e:
            print(f"❌ Failed to calculate Cartesian coordinates: {e}")
            print("   The motor positions above are still valid for rest_pose")
        
        print("\n" + "=" * 60)
        print("✅ POSE CAPTURE COMPLETE!")
        print("=" * 60)
        print("\n💡 Copy the values above into your phone teleoperator config file:")
        print("   lerobot/common/teleoperators/phone_teleoperator/config_phone_teleoperator.py")
        print("\n🔄 You can run this script multiple times to fine-tune your position.")
        
    except Exception as e:
        print(f"❌ Error reading robot state: {e}")
    
    finally:
        try:
            robot.disconnect()
            print("\n📡 Robot disconnected.")
        except:
            pass


if __name__ == "__main__":
    print("🚀 Starting pose capture...")
    print("📋 Make sure your robot arm is in the desired position before running!")
    print()
    
    # Give user a moment to read
    time.sleep(1)
    
    capture_current_pose() 