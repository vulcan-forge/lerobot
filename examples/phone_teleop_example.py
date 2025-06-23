#!/usr/bin/env python

"""
Example usage of PhoneTeleoperator with SO100 robot.

This script demonstrates how to integrate phone teleoperation into the new lerobot architecture.
It uses the PhoneTeleoperator to receive commands from a mobile phone via gRPC and control an SO100 robot.

Requirements:
- Install additional dependencies: pip install pyroki viser yourdfpy
- Ensure the daxie package is installed and the gRPC server is accessible
- Have the robot URDF and mesh files available
- Connect your SO100 robot
"""

import time
from pathlib import Path

from lerobot.common.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.common.teleoperators.phone_teleoperator import PhoneTeleoperator, PhoneTeleoperatorConfig
from lerobot.common.constants import HF_LEROBOT_CALIBRATION, ROBOTS


def find_existing_calibration_id(robot_name: str) -> str | None:
    """Find existing calibration file ID for the robot."""
    calib_dir = HF_LEROBOT_CALIBRATION / ROBOTS / robot_name
    
    if not calib_dir.exists():
        return None
    
    # Look specifically for follower_arm_2.json file
    target_file = calib_dir / "follower_arm_2.json"
    
    if not target_file.exists():
        return None
    
    # Return the specific calibration ID
    return "follower_arm_2"


def main():
    # Get URDF and mesh paths from daxie package
    try:
        from daxie import get_so100_path
        urdf_path, mesh_path = get_so100_path()
        print(f"Using URDF: {urdf_path}")
        print(f"Using meshes: {mesh_path}")
    except ImportError:
        print("ERROR: Could not import daxie package")
        print("Make sure the daxie package is installed and accessible")
        return
    
    # Find existing calibration or use default ID
    existing_id = find_existing_calibration_id("so100_follower")
    
    if existing_id:
        robot_id = existing_id
        print(f"Found existing calibration for ID: {robot_id}")
    else:
        print("No existing calibration found. Exiting.")
        return
    
    # Configuration for the SO100 follower robot
    robot_config = SO100FollowerConfig(
        id=robot_id,
        port="COM11",  # Adjust based on your setup - could be /dev/ttyUSB1, COM3, etc.
        use_degrees=True,
        max_relative_target=30.0,  # Safety limit in degrees
    )
    
    # Configuration for the phone teleoperator
    phone_config = PhoneTeleoperatorConfig(
        id="phone_teleop_main",
        urdf_path=urdf_path,
        mesh_path=mesh_path,
        target_link_name="Fixed_Jaw",
        grpc_port=8765,  # Match your phone app's expected port
        sensitivity_normal=0.5,
        sensitivity_precision=0.2,
        rotation_sensitivity=1.0,
        initial_position=(0.0, -0.17, 0.237),
        initial_wxyz=(0, 0, 1, 0),  # wxyz quaternion
        rest_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),  # radians - conservative middle position
        enable_visualization=True,
        viser_port=8080,
    )
    
    # Initialize robot and teleoperator
    robot = SO100Follower(robot_config)
    phone_teleop = PhoneTeleoperator(phone_config)
    
    try:
        # Connect devices
        print("Connecting to robot...")
        robot.connect()
        
        print("Connecting to phone teleoperator...")
        phone_teleop.connect()
        
        print("Phone teleoperation ready!")
        print("- Start the phone app and connect to the gRPC server")
        print("- Use your phone to control the robot")
        print("- Press Ctrl+C to exit")
        
        # Main control loop
        debug_timer = 0
        debug_printed = False
        
        while True:
            start_time = time.perf_counter()
            debug_timer += 1
            
            try:
                # Get current observation first
                observation = robot.get_observation()
                
                # Debug: Print observation after ~5 seconds
                if debug_timer == 50 and not debug_printed:
                    print(f"\n=== NEW SYSTEM MAIN LOOP DEBUG ===")
                    print(f"Step 1: Robot observation")
                    print(f"  observation keys: {list(observation.keys())}")
                    print(f"  observation values: {observation}")
                    debug_printed = True
                
                # Get action from phone (pass observation for current robot state)
                action = phone_teleop.get_action(observation)
                
                # Debug: Print action after ~5 seconds
                if debug_timer == 50 and debug_printed:
                    print(f"\nStep 2: Phone teleoperator action")
                    print(f"  action keys: {list(action.keys())}")
                    print(f"  action values: {action}")
                
                # Send action to robot
                actual_action = robot.send_action(action)
                
                # Debug: Print actual action after ~5 seconds
                if debug_timer == 50 and debug_printed:
                    print(f"\nStep 3: Robot send_action result")
                    print(f"  actual_action: {actual_action}")
                    print(f"========================================\n")
                
                # Log timing (optional)
                loop_time = time.perf_counter() - start_time
                if loop_time > 0.1:  # Log if loop takes more than 100ms
                    print(f"Loop time: {loop_time*1000:.1f}ms")
                
                # Control frequency (adjust as needed)
                time.sleep(max(0, 1/30 - loop_time))  # Target ~30 Hz
                
            except KeyboardInterrupt:
                print("\nStopping teleoperation...")
                break
            except Exception as e:
                print(f"Error in control loop: {e}")
                # Continue running but with a small delay
                time.sleep(0.1)
    
    finally:
        # Cleanup
        print("Disconnecting devices...")
        try:
            phone_teleop.disconnect()
        except:
            pass
        try:
            robot.disconnect()
        except:
            pass
        print("Done!")


if __name__ == "__main__":
    main() 