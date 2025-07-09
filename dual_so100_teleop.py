#!/usr/bin/env python3

import time
import threading
from lerobot.common.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.common.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig
from lerobot.common.utils.robot_utils import busy_wait
from lerobot.common.utils.visualization_utils import _init_rerun, log_rerun_data

class DualSO100Teleop:
    def __init__(self):
        # Left arm configuration
        self.left_follower_config = SO100FollowerConfig(
            port="/dev/tty.usbmodem585A0076841",  # Replace with your left follower port
            id="left_follower_arm"
        )
        self.left_leader_config = SO100LeaderConfig(
            port="/dev/tty.usbmodem575E0031751",  # Replace with your left leader port
            id="left_leader_arm"
        )
        
        # Right arm configuration
        self.right_follower_config = SO100FollowerConfig(
            port="/dev/tty.usbmodem585A0076842",  # Replace with your right follower port
            id="right_follower_arm"
        )
        self.right_leader_config = SO100LeaderConfig(
            port="/dev/tty.usbmodem575E0031752",  # Replace with your right leader port
            id="right_leader_arm"
        )
        
        # Initialize robots and teleoperators
        self.left_follower = SO100Follower(self.left_follower_config)
        self.left_leader = SO100Leader(self.left_leader_config)
        self.right_follower = SO100Follower(self.right_follower_config)
        self.right_leader = SO100Leader(self.right_leader_config)
        
        self.running = False
        self.fps = 30
        
    def connect_all(self):
        """Connect all arms and teleoperators"""
        print("Connecting left arm...")
        self.left_follower.connect()
        self.left_leader.connect()
        
        print("Connecting right arm...")
        self.right_follower.connect()
        self.right_leader.connect()
        
        print("All arms connected successfully!")
        
    def disconnect_all(self):
        """Disconnect all arms and teleoperators"""
        self.left_follower.disconnect()
        self.left_leader.disconnect()
        self.right_follower.disconnect()
        self.right_leader.disconnect()
        print("All arms disconnected.")
        
    def teleop_left_arm(self):
        """Teleoperation loop for left arm"""
        while self.running:
            try:
                action = self.left_leader.get_action()
                self.left_follower.send_action(action)
                
                # Optional: visualize left arm data
                observation = self.left_follower.get_observation()
                # Add prefix to action keys for left arm
                left_action = {f"left_{k}": v for k, v in action.items()}
                log_rerun_data(observation, left_action)
                
            except Exception as e:
                print(f"Left arm error: {e}")
                break
                
    def teleop_right_arm(self):
        """Teleoperation loop for right arm"""
        while self.running:
            try:
                action = self.right_leader.get_action()
                self.right_follower.send_action(action)
                
                # Optional: visualize right arm data
                observation = self.right_follower.get_observation()
                # Add prefix to action keys for right arm
                right_action = {f"right_{k}": v for k, v in action.items()}
                log_rerun_data(observation, right_action)
                
            except Exception as e:
                print(f"Right arm error: {e}")
                break
                
    def run(self):
        """Main teleoperation loop"""
        try:
            self.connect_all()
            
            # Initialize visualization
            _init_rerun(session_name="dual_so100_teleop")
            
            self.running = True
            
            # Start teleoperation threads
            left_thread = threading.Thread(target=self.teleop_left_arm)
            right_thread = threading.Thread(target=self.teleop_right_arm)
            
            left_thread.start()
            right_thread.start()
            
            print("Dual SO100 teleoperation started!")
            print("Press Ctrl+C to stop...")
            
            # Main loop for timing
            while self.running:
                time.sleep(1/self.fps)
                
        except KeyboardInterrupt:
            print("\nStopping teleoperation...")
        finally:
            self.running = False
            self.disconnect_all()

if __name__ == "__main__":
    teleop = DualSO100Teleop()
    teleop.run() 