#!/usr/bin/env python3

import time
import threading
import argparse
from enum import Enum
from lerobot.common.robots.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.common.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig
from lerobot.common.utils.robot_utils import busy_wait
from lerobot.common.utils.visualization_utils import _init_rerun, log_rerun_data

class ControlMode(Enum):
    INDEPENDENT = "independent"  # Each arm controlled by its own leader
    SYNCHRONIZED = "synchronized"  # Both arms follow the same leader
    MIRRORED = "mirrored"  # Arms move in opposite directions
    RECORDING = "recording"  # Record training data

class AdvancedDualSO100Teleop:
    def __init__(self, left_follower_port, left_leader_port, 
                 right_follower_port, right_leader_port, control_mode):
        self.control_mode = ControlMode(control_mode)
        
        # Left arm configuration
        self.left_follower_config = SO100FollowerConfig(
            port=left_follower_port,
            id="left_follower_arm"
        )
        self.left_leader_config = SO100LeaderConfig(
            port=left_leader_port,
            id="left_leader_arm"
        )
        
        # Right arm configuration
        self.right_follower_config = SO100FollowerConfig(
            port=right_follower_port,
            id="right_follower_arm"
        )
        self.right_leader_config = SO100LeaderConfig(
            port=right_leader_port,
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
        
    def independent_control(self):
        """Independent control mode - each arm controlled by its own leader"""
        print("Starting independent control mode...")
        
        def left_control():
            while self.running:
                try:
                    action = self.left_leader.get_action()
                    self.left_follower.send_action(action)
                    time.sleep(1/self.fps)
                except Exception as e:
                    print(f"Left arm error: {e}")
                    break
                    
        def right_control():
            while self.running:
                try:
                    action = self.right_leader.get_action()
                    self.right_follower.send_action(action)
                    time.sleep(1/self.fps)
                except Exception as e:
                    print(f"Right arm error: {e}")
                    break
        
        # Start both control threads
        left_thread = threading.Thread(target=left_control)
        right_thread = threading.Thread(target=right_control)
        
        left_thread.start()
        right_thread.start()
        
        return left_thread, right_thread
        
    def synchronized_control(self):
        """Synchronized control mode - both arms follow the left leader"""
        print("Starting synchronized control mode...")
        
        def synchronized_control_loop():
            while self.running:
                try:
                    # Get action from left leader
                    action = self.left_leader.get_action()
                    
                    # Apply same action to both arms
                    self.left_follower.send_action(action)
                    self.right_follower.send_action(action)
                    
                    time.sleep(1/self.fps)
                except Exception as e:
                    print(f"Synchronized control error: {e}")
                    break
        
        thread = threading.Thread(target=synchronized_control_loop)
        thread.start()
        return thread
        
    def mirrored_control(self):
        """Mirrored control mode - arms move in opposite directions"""
        print("Starting mirrored control mode...")
        
        def mirrored_control_loop():
            while self.running:
                try:
                    # Get action from left leader
                    action = self.left_leader.get_action()
                    
                    # Apply action to left arm
                    self.left_follower.send_action(action)
                    
                    # Mirror action for right arm (invert certain joints)
                    mirrored_action = {}
                    for key, value in action.items():
                        if "shoulder_pan" in key or "wrist_roll" in key:
                            # Invert pan and roll movements
                            mirrored_action[key] = -value
                        else:
                            # Keep other movements the same
                            mirrored_action[key] = value
                    
                    self.right_follower.send_action(mirrored_action)
                    
                    time.sleep(1/self.fps)
                except Exception as e:
                    print(f"Mirrored control error: {e}")
                    break
        
        thread = threading.Thread(target=mirrored_control_loop)
        thread.start()
        return thread
        
    def run(self):
        """Main teleoperation loop"""
        try:
            self.connect_all()
            
            # Initialize visualization
            _init_rerun(session_name=f"dual_so100_{self.control_mode.value}")
            
            self.running = True
            
            # Start control based on mode
            if self.control_mode == ControlMode.INDEPENDENT:
                left_thread, right_thread = self.independent_control()
                threads = [left_thread, right_thread]
            elif self.control_mode == ControlMode.SYNCHRONIZED:
                thread = self.synchronized_control()
                threads = [thread]
            elif self.control_mode == ControlMode.MIRRORED:
                thread = self.mirrored_control()
                threads = [thread]
            else:
                raise ValueError(f"Unsupported control mode: {self.control_mode}")
            
            print(f"Dual SO100 teleoperation started in {self.control_mode.value} mode!")
            print("Press Ctrl+C to stop...")
            
            # Wait for threads to complete
            for thread in threads:
                thread.join()
                
        except KeyboardInterrupt:
            print("\nStopping teleoperation...")
        finally:
            self.running = False
            self.disconnect_all()

def main():
    parser = argparse.ArgumentParser(description="Advanced Dual SO100 Teleoperation")
    parser.add_argument("--left-follower-port", required=True, help="Left follower arm port")
    parser.add_argument("--left-leader-port", required=True, help="Left leader arm port")
    parser.add_argument("--right-follower-port", required=True, help="Right follower arm port")
    parser.add_argument("--right-leader-port", required=True, help="Right leader arm port")
    parser.add_argument("--control-mode", choices=["independent", "synchronized", "mirrored"], 
                       default="independent", help="Control mode for the arms")
    
    args = parser.parse_args()
    
    teleop = AdvancedDualSO100Teleop(
        left_follower_port=args.left_follower_port,
        left_leader_port=args.left_leader_port,
        right_follower_port=args.right_follower_port,
        right_leader_port=args.right_leader_port,
        control_mode=args.control_mode
    )
    
    teleop.run()

if __name__ == "__main__":
    main() 