#!/usr/bin/env python3

import time
import threading
from lerobot.common.robots.so100_Double_follower.so100_follower import SO100Follower, SO100FollowerConfig
from lerobot.common.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig
from lerobot.common.utils.robot_utils import busy_wait

class DualSO100Teleop:
    def __init__(self, left_follower_port, right_follower_port, left_leader_port, right_leader_port):
        # Follower configuration (dual-arm)
        self.follower_config = SO100FollowerConfig(
            left_port=left_follower_port,
            right_port=right_follower_port,
            id="dual_so100_follower"
        )
        
        # Left leader configuration
        self.left_leader_config = SO100LeaderConfig(
            port=left_leader_port,
            id="left_so100_leader"
        )
        
        # Right leader configuration
        self.right_leader_config = SO100LeaderConfig(
            port=right_leader_port,
            id="right_so100_leader"
        )
        
        # Initialize robots and teleoperators
        self.follower = SO100Follower(self.follower_config)
        self.left_leader = SO100Leader(self.left_leader_config)
        self.right_leader = SO100Leader(self.right_leader_config)
        
        self.running = False
        self.fps = 30
        
    def connect_all(self):
        """Connect all arms and teleoperators"""
        print("Connecting dual-arm follower...")
        self.follower.connect()
        
        print("Connecting left leader...")
        self.left_leader.connect()
        
        print("Connecting right leader...")
        self.right_leader.connect()
        
        print("All arms connected successfully!")
        
    def disconnect_all(self):
        """Disconnect all arms and teleoperators"""
        self.follower.disconnect()
        self.left_leader.disconnect()
        self.right_leader.disconnect()
        print("All arms disconnected.")
        
    def run(self):
        """Main teleop loop"""
        try:
            self.connect_all()
            self.running = True
            
            print("Starting dual-arm teleoperation...")
            print("Press Ctrl+C to stop")
            
            while self.running:
                start_time = time.perf_counter()
                
                # Get leader actions
                left_action = self.left_leader.get_action()
                right_action = self.right_leader.get_action()
                
                # Combine actions with prefixes
                combined_action = {}
                
                # Add left arm actions with left_ prefix
                for key, value in left_action.items():
                    combined_action[f"left_{key}"] = value
                
                # Add right arm actions with right_ prefix
                for key, value in right_action.items():
                    combined_action[f"right_{key}"] = value
                
                # Send combined action to follower
                self.follower.send_action(combined_action)
                
                # Maintain target FPS
                busy_wait(start_time, 1.0 / self.fps)
                
        except KeyboardInterrupt:
            print("\nStopping teleoperation...")
        except Exception as e:
            print(f"Error during teleoperation: {e}")
        finally:
            self.running = False
            self.disconnect_all()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Dual SO100 Teleoperation")
    parser.add_argument("--left-follower-port", required=True, help="Left follower arm port (e.g., /dev/ttyUSB3)")
    parser.add_argument("--right-follower-port", required=True, help="Right follower arm port (e.g., /dev/ttyUSB2)")
    parser.add_argument("--left-leader-port", required=True, help="Left leader arm port (e.g., /dev/ttyUSB1)")
    parser.add_argument("--right-leader-port", required=True, help="Right leader arm port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS for teleoperation")
    
    args = parser.parse_args()
    
    teleop = DualSO100Teleop(
        left_follower_port=args.left_follower_port,
        right_follower_port=args.right_follower_port,
        left_leader_port=args.left_leader_port,
        right_leader_port=args.right_leader_port
    )
    
    teleop.fps = args.fps
    teleop.run()

if __name__ == "__main__":
    main() 