#!/usr/bin/env python3

import time
import threading
from lerobot.common.motors.feetech import FeetechMotorsBus
from lerobot.common.motors import Motor, MotorNormMode

class SimpleDualSO100Teleop:
    def __init__(self, left_follower_port, right_follower_port, left_leader_port, right_leader_port):
        self.left_follower_port = left_follower_port
        self.right_follower_port = right_follower_port
        self.left_leader_port = left_leader_port
        self.right_leader_port = right_leader_port
        
        self.running = False
        self.fps = 30
        
    def connect_all(self):
        """Connect all arms with minimal checks"""
        print("Connecting left follower...")
        try:
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
            self.left_follower_bus.connect()
            print("✓ Left follower connected")
        except Exception as e:
            print(f"✗ Left follower error: {e}")
            return False
            
        print("Connecting right follower...")
        try:
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
            self.right_follower_bus.connect()
            print("✓ Right follower connected")
        except Exception as e:
            print(f"✗ Right follower error: {e}")
            return False
            
        print("Connecting left leader...")
        try:
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
            self.left_leader_bus.connect()
            print("✓ Left leader connected")
        except Exception as e:
            print(f"✗ Left leader error: {e}")
            return False
            
        print("Connecting right leader...")
        try:
            self.right_leader_bus = FeetechMotorsBus(
                port=self.right_leader_port,
                motors={
                    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
                    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
                    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
                    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
                    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
                    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
                }
            )
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
            self.left_leader_bus.disconnect()
            self.right_follower_bus.disconnect()
            self.right_leader_bus.disconnect()
            print("All arms disconnected.")
        except:
            pass
        
    def run(self):
        """Main teleop loop"""
        try:
            if not self.connect_all():
                print("Failed to connect all arms")
                return
                
            self.running = True
            print("Starting dual-arm teleoperation...")
            print("Press Ctrl+C to stop")
            
            while self.running:
                start_time = time.perf_counter()
                
                try:
                    # Get leader positions
                    left_leader_pos = self.left_leader_bus.sync_read("Present_Position")
                    right_leader_pos = self.right_leader_bus.sync_read("Present_Position")
                    
                    # Send to followers
                    self.left_follower_bus.sync_write("Goal_Position", left_leader_pos)
                    self.right_follower_bus.sync_write("Goal_Position", right_leader_pos)
                    
                except Exception as e:
                    print(f"Error in teleop loop: {e}")
                    break
                
                # Maintain target FPS
                time.sleep(1.0 / self.fps)
                
        except KeyboardInterrupt:
            print("\nStopping teleoperation...")
        except Exception as e:
            print(f"Error during teleoperation: {e}")
        finally:
            self.running = False
            self.disconnect_all()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Simple Dual SO100 Teleoperation")
    parser.add_argument("--left-follower-port", required=True, help="Left follower arm port")
    parser.add_argument("--right-follower-port", required=True, help="Right follower arm port")
    parser.add_argument("--left-leader-port", required=True, help="Left leader arm port")
    parser.add_argument("--right-leader-port", required=True, help="Right leader arm port")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS for teleoperation")
    
    args = parser.parse_args()
    
    teleop = SimpleDualSO100Teleop(
        left_follower_port=args.left_follower_port,
        right_follower_port=args.right_follower_port,
        left_leader_port=args.left_leader_port,
        right_leader_port=args.right_leader_port
    )
    
    teleop.fps = args.fps
    teleop.run()

if __name__ == "__main__":
    main() 