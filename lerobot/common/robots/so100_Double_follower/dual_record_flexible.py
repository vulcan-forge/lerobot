#!/usr/bin/env python3

import time
import json
import argparse
import numpy as np
from pathlib import Path
from lerobot.common.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.common.motors import Motor, MotorNormMode, MotorCalibration
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.utils import hw_to_dataset_features
from lerobot.common.utils.control_utils import init_keyboard_listener
from lerobot.common.utils.utils import log_say
from lerobot.common.utils.visualization_utils import _init_rerun
from lerobot.record import record_loop

class FlexibleDualSO100Recorder:
    def __init__(self, config):
        self.config = config
        self.running = False
        self.fps = config.get('fps', 30)
        
        # Load calibrations
        self.load_calibrations()
        
        # Initialize motor buses
        self.setup_motor_buses()
        
        # Initialize dataset
        self.setup_dataset()
        
    def load_calibrations(self):
        """Load calibrations for each arm"""
        self.calibrations = {}
        cal_dir = Path.home() / ".lerobot" / "calibrations"
        
        if not cal_dir.exists():
            print("No calibration directory found. Please run calibration first.")
            return
        
        # Try to load individual calibration files
        for arm_name in self.config['arms'].keys():
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
        arm_config = self.config['arms'][arm_name]
        motor_ids = arm_config["motor_ids"]
        
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
            port=arm_config["port"],
            motors=motors,
            calibration=calibration
        )
    
    def setup_motor_buses(self):
        """Setup all motor buses"""
        self.follower_buses = {}
        self.leader_buses = {}
        
        # Setup follower buses
        for arm_name in ['left_follower', 'right_follower']:
            if arm_name in self.config['arms']:
                self.follower_buses[arm_name] = self.create_motor_bus(arm_name)
        
        # Setup leader buses
        for arm_name in ['left_leader', 'right_leader']:
            if arm_name in self.config['arms']:
                self.leader_buses[arm_name] = self.create_motor_bus(arm_name)
    
    def setup_dataset(self):
        """Setup the dataset for recording"""
        # Create action features for all arms
        action_features = {}
        for arm_name, bus in self.follower_buses.items():
            prefix = arm_name.replace('_follower', '')
            for motor_name in bus.motors:
                action_features[f"{prefix}_{motor_name}.pos"] = {
                    "dtype": "float32",
                    "shape": (1,),
                    "names": [f"{prefix}_{motor_name}.pos"]
                }
        
        # Create observation features for all arms
        observation_features = {}
        for arm_name, bus in self.follower_buses.items():
            prefix = arm_name.replace('_follower', '')
            for motor_name in bus.motors:
                observation_features[f"{prefix}_{motor_name}.pos"] = {
                    "dtype": "float32",
                    "shape": (1,),
                    "names": [f"{prefix}_{motor_name}.pos"]
                }
        
        # Combine features
        dataset_features = {**action_features, **observation_features}
        
        # Create dataset
        self.dataset = LeRobotDataset.create(
            self.config['dataset']['repo_id'],
            self.fps,
            features=dataset_features,
            robot_type="dual_so100_follower",
            use_videos=True,
            image_writer_threads=4,
        )
    
    def connect_all(self):
        """Connect to all arms"""
        try:
            # Connect followers
            for arm_name, bus in self.follower_buses.items():
                print(f"Connecting to {arm_name}...")
                bus.connect()
                print(f"✓ {arm_name} connected")
            
            # Connect leaders
            for arm_name, bus in self.leader_buses.items():
                print(f"Connecting to {arm_name}...")
                bus.connect()
                print(f"✓ {arm_name} connected")
            
            print("All arms connected successfully!")
            return True
            
        except Exception as e:
            print(f"Error connecting to arms: {e}")
            return False
    
    def disconnect_all(self):
        """Disconnect from all arms"""
        try:
            for bus in self.follower_buses.values():
                if bus.is_connected:
                    bus.disconnect()
            for bus in self.leader_buses.values():
                if bus.is_connected:
                    bus.disconnect()
        except Exception as e:
            print(f"Error disconnecting: {e}")
    
    def get_leader_actions(self):
        """Get actions from all leader arms"""
        actions = {}
        
        for arm_name, bus in self.leader_buses.items():
            try:
                positions = bus.sync_read("Present_Position")
                prefix = arm_name.replace('_leader', '')
                for motor_name, pos in positions.items():
                    # Convert to numpy array with correct dtype
                    actions[f"{prefix}_{motor_name}.pos"] = np.array([pos], dtype=np.float32)
            except Exception as e:
                print(f"Error reading from {arm_name}: {e}")
        
        return actions
    
    def send_follower_actions(self, actions):
        """Send actions to all follower arms"""
        # Group actions by arm
        arm_actions = {}
        for action_key, value in actions.items():
            if 'left_' in action_key:
                if 'left_follower' not in arm_actions:
                    arm_actions['left_follower'] = {}
                motor_name = action_key.replace('left_', '')
                # Extract the float value from numpy array
                arm_actions['left_follower'][motor_name] = float(value[0])
            elif 'right_' in action_key:
                if 'right_follower' not in arm_actions:
                    arm_actions['right_follower'] = {}
                motor_name = action_key.replace('right_', '')
                # Extract the float value from numpy array
                arm_actions['right_follower'][motor_name] = float(value[0])
        
        # Send to each arm
        for arm_name, motor_actions in arm_actions.items():
            if arm_name in self.follower_buses:
                try:
                    self.follower_buses[arm_name].sync_write("Goal_Position", motor_actions)
                except Exception as e:
                    print(f"Error sending to {arm_name}: {e}")
    
    def get_follower_observations(self):
        """Get observations from all follower arms"""
        observations = {}
        
        for arm_name, bus in self.follower_buses.items():
            try:
                positions = bus.sync_read("Present_Position")
                prefix = arm_name.replace('_follower', '')
                for motor_name, pos in positions.items():
                    # Convert to numpy array with correct dtype
                    observations[f"{prefix}_{motor_name}.pos"] = np.array([pos], dtype=np.float32)
            except Exception as e:
                print(f"Error reading from {arm_name}: {e}")
        
        return observations
    
    def record_episode(self, episode_time_s, task_description):
        """Record a single episode"""
        print(f"Recording episode: {task_description}")
        
        start_time = time.perf_counter()
        frame_count = 0
        
        while time.perf_counter() - start_time < episode_time_s:
            try:
                # Get leader actions
                actions = self.get_leader_actions()
                
                # Get follower observations
                observations = self.get_follower_observations()
                
                # Send actions to followers
                self.send_follower_actions(actions)
                
                # Add frame to dataset
                frame = {**observations, **actions}
                self.dataset.add_frame(frame, task=task_description)
                
                frame_count += 1
                
                # Control loop rate
                time.sleep(1.0 / self.fps)
                
            except KeyboardInterrupt:
                print("\nRecording interrupted by user")
                break
            except Exception as e:
                print(f"Error in recording loop: {e}")
                break
        
        print(f"Recorded {frame_count} frames")
        return frame_count > 0
    
    def start_recording(self):
        """Start the recording session"""
        if not self.connect_all():
            print("Failed to connect to all arms")
            return
        
        try:
            # Initialize visualization and keyboard listener
            if self.config.get('display_data', False):
                _init_rerun(session_name="dual_recording")
            
            listener, events = init_keyboard_listener()
            
            num_episodes = self.config['dataset']['num_episodes']
            episode_time_s = self.config['dataset']['episode_time_s']
            task_description = self.config['dataset']['single_task']
            
            recorded_episodes = 0
            
            while recorded_episodes < num_episodes and not events["stop_recording"]:
                log_say(f"Recording episode {recorded_episodes + 1} of {num_episodes}")
                
                # Record episode
                success = self.record_episode(episode_time_s, task_description)
                
                if success:
                    self.dataset.save_episode()
                    recorded_episodes += 1
                
                # Reset time between episodes (fixed at 10 seconds)
                if recorded_episodes < num_episodes and not events["stop_recording"]:
                    log_say("Reset the environment")
                    time.sleep(10)  # Fixed 10 second reset time
                
                # Handle re-recording
                if events["rerecord_episode"]:
                    log_say("Re-recording episode")
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    self.dataset.clear_episode_buffer()
                    continue
            
            # Finalize dataset (no push to hub by default)
            print(f"Recording complete! Recorded {recorded_episodes} episodes.")
            print(f"Dataset saved locally. To push to Hugging Face Hub, edit the script.")
            
        except KeyboardInterrupt:
            print("\nRecording interrupted by user")
        finally:
            self.disconnect_all()
            if 'listener' in locals() and listener is not None:
                listener.stop()

def get_config_from_user():
    """Get configuration from user input"""
    print("=== FLEXIBLE DUAL SO100 RECORDING CONFIGURATION ===")
    print("This script will record training data from dual SO100 arms.")
    print()
    
    config = {
        'arms': {},
        'dataset': {},
        'fps': 30,
        'display_data': False
    }
    
    # Get arm configurations
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
        
        config['arms'][arm_name] = {
            "port": port,
            "motor_ids": motor_ids,
            "id_range": id_range
        }
    
    # Get dataset configuration
    print(f"\n--- DATASET CONFIGURATION ---")
    config['dataset']['repo_id'] = input("Enter dataset repository ID (e.g., username/dataset_name): ").strip()
    config['dataset']['single_task'] = input("Enter task description: ").strip()
    config['dataset']['num_episodes'] = int(input("Enter number of episodes to record: "))
    config['dataset']['episode_time_s'] = int(input("Enter episode duration in seconds: "))
    
    display_data = input("Display data during recording? (y/n): ").strip().lower()
    config['display_data'] = display_data == 'y'
    
    return config

def main():
    parser = argparse.ArgumentParser(description="Flexible Dual SO100 Recording")
    parser.add_argument("--config", help="Path to configuration JSON file")
    parser.add_argument("--interactive", action="store_true", help="Get configuration interactively")
    
    args = parser.parse_args()
    
    if args.config:
        # Load config from file
        with open(args.config, 'r') as f:
            config = json.load(f)
    elif args.interactive:
        # Get config interactively
        config = get_config_from_user()
    else:
        print("Please provide either --config or --interactive")
        return
    
    # Start recording
    recorder = FlexibleDualSO100Recorder(config)
    recorder.start_recording()

if __name__ == "__main__":
    main() 