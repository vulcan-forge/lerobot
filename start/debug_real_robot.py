#!/usr/bin/env python3
"""
Debug script to test with real robot data in the evaluate loop.
"""

import sys
import os
import numpy as np
import torch
import time
from pathlib import Path

# Add the project root to the path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.utils.control_utils import predict_action
from lerobot.common.utils.utils import get_safe_torch_device
from lerobot.common.datasets.utils import build_dataset_frame, hw_to_dataset_features
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient

def test_real_robot_loop():
    """Test the evaluate loop with real robot data."""
    print("🤖 Testing with real robot data...")

    # Create robot config and robot (same as evaluate.py)
    robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="sourccey_v2beta")
    robot = SourcceyV2BetaClient(robot_config)

    # Connect to robot
    print("🔌 Connecting to robot...")
    try:
        robot.connect()
        if not robot.is_connected:
            print("❌ Failed to connect to robot")
            return
        print("✅ Connected to robot successfully!")
    except Exception as e:
        print(f"❌ Error connecting to robot: {e}")
        return

    # Load policy
    policy_path = "outputs/train/act_sourccey_v2beta_001_tape_a/checkpoints/020000/pretrained_model"
    if not os.path.exists(policy_path):
        print(f"❌ Policy not found at {policy_path}")
        return

    print(f"📂 Loading policy from {policy_path}")
    policy = ACTPolicy.from_pretrained(policy_path)

    # Get device
    device = get_safe_torch_device("cuda")
    print(f"🖥️ Using device: {device}")

    # Build dataset features (same as evaluate.py)
    obs_features = hw_to_dataset_features(robot.observation_features, "observation", False)

    # Test the evaluate loop with real robot data
    print("\n🔄 Starting evaluate loop with real robot data...")
    timestamp = 0
    start_episode_t = time.perf_counter()
    fps = 30
    control_time_s = 10  # Test for 10 seconds

    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        try:
            # Get real observation from robot (same as evaluate.py)
            observation = robot.get_observation()

            # Check for NaN in real data
            has_nan_in_data = False
            for key, value in observation.items():
                if isinstance(value, np.ndarray):
                    if np.isnan(value).any():
                        print(f"❌ NaN found in robot data: {key}")
                        has_nan_in_data = True
                        break

            if has_nan_in_data:
                print("❌ Stopping due to NaN in robot data")
                break

            # Build observation frame (same as evaluate.py)
            observation_frame = build_dataset_frame(obs_features, observation, prefix="observation")

            # Get action from policy (same as evaluate.py)
            action_values = predict_action(
                observation_frame,
                policy,
                device,
                False,
                task="Grab the tape and put it in the cup",
                robot_type="sourccey_v2beta"
            )

            # Check for NaN in action
            if torch.isnan(action_values).any():
                print(f"❌ NaN detected in action at timestamp {timestamp:.2f}s!")
                print(f"Action values: {action_values}")
                print(f"Observation state: {observation.get('observation.state', 'N/A')}")
                break
            else:
                print(f"✅ Timestamp {timestamp:.2f}s: OK (action range: [{action_values.min().item():.3f}, {action_values.max().item():.3f}])")

        except Exception as e:
            print(f"❌ Error at timestamp {timestamp:.2f}s: {e}")
            import traceback
            traceback.print_exc()
            break

        # Maintain timing (same as evaluate.py)
        dt_s = time.perf_counter() - start_loop_t
        sleep_time = max(0, 1 / fps - dt_s)
        if sleep_time > 0:
            time.sleep(sleep_time)

        timestamp = time.perf_counter() - start_episode_t

    print(f"\n🏁 Loop completed. Total iterations: {int(timestamp * fps)}")

    # Disconnect robot
    try:
        robot.disconnect()
        print("🔌 Disconnected from robot")
    except Exception as e:
        print(f"⚠️ Error disconnecting: {e}")

if __name__ == "__main__":
    test_real_robot_loop()
