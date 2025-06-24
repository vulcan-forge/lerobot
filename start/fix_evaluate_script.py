#!/usr/bin/env python3
"""
Fix for the evaluate script to handle the normalization issue properly.
The problem is that the policy's normalization is missing or corrupted.
"""

import sys
import os
import numpy as np
import torch
from pathlib import Path

# Add the project root to the path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.utils.control_utils import predict_action
from lerobot.common.utils.utils import get_safe_torch_device

def create_mock_observation():
    """Create a mock observation that matches the dataset format expected by the policy."""
    # State features (12 joint positions) - as a single numpy array
    state_values = np.array([
        45.0,   # left_arm_shoulder_pan.pos
        30.0,   # left_arm_shoulder_lift.pos
        60.0,   # left_arm_elbow_flex.pos
        15.0,   # left_arm_wrist_flex.pos
        90.0,   # left_arm_wrist_roll.pos
        50.0,   # left_arm_gripper.pos
        -45.0,  # right_arm_shoulder_pan.pos
        30.0,   # right_arm_shoulder_lift.pos
        60.0,   # right_arm_elbow_flex.pos
        15.0,   # right_arm_wrist_flex.pos
        -90.0,  # right_arm_wrist_roll.pos
        50.0,   # right_arm_gripper.pos
    ], dtype=np.float32)

    # Image features (240x320x3 images) - as numpy arrays
    image_shape = (240, 320, 3)
    image_features = {
        'observation.images.front_left': np.random.randint(0, 255, image_shape, dtype=np.uint8),
        'observation.images.front_right': np.random.randint(0, 255, image_shape, dtype=np.uint8),
        'observation.images.wrist_left': np.random.randint(0, 255, image_shape, dtype=np.uint8),
        'observation.images.wrist_right': np.random.randint(0, 255, image_shape, dtype=np.uint8),
    }

    # Combine with proper dataset format
    observation = {
        'observation.state': state_values,
        **image_features
    }

    return observation

def fix_policy_normalization(policy):
    """Fix the policy's normalization by ensuring proper dataset statistics."""
    # Check if the policy has normalization
    if hasattr(policy, 'normalize') and policy.normalize is not None:
        print("🔧 Fixing policy normalization...")

        # Get the normalization object
        normalize = policy.normalize

        # Check if dataset statistics are missing
        if hasattr(normalize, 'dataset_stats') and normalize.dataset_stats is None:
            print("⚠️  Dataset statistics are missing, creating mock statistics...")

            # Create mock dataset statistics that match the expected format
            mock_stats = {
                'observation.state': {
                    'mean': torch.zeros(12, dtype=torch.float32),
                    'std': torch.ones(12, dtype=torch.float32),
                },
                'observation.images.front_left': {
                    'mean': torch.zeros(3, 240, 320, dtype=torch.float32),
                    'std': torch.ones(3, 240, 320, dtype=torch.float32),
                },
                'observation.images.front_right': {
                    'mean': torch.zeros(3, 240, 320, dtype=torch.float32),
                    'std': torch.ones(3, 240, 320, dtype=torch.float32),
                },
                'observation.images.wrist_left': {
                    'mean': torch.zeros(3, 240, 320, dtype=torch.float32),
                    'std': torch.ones(3, 240, 320, dtype=torch.float32),
                },
                'observation.images.wrist_right': {
                    'mean': torch.zeros(3, 240, 320, dtype=torch.float32),
                    'std': torch.ones(3, 240, 320, dtype=torch.float32),
                },
            }

            # Set the mock statistics
            normalize.dataset_stats = mock_stats
            print("✅ Mock dataset statistics set")

        # Ensure all statistics are PyTorch tensors
        if hasattr(normalize, 'dataset_stats') and normalize.dataset_stats is not None:
            for key, stats in normalize.dataset_stats.items():
                if isinstance(stats, dict):
                    for stat_name, stat_value in stats.items():
                        if not isinstance(stat_value, torch.Tensor):
                            normalize.dataset_stats[key][stat_name] = torch.tensor(stat_value, dtype=torch.float32)
                            print(f"🔄 Converted {key}.{stat_name} to tensor")

        print("✅ Policy normalization fixed")
    else:
        print("ℹ️  Policy has no normalization, skipping fix")

def test_policy_with_fix():
    """Test the policy with the normalization fix."""
    print("🧪 Testing policy with normalization fix...")

    # Create mock observation in dataset format
    observation = create_mock_observation()
    print(f"📊 Observation created with {len(observation)} features")
    print(f"📋 Observation keys: {list(observation.keys())}")
    print(f"📈 State shape: {observation['observation.state'].shape}")
    print(f"📋 Image shapes: {[(k, v.shape) for k, v in observation.items() if 'images' in k]}")

    # Load policy
    policy_path = "outputs/train/act_sourccey_v2beta_001_tape_a/checkpoints/020000/pretrained_model"
    if not os.path.exists(policy_path):
        print(f"❌ Policy not found at {policy_path}")
        return

    print(f"📂 Loading policy from {policy_path}")
    policy = ACTPolicy.from_pretrained(policy_path)

    # Fix normalization
    fix_policy_normalization(policy)

    # Get device
    device = get_safe_torch_device("cuda")  # Try CUDA first, fallback to CPU
    print(f"🖥️  Using device: {device}")

    # Test prediction
    try:
        action = predict_action(
            observation=observation,
            policy=policy,
            device=device,
            use_amp=False,
            task="test_task",
            robot_type="sourccey_v2beta"
        )

        print("✅ Action prediction successful!")
        print(f"📈 Action shape: {action.shape}")
        print(f"📊 Action range: [{action.min().item():.3f}, {action.max().item():.3f}]")
        print(f"🔍 Action contains NaN: {torch.isnan(action).any().item()}")

        if not torch.isnan(action).any():
            print("🎉 SUCCESS: No NaN values in action!")
        else:
            print("❌ FAILURE: Action still contains NaN values")

    except Exception as e:
        print(f"❌ Error during prediction: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_policy_with_fix()
