#!/usr/bin/env python3
"""
Simple Policy Test Script

This script tests a single policy with the correct input format.
"""

import torch
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.utils.utils import get_safe_torch_device


def test_policy(policy_path: str):
    """Test a single policy with correct input format"""
    print(f"Testing policy: {policy_path}")

    # Load the policy
    try:
        policy = ACTPolicy.from_pretrained(policy_path)
        print(f"✓ Successfully loaded policy")
    except Exception as e:
        print(f"✗ Failed to load policy: {e}")
        return False

    # Get device
    device = get_safe_torch_device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Move policy to device
    policy.to(device)
    policy.eval()

    # Get the camera key from the policy config
    image_keys = list(policy.config.input_features.keys())
    camera_key = None
    for key in image_keys:
        if key.startswith('observation.images.'):
            camera_key = key
            break

    if not camera_key:
        print(f"✗ No camera key found in config")
        return False

    print(f"Using camera key: {camera_key}")

    # Create dummy observation with the correct format
    dummy_observation = {
        'observation.state': torch.randn(1, 6, device=device),  # 6 DOF robot state
        camera_key: torch.rand(1, 3, 480, 640, device=device),  # RGB image
        'action': torch.randn(1, 100, 6, device=device),  # Dummy action for VAE
        'action_is_pad': torch.zeros(1, 100, dtype=torch.bool, device=device),  # Action padding mask
    }

    print(f"Observation keys: {list(dummy_observation.keys())}")
    print(f"State shape: {dummy_observation['observation.state'].shape}")
    print(f"Image shape: {dummy_observation[camera_key].shape}")

    # Test the policy
    try:
        with torch.no_grad():
            # Get action prediction
            action = policy.select_action(dummy_observation)

            print()
            print(f"✓ Successfully predicted action!")
            print(f"Action shape: {action.shape}")
            print(f"Action mean: {action.mean().item():.6f}")
            print(f"Action std: {action.std().item():.6f}")
            print(f"Action range: [{action.min().item():.6f}, {action.max().item():.6f}]")
            print(f"Action: {action}")
            print()

            # Check if action is all zeros (indicates failed policy)
            if torch.allclose(action, torch.zeros_like(action), atol=1e-6):
                print(f"✗ FAIL: Action is all zeros - policy is not working properly")
                return False
            else:
                print(f"✓ PASS: Action has non-zero values - policy is working")
                return True

    except Exception as e:
        print(f"✗ Error during prediction: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    # Test the "broken" policy (which should work with main camera)
    policy_path = "outputs/train/act_so100_robot_001_tape_a/checkpoints/020000/pretrained_model"
    # policy_path = "outputs/train/act_so100-001-test/checkpoints/040000/pretrained_model"

    success = test_policy(policy_path)

    if success:
        print(f"\n✓ Policy test passed!")
    else:
        print(f"\n⚠️  Policy test failed.")
