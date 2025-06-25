from dataclasses import dataclass
import draccus
import rerun as rr
import torch
from enum import Enum
import time
from pprint import pformat
import numpy as np
import os

from examples.so100_robot.utils import display_data
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.policies.pretrained import PreTrainedPolicy
from lerobot.common.robots.so100_robot.config_so100_robot import SO100RobotClientConfig
from lerobot.common.robots.so100_robot.so100_robot_client import SO100RobotClient
from lerobot.common.utils.control_utils import predict_action
from lerobot.common.utils.utils import get_safe_torch_device, init_logging, log_say
from lerobot.common.utils.visualization_utils import _init_rerun
from lerobot.common.utils.robot_utils import busy_wait
from lerobot.common.datasets.utils import build_dataset_frame, hw_to_dataset_features


class PolicyType(Enum):
    ACT = "act"
    SMOLVLA = "smolvla"
    PI0 = "pi0"


@dataclass
class EvaluateConfig:
    # Evaluation parameters
    nb_cycles: int = 9000
    # Control frequency (should match recording FPS)
    fps: int = 30
    # Robot configuration
    robot_ip: str = "192.168.1.208"
    robot_id: str = "so100_robot"
    # Policy configuration
    policy_type: PolicyType = PolicyType.ACT
    policy_name: str = "outputs/train/act_so100_robot_001_tape_a/checkpoints/020000/pretrained_model"
    # Task description for the dataset
    task_description: str = "Grab the tape and put it in the cup"
    # Display configuration
    display_data: bool = False
    rerun_session_name: str = "so100_robot_evaluation"
    # Use vocal synthesis to read events
    play_sounds: bool = True


def load_policy(policy_type: PolicyType, policy_name: str) -> PreTrainedPolicy:
    """Load the specified policy type."""
    # Debug: Check if policy path exists
    if os.path.exists(policy_name):
        print(f"DEBUG: Policy path exists: {policy_name}")

        # Check if it's a directory
        if os.path.isdir(policy_name):
            print(f"DEBUG: Policy path is a directory, checking contents:")
            total_size = 0
            for root, dirs, files in os.walk(policy_name):
                for file in files:
                    file_path = os.path.join(root, file)
                    file_size = os.path.getsize(file_path)
                    total_size += file_size
                    print(f"  {file_path}: {file_size} bytes")
            print(f"DEBUG: Total directory size: {total_size} bytes")

            # Check for expected files
            expected_files = ['config.json', 'pytorch_model.bin']
            for expected_file in expected_files:
                expected_path = os.path.join(policy_name, expected_file)
                if os.path.exists(expected_path):
                    print(f"DEBUG: Found expected file: {expected_file}")
                else:
                    print(f"WARNING: Missing expected file: {expected_file}")
        else:
            # It's a file
            file_size = os.path.getsize(policy_name)
            print(f"DEBUG: Policy path is a file, size: {file_size} bytes")
            if file_size < 1000:
                print("WARNING: Policy file is very small, might be corrupted or empty")
    else:
        print(f"ERROR: Policy path does not exist: {policy_name}")
        raise FileNotFoundError(f"Policy path not found: {policy_name}")

    if policy_type == PolicyType.ACT:
        try:
            policy = ACTPolicy.from_pretrained(policy_name)
            print(f"DEBUG: Loaded ACT policy: {type(policy)}")

            # Additional debugging for ACT policy
            if hasattr(policy, 'model'):
                print(f"DEBUG: Model state keys: {list(policy.model.state_dict().keys())}")
                # Check a few key parameters
                for name, param in list(policy.model.named_parameters())[:5]:
                    print(f"DEBUG: {name} - shape: {param.shape}, mean: {param.mean():.6f}, std: {param.std():.6f}")

            return policy
        except Exception as e:
            print(f"ERROR: Failed to load ACT policy: {e}")
            import traceback
            traceback.print_exc()
            raise
    # elif policy_type == PolicyType.SMOLVLA:
    #     return SmolvlaPolicy.from_pretrained(policy_name)
    # elif policy_type == PolicyType.PI0:
    #     return Pi0Policy.from_pretrained(policy_name)
    else:
        raise ValueError(f"Unsupported policy type: {policy_type}")


def evaluate_loop(
    robot,
    policy: PreTrainedPolicy,
    fps: int,
    control_time_s: int | None = None,
    task_description: str | None = None,
    should_display_data: bool = False,
):
    """Evaluation loop that handles policy inference and robot control."""
    # Policy needs cleaning up
    policy.reset()

    # Debug: Check policy state
    print(f"DEBUG: Policy type: {type(policy)}")
    print(f"DEBUG: Policy config: {policy.config}")
    print(f"DEBUG: Policy device: {policy.config.device}")

    # Debug: Check if policy has been trained
    if hasattr(policy, 'model'):
        print(f"DEBUG: Policy model type: {type(policy.model)}")
        # Check if model has any non-zero parameters
        total_params = 0
        non_zero_params = 0
        for param in policy.model.parameters():
            total_params += param.numel()
            non_zero_params += (param != 0).sum().item()
        print(f"DEBUG: Model parameters - Total: {total_params}, Non-zero: {non_zero_params}")
        if non_zero_params == 0:
            print("WARNING: All model parameters are zero! This suggests the model wasn't trained properly.")

    timestamp = 0
    start_episode_t = time.perf_counter()

    # Build dataset features for policy input (same as record.py)
    obs_features = hw_to_dataset_features(robot.observation_features, "observation", False)

    # Debug: Print observation features
    print(f"DEBUG: Observation features: {obs_features}")

    # Get device
    device = get_safe_torch_device(policy.config.device)
    print(f"DEBUG: Using device: {device}")

    print_timestamp = 0
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        observation = robot.get_observation()

        # Debug: Print raw observation
        print(f"DEBUG: Raw observation keys: {list(observation.keys())}")
        for key, value in observation.items():
            if isinstance(value, np.ndarray):
                print(f"DEBUG: Raw {key} shape: {value.shape}, dtype: {value.dtype}")

        # Build observation frame for policy (same as record.py)
        observation_frame = build_dataset_frame(obs_features, observation, prefix="observation")

        # Debug: Print observation frame
        print(f"DEBUG: Observation frame keys: {list(observation_frame.keys())}")

        # Get action from policy (same as record.py)
        action_values = predict_action(
            observation_frame,
            policy,
            device,
            False,
            task=task_description,
            robot_type=robot.robot_type,
        )

        # Debug: Print action values
        print(f"DEBUG: Action values shape: {action_values.shape}")
        print(f"DEBUG: Action values: {action_values}")

        action = {key: action_values[i].item() for i, key in enumerate(robot.action_features)}

        # Debug: Print final action
        print(f"DEBUG: Final action: {action}")

        # Send action to robot (same as record.py)
        robot.send_action(action)

        # Display data in Rerun (same as record.py)
        if should_display_data:
            display_data(observation, action)

        # Maintain timing (same as record.py)
        dt_s = time.perf_counter() - start_loop_t
        busy_wait(1 / fps - dt_s)

        timestamp = time.perf_counter() - start_episode_t


@draccus.wrap()
def evaluate(cfg: EvaluateConfig):
    if cfg.display_data:
        _init_rerun(session_name=cfg.rerun_session_name)

    # Initialize robot
    robot_config = SO100RobotClientConfig(
        remote_ip=cfg.robot_ip,
        id=cfg.robot_id
    )
    robot = SO100RobotClient(robot_config)

    # Load policy based on type
    print(f"Loading {cfg.policy_type.value} policy: {cfg.policy_name}")
    policy = load_policy(cfg.policy_type, cfg.policy_name)

    # Connect to robot
    robot.connect()

    # Check connection status
    if not robot.is_connected:
        print("Failed to connect to robot")
        return

    print("Sourccey V2 Beta Evaluation initialized")
    print(f"Robot IP: {cfg.robot_ip}")
    print(f"Policy Type: {cfg.policy_type.value}")
    print(f"Policy: {cfg.policy_name}")
    print(f"Cycles: {cfg.nb_cycles}")
    print(f"FPS: {cfg.fps}")
    print(f"Display Data: {cfg.display_data}")
    print("Press Ctrl+C to stop evaluation")

    try:
        # Calculate control time based on nb_cycles and fps (same as record.py)
        control_time_s = cfg.nb_cycles / cfg.fps

        log_say(f"Starting evaluation for {control_time_s:.1f} seconds", cfg.play_sounds)

        evaluate_loop(
            robot=robot,
            policy=policy,
            fps=cfg.fps,
            control_time_s=control_time_s,
            task_description=cfg.task_description,
            should_display_data=cfg.display_data,
        )

    except KeyboardInterrupt:
        print("\nEvaluation stopped by user")
    finally:
        print("Cleaning up...")
        rr.rerun_shutdown()
        robot.disconnect()
        log_say("Evaluation ended", cfg.play_sounds)
        print("Evaluation ended")


if __name__ == "__main__":
    evaluate()
