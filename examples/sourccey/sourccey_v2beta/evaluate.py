from dataclasses import dataclass
import draccus
import rerun as rr
import torch
from enum import Enum
import time
from pprint import pformat
import numpy as np

from examples.sourccey.sourccey_v2beta.utils import display_data
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.policies.pretrained import PreTrainedPolicy
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
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
    robot_ip: str = "192.168.1.191"
    robot_id: str = "sourccey_v2beta"
    # Policy configuration
    policy_type: PolicyType = PolicyType.ACT
    policy_name: str = "outputs/train/act_sourccey_v2beta_001_tape_a/checkpoints/020000/pretrained_model"
    # Task description for the dataset
    task_description: str = "Grab the tape and put it in the cup"
    # Display configuration
    display_data: bool = False
    rerun_session_name: str = "sourccey_v2beta_evaluation"
    # Use vocal synthesis to read events
    play_sounds: bool = True


def load_policy(policy_type: PolicyType, policy_name: str) -> PreTrainedPolicy:
    """Load the specified policy type."""
    if policy_type == PolicyType.ACT:
        return ACTPolicy.from_pretrained(policy_name)
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
    display_data: bool = False,
):
    """Evaluation loop that handles policy inference and robot control."""
    # Policy needs cleaning up
    policy.reset()

    timestamp = 0
    start_episode_t = time.perf_counter()

    # Build dataset features for policy input (same as record.py)
    obs_features = hw_to_dataset_features(robot.observation_features, "observation", False)

    # Get device
    device = get_safe_torch_device("cuda") #get_safe_torch_device(policy.config.device)

    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        observation = robot.get_observation()

        # Build observation frame for policy (same as record.py)
        observation_frame = build_dataset_frame(obs_features, observation, prefix="observation")

        # Get action from policy (same as record.py)
        action_values = predict_action(
            observation_frame,
            policy,
            device,
            False if timestamp == 0 else True,
            task=task_description,
            robot_type=robot.robot_type,
        )
        action = {key: action_values[i].item() for i, key in enumerate(robot.action_features)}

        # Send action to robot (same as record.py)
        robot.send_action(action)

        # Display data in Rerun (same as record.py)
        if display_data:
            for obs, val in observation.items():
                if isinstance(val, float):
                    rr.log(f"observation.{obs}", rr.Scalars(val))
                elif isinstance(val, np.ndarray):
                    rr.log(f"observation.{obs}", rr.Image(val), static=True)
            for act, val in action.items():
                if isinstance(val, float):
                    rr.log(f"action.{act}", rr.Scalars(val))

        # Maintain timing (same as record.py)
        dt_s = time.perf_counter() - start_loop_t
        busy_wait(1 / fps - dt_s)

        timestamp = time.perf_counter() - start_episode_t


@draccus.wrap()
def evaluate(cfg: EvaluateConfig):
    if cfg.display_data:
        _init_rerun(session_name=cfg.rerun_session_name)

    # Initialize robot
    robot_config = SourcceyV2BetaClientConfig(
        remote_ip=cfg.robot_ip,
        id=cfg.robot_id
    )
    robot = SourcceyV2BetaClient(robot_config)

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
            display_data=cfg.display_data,
            task_description=cfg.task_description,
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
