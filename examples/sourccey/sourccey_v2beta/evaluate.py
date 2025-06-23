from dataclasses import dataclass
import draccus
import rerun as rr
import torch
from enum import Enum

from examples.sourccey.sourccey_v2beta.utils import display_data
from lerobot.common.policies.act.modeling_act import ACTPolicy
# from lerobot.common.policies.smolvla.modeling_smolvla import SmolvlaPolicy
# from lerobot.common.policies.pi0.modeling_pi0 import Pi0Policy
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.utils.control_utils import predict_action
from lerobot.common.utils.utils import get_safe_torch_device, init_logging
from lerobot.common.utils.visualization_utils import _init_rerun


class PolicyType(Enum):
    ACT = "act"
    SMOLVLA = "smolvla"
    PI0 = "pi0"


@dataclass
class EvaluateConfig:
    # Evaluation parameters
    nb_cycles: int = 9000
    # Robot configuration
    robot_ip: str = "192.168.1.191"
    robot_id: str = "sourccey_v2beta"
    # Policy configuration
    policy_type: PolicyType = PolicyType.ACT
    policy_name: str = "outputs/train/act_sourccey_v2beta_towel_010_a"
    # Display configuration
    display_data: bool = False
    rerun_session_name: str = "sourccey_v2beta_evaluation"


def load_policy(policy_type: PolicyType, policy_name: str):
    """Load the specified policy type."""
    if policy_type == PolicyType.ACT:
        return ACTPolicy.from_pretrained(policy_name)
    # elif policy_type == PolicyType.SMOLVLA:
    #     return SmolvlaPolicy.from_pretrained(policy_name)
    # elif policy_type == PolicyType.PI0:
    #     return Pi0Policy.from_pretrained(policy_name)
    else:
        raise ValueError(f"Unsupported policy type: {policy_type}")


@draccus.wrap()
def evaluate(cfg: EvaluateConfig):
    if cfg.display_data:
        _init_rerun(session_name=cfg.rerun_session_name)

    # Initialize robot and policy
    robot_config = SourcceyV2BetaClientConfig(
        remote_ip=cfg.robot_ip,
        id=cfg.robot_id
    )

    robot = SourcceyV2BetaClient(robot_config)

    # Load policy based on type
    print(f"Loading {cfg.policy_type.value} policy: {cfg.policy_name}")
    policy = load_policy(cfg.policy_type, cfg.policy_name)
    policy.reset()

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
    print(f"Display Data: {cfg.display_data}")
    print("Press Ctrl+C to stop evaluation")

    try:
        i = 0
        while i < cfg.nb_cycles:
            observation = robot.get_observation()

            # Convert torch tensors to numpy for policy input
            for key, value in observation.items():
                if isinstance(value, torch.Tensor):
                    observation[key] = value.numpy()

            # Get action from policy
            action_values = predict_action(
                observation,
                policy,
                get_safe_torch_device(policy.config.device),
                policy.config.use_amp
            )

            # Format action for robot
            action = {
                key: action_values[i].item() if isinstance(action_values[i], torch.Tensor) else action_values[i]
                for i, key in enumerate(robot.action_features)
            }

            # Display data in Rerun if enabled
            if cfg.display_data:
                display_data(observation, action, {})

            # Send action to robot
            robot.send_action(action)
            i += 1

    except KeyboardInterrupt:
        print("\nEvaluation stopped by user")
    finally:
        print("Cleaning up...")
        rr.rerun_shutdown()
        robot.disconnect()
        print("Evaluation ended")


if __name__ == "__main__":
    evaluate()
