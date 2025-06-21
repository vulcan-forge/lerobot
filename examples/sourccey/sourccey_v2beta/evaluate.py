import torch

from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.utils.control_utils import predict_action
from lerobot.common.utils.utils import get_safe_torch_device

NB_CYCLES_CLIENT_CONNECTION = 1000

robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="sourccey_v2beta")
robot = SourcceyV2BetaClient(robot_config)

robot.connect()

policy = ACTPolicy.from_pretrained("pepijn223/act_sourccey_v2beta_circle")
policy.reset()

print("Running inference")
i = 0
while i < NB_CYCLES_CLIENT_CONNECTION:
    obs = robot.get_observation()

    for key, value in obs.items():
        if isinstance(value, torch.Tensor):
            obs[key] = value.numpy()

    action_values = predict_action(
        obs, policy, get_safe_torch_device(policy.config.device), policy.config.use_amp
    )
    action = {
        key: action_values[i].item() if isinstance(action_values[i], torch.Tensor) else action_values[i]
        for i, key in enumerate(robot.action_features)
    }
    robot.send_action(action)
    i += 1

robot.disconnect()
