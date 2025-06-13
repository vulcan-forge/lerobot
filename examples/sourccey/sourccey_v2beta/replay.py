import time

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.utils.robot_utils import busy_wait

robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="sourccey_v2beta")
robot = SourcceyV2BetaClient(robot_config)

dataset = LeRobotDataset("pepijn223/sourccey_v2beta1749025613", episodes=[0])

robot.connect()

print("Replaying episode…")
for _, action_array in enumerate(dataset.hf_dataset["action"]):
    t0 = time.perf_counter()

    action = {name: float(action_array[i]) for i, name in enumerate(dataset.features["action"]["names"])}
    robot.send_action(action)

    busy_wait(max(1.0 / dataset.fps - (time.perf_counter() - t0), 0.0))

print("Disconnecting SourcceyV2Beta Client")
robot.disconnect()
