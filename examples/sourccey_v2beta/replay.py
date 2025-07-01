import time

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.robots.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.utils.robot_utils import busy_wait
from lerobot.common.utils.utils import log_say

EPISODE_IDX = 0

robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="sourccey_v2beta")
robot = SourcceyV2BetaClient(robot_config)

dataset = LeRobotDataset("lerobot/sourccey_v2beta", episodes=[EPISODE_IDX])
actions = dataset.hf_dataset.select_columns("action")

robot.connect()

if not robot.is_connected:
    raise ValueError("Robot is not connected!")

log_say(f"Replaying episode {EPISODE_IDX}")
for idx in range(dataset.num_frames):
    t0 = time.perf_counter()

    action = {
        name: float(actions[idx]["action"][i]) for i, name in enumerate(dataset.features["action"]["names"])
    }
    robot.send_action(action)

    busy_wait(max(1.0 / dataset.fps - (time.perf_counter() - t0), 0.0))

robot.disconnect()
