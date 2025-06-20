from dataclasses import asdict, dataclass
from pprint import pformat
import draccus
import rerun as rr
import time

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.utils.robot_utils import busy_wait
from lerobot.common.utils.utils import init_logging
from lerobot.common.utils.visualization_utils import _init_rerun
from examples.sourccey.sourccey_v2beta.utils import display_data

@dataclass
class ReplayConfig:
    # Dataset configuration
    dataset_path: str = "user/sourccey_v2beta1749025613"
    episode: int = 0
    # Robot configuration
    robot_ip: str = "192.168.1.191"
    robot_id: str = "sourccey_v2beta"
    # Display configuration
    display_data: bool = False
    rerun_session_name: str = "sourccey_v2beta_replay"


@draccus.wrap()
def replay(cfg: ReplayConfig):
    if cfg.display_data:
        _init_rerun(session_name=cfg.rerun_session_name)

    # Initialize robot
    robot_config = SourcceyV2BetaClientConfig(
        remote_ip=cfg.robot_ip,
        id=cfg.robot_id
    )
    robot = SourcceyV2BetaClient(robot_config)

    # Load dataset
    dataset = LeRobotDataset(cfg.dataset_path, episodes=[cfg.episode])

    # Connect to robot
    robot.connect()

    # Check connection status
    if not robot.is_connected:
        print("Failed to connect to robot")
        return

    print("Sourccey V2 Beta Replay initialized")
    print(f"Dataset: {cfg.dataset_path}")
    print(f"Episode: {cfg.episode}")
    print(f"Robot IP: {cfg.robot_ip}")
    print(f"Display Data: {cfg.display_data}")
    print("Press Ctrl+C to stop replay")

    try:
        print("Replaying episode...")
        for i, action_array in enumerate(dataset.hf_dataset["action"]):
            t0 = time.perf_counter()

            action = {name: float(action_array[i]) for i, name in enumerate(dataset.features["action"]["names"])}

            # Display data in Rerun if enabled
            if cfg.display_data:
                # Get current observation for display (if available)
                observation = robot.get_observation()
                display_data(observation, action, {})

            robot.send_action(action)

            # Maintain timing
            busy_wait(max(1.0 / dataset.fps - (time.perf_counter() - t0), 0.0))

    except KeyboardInterrupt:
        print("\nReplay stopped by user")
    finally:
        print("Cleaning up...")
        rr.rerun_shutdown()
        robot.disconnect()
        print("Replay ended")


if __name__ == "__main__":
    replay()
