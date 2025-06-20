import time
from dataclasses import dataclass
from pprint import pformat
import draccus

from examples.sourccey.sourccey_v2beta.utils import display_data
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.utils import hw_to_dataset_features
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_leader.config_sourccey_v2beta_leader import SourcceyV2BetaLeaderConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_leader.sourccey_v2beta_leader import SourcceyV2BetaLeader
from lerobot.common.utils.utils import init_logging
from lerobot.common.utils.visualization_utils import _init_rerun


@dataclass
class RecordConfig:
    # Number of cycles to record
    nb_cycles: int = 750
    # Dataset repository ID (will append timestamp if not provided)
    repo_id: str = "user/sourccey_v2beta"
    # Recording FPS
    fps: int = 30
    # Task description for the dataset
    task_description: str = "Dummy Example Task Dataset"
    # Robot configuration
    robot_ip: str = "192.168.1.191"
    robot_id: str = "sourccey_v2beta_2"
    # Leader arm configuration
    leader_arm_port: str = "COM29"
    leader_arm_id: str = "my_sourccey_v2beta_teleop_2"
    # Keyboard configuration
    keyboard_id: str = "my_laptop_keyboard"
    # Rerun session
    display_data: bool = False
    rerun_session_name: str = "sourccey_v2beta_teleoperation"


@draccus.wrap()
def record(cfg: RecordConfig):
    if cfg.display_data:
        _init_rerun(session_name=cfg.rerun_session_name)

    # Initialize robot and teleop devices
    robot_config = SourcceyV2BetaClientConfig(
        remote_ip=cfg.robot_ip,
        id=cfg.robot_id
    )
    leader_arm_config = SourcceyV2BetaLeaderConfig(
        port=cfg.leader_arm_port,
        id=cfg.leader_arm_id
    )
    keyboard_config = KeyboardTeleopConfig(id=cfg.keyboard_id)

    robot = SourcceyV2BetaClient(robot_config)
    leader_arm = SourcceyV2BetaLeader(leader_arm_config)
    keyboard = KeyboardTeleop(keyboard_config)

    # Connect to all devices
    robot.connect()
    leader_arm.connect()
    keyboard.connect()

    # Check connection status
    if not all([robot.is_connected, leader_arm.is_connected, keyboard.is_connected]):
        print("Failed to connect to one or more devices:")
        print(f"  Robot: {robot.is_connected}")
        print(f"  Leader Arm: {leader_arm.is_connected}")
        print(f"  Keyboard: {keyboard.is_connected}")
        return

    # Setup dataset
    action_features = hw_to_dataset_features(robot.action_features, "action")
    obs_features = hw_to_dataset_features(robot.observation_features, "observation")
    dataset_features = {**action_features, **obs_features}

    # Create dataset with timestamp if repo_id doesn't already have one
    repo_id = cfg.repo_id
    if not repo_id.endswith(str(int(time.time()))):
        repo_id = f"{cfg.repo_id}_{int(time.time())}"

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=cfg.fps,
        features=dataset_features,
        robot_type=robot.name,
    )

    print(f"Starting SourcceyV2Beta recording for {cfg.nb_cycles} cycles")
    print(f"Dataset will be saved to: {repo_id}")

    try:
        for i in range(cfg.nb_cycles):
            observation = robot.get_observation()

            arm_action = leader_arm.get_action()
            arm_action = {k: v for k, v in arm_action.items() if k.startswith(("left_arm", "right_arm"))}

            keyboard_keys = keyboard.get_action()
            base_action = robot._from_keyboard_to_base_action(keyboard_keys)

            # Display all data in Rerun
            if cfg.display_data:
                display_data(observation, arm_action, base_action)

            action = arm_action | base_action if len(base_action) > 0 else arm_action
            action_sent = robot.send_action(action)

            # Create frame and add to dataset
            frame = {**action_sent, **observation}
            dataset.add_frame(frame, cfg.task_description)

            # Progress indicator
            if (i + 1) % (cfg.fps * 5) == 0:
                print(f"Recorded {i + 1}/{cfg.nb_cycles} frames")

    except KeyboardInterrupt:
        print("\nRecording interrupted by user")
    finally:
        # Cleanup connections
        print("Disconnecting devices...")
        robot.disconnect()
        leader_arm.disconnect()
        keyboard.disconnect()

        # Save and upload dataset
        print("Saving and uploading dataset to the hub...")
        dataset.save_episode()

        # Todo: 6/20/2025: Will push to hub when proper data structure is implemented
        # dataset.push_to_hub()
        # print(f"Dataset successfully uploaded to: {repo_id}")


if __name__ == "__main__":
    record()
