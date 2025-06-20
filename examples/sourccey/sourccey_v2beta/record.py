import time
from dataclasses import dataclass
from pprint import pformat
import draccus
import rerun as rr

from examples.sourccey.sourccey_v2beta.utils import display_data
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.utils import hw_to_dataset_features
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_leader.config_sourccey_v2beta_leader import SourcceyV2BetaLeaderConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_leader.sourccey_v2beta_leader import SourcceyV2BetaLeader
from lerobot.common.utils.utils import init_logging, log_say
from lerobot.common.utils.visualization_utils import _init_rerun
from lerobot.common.utils.control_utils import init_keyboard_listener, is_headless
from lerobot.common.utils.robot_utils import busy_wait


@dataclass
class RecordConfig:
    # Number of episodes to record
    num_episodes: int = 1
    # Number of cycles per episode
    nb_cycles: int = 750
    # Dataset repository ID (will append timestamp if not provided)
    repo_id: str = "user/sourccey_v2beta"
    # Recording FPS
    fps: int = 30
    # Warm up and reset time
    reset_time_s: int | float = 10
    warmup_time_s: int | float = 5
    # Task description for the dataset
    task_description: str = "Grab the towel and fold it"
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
    # Use vocal synthesis to read events
    play_sounds: bool = True


def record_loop(
    robot,
    leader_arm,
    keyboard,
    events: dict,
    fps: int,
    dataset: LeRobotDataset | None = None,
    task_description: str | None = None,
    display_data: bool = False,
    control_time_s: int | None = None,
):
    """Record loop that handles keyboard events and data collection."""
    timestamp = 0
    start_episode_t = time.perf_counter()

    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        # Check for keyboard events
        if events["exit_early"]:
            events["exit_early"] = False
            break

        observation = robot.get_observation()

        arm_action = leader_arm.get_action()
        arm_action = {k: v for k, v in arm_action.items() if k.startswith(("left_arm", "right_arm"))}

        keyboard_keys = keyboard.get_action()
        base_action = robot._from_keyboard_to_base_action(keyboard_keys)

        # Display all data in Rerun
        if display_data:
            display_data(observation, arm_action, base_action)

        action = arm_action | base_action if len(base_action) > 0 else arm_action
        action_sent = robot.send_action(action)

        # Create frame and add to dataset only if dataset is provided
        if dataset is not None and task_description is not None:
            frame = {**action_sent, **observation}
            dataset.add_frame(frame, task_description)

        # Maintain timing
        dt_s = time.perf_counter() - start_loop_t
        busy_wait(1 / fps - dt_s)

        timestamp = time.perf_counter() - start_episode_t


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

    # Initialize keyboard listener
    listener, events = init_keyboard_listener()

    print(f"Starting SourcceyV2Beta recording for {cfg.num_episodes} episodes")
    print(f"Dataset will be saved to: {repo_id}")
    print("Keyboard controls:")
    print("  Right arrow: Save current episode and continue")
    print("  Left arrow: Re-record current episode")
    print("  Escape: Stop recording entirely")
    print("  Ctrl+C: Emergency stop")

    try:
        # Calculate control time based on nb_cycles and fps
        control_time_s = cfg.nb_cycles / cfg.fps

        # Warm-up period before first episode
        if cfg.warmup_time_s > 0:
            log_say(f"Warming up for {cfg.warmup_time_s} seconds...", cfg.play_sounds)
            record_loop(
                robot=robot,
                leader_arm=leader_arm,
                keyboard=keyboard,
                events=events,
                fps=cfg.fps,
                dataset=None,  # No dataset during warm-up
                task_description=None,
                display_data=cfg.display_data,
                control_time_s=cfg.warmup_time_s,
            )

        recorded_episodes = 0
        while recorded_episodes < cfg.num_episodes:
            # Audio feedback for episode start (using dataset.num_episodes like main record)
            log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
            print(f"\nRecording episode {recorded_episodes + 1}/{cfg.num_episodes}")

            # Reset events for new episode
            events["exit_early"] = False
            events["rerecord_episode"] = False

            record_loop(
                robot=robot,
                leader_arm=leader_arm,
                keyboard=keyboard,
                events=events,
                fps=cfg.fps,
                dataset=dataset,
                task_description=cfg.task_description,
                display_data=cfg.display_data,
                control_time_s=control_time_s,
            )

            # Handle re-record episode event
            if events["rerecord_episode"]:
                log_say("Re-record episode", cfg.play_sounds)
                print("Re-recording episode...")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue  # Re-run the for loop for this episode

            # Save the episode
            dataset.save_episode()
            print(f"Episode {recorded_episodes + 1} saved")

            # Check if we should stop recording
            if events["stop_recording"]:
                recorded_episodes += 1
                break

            # Execute reset time without recording to give time to manually reset the environment
            # Skip reset for the last episode to be recorded
            if not events["stop_recording"] and recorded_episodes < cfg.num_episodes - 1:
                log_say("Reset the environment", cfg.play_sounds)
                record_loop(
                    robot=robot,
                    leader_arm=leader_arm,
                    keyboard=keyboard,
                    events=events,
                    fps=cfg.fps,
                    dataset=None,  # No dataset during reset time
                    task_description=None,
                    display_data=cfg.display_data,
                    control_time_s=cfg.reset_time_s,
                )

                if events["stop_recording"]:
                    recorded_episodes += 1
                    break

            recorded_episodes += 1

    except KeyboardInterrupt:
        print("\nRecording interrupted by user")
    finally:
        # Cleanup connections
        print("Disconnecting devices...")
        robot.disconnect()
        leader_arm.disconnect()
        keyboard.disconnect()

        # Cleanup keyboard listener
        if not is_headless() and listener is not None:
            listener.stop()

        # Save and upload dataset
        print("Saving dataset...")
        dataset.save_episode()

        log_say("Stop recording", cfg.play_sounds, blocking=True)
        log_say("Exiting", cfg.play_sounds)

        # Todo: 6/20/2025: Will push to hub when proper data structure is implemented
        # dataset.push_to_hub()
        # print(f"Dataset successfully uploaded to: {repo_id}")


if __name__ == "__main__":
    record()
