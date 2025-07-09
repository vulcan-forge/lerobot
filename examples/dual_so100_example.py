#!/usr/bin/env python3
"""
Example script demonstrating the use of so100_dual_follower and so100_dual_leader types.

This script shows how to use the new dual arm robot types with the default record function.
"""

from lerobot.common.robots.so100_Double_follower import SO100DualFollower, SO100DualFollowerConfig
from lerobot.common.teleoperators.so100_dual_leader import SO100DualLeader, SO100DualLeaderConfig
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.utils import hw_to_dataset_features
from lerobot.common.utils.control_utils import init_keyboard_listener
from lerobot.common.utils.utils import log_say
from lerobot.common.utils.visualization_utils import _init_rerun
from lerobot.record import record_loop

# Configuration
NUM_EPISODES = 3
FPS = 30
EPISODE_TIME_SEC = 60
RESET_TIME_SEC = 10
TASK_DESCRIPTION = "Dual arm manipulation task"

def main():
    """Main function demonstrating dual arm recording."""
    
    # Create the robot and teleoperator configurations
    robot_config = SO100DualFollowerConfig(
        left_port="/dev/tty.usbmodem585A0076841",  # Replace with your left follower port
        right_port="/dev/tty.usbmodem585A0076842",  # Replace with your right follower port
        id="dual_so100_follower"
    )
    
    teleop_config = SO100DualLeaderConfig(
        left_port="/dev/tty.usbmodem575E0031751",  # Replace with your left leader port
        right_port="/dev/tty.usbmodem575E0031752",  # Replace with your right leader port
        id="dual_so100_leader"
    )

    # Initialize the robot and teleoperator
    robot = SO100DualFollower(robot_config)
    teleop = SO100DualLeader(teleop_config)

    # Configure the dataset features
    action_features = hw_to_dataset_features(robot.action_features, "action")
    obs_features = hw_to_dataset_features(robot.observation_features, "observation")
    dataset_features = {**action_features, **obs_features}

    # Create the dataset
    dataset = LeRobotDataset.create(
        repo_id="<hf_username>/<dataset_repo_id>",  # Replace with your dataset repo
        fps=FPS,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=4,
    )

    # Initialize the keyboard listener and rerun visualization
    _, events = init_keyboard_listener()
    _init_rerun(session_name="dual_recording")

    # Connect the robot and teleoperator
    print("Connecting robot...")
    robot.connect()
    print("Connecting teleoperator...")
    teleop.connect()

    if not robot.is_connected or not teleop.is_connected:
        raise ValueError("Robot or teleoperator is not connected!")

    print("Starting recording session...")
    episode_idx = 0
    while episode_idx < NUM_EPISODES and not events["stop_recording"]:
        log_say(f"Recording episode {episode_idx + 1} of {NUM_EPISODES}")

        # Record episode
        record_loop(
            robot=robot,
            events=events,
            fps=FPS,
            teleop=teleop,
            dataset=dataset,
            control_time_s=EPISODE_TIME_SEC,
            single_task=TASK_DESCRIPTION,
            display_data=True,
        )

        # Reset the environment if not stopping or re-recording
        if not events["stop_recording"] and (episode_idx < NUM_EPISODES - 1 or events["rerecord_episode"]):
            log_say("Reset the environment")
            record_loop(
                robot=robot,
                events=events,
                fps=FPS,
                teleop=teleop,
                control_time_s=RESET_TIME_SEC,
                single_task=TASK_DESCRIPTION,
                display_data=True,
            )

        if events["rerecord_episode"]:
            log_say("Re-recording episode")
            events["rerecord_episode"] = False
            events["exit_early"] = False
            dataset.clear_episode_buffer()
            continue

        dataset.save_episode()
        episode_idx += 1

    # Clean up
    log_say("Stop recording")
    robot.disconnect()
    teleop.disconnect()
    
    # Optionally push to hub
    # dataset.push_to_hub()
    
    print(f"Recording complete! Recorded {episode_idx} episodes.")

if __name__ == "__main__":
    main() 