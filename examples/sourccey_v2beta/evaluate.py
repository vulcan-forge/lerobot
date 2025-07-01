from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.utils import hw_to_dataset_features
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.robots.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.utils.control_utils import init_keyboard_listener
from lerobot.common.utils.utils import log_say
from lerobot.common.utils.visualization_utils import _init_rerun
from lerobot.record import record_loop

NUM_EPISODES = 1
FPS = 30
EPISODE_TIME_SEC = 60
TASK_DESCRIPTION = "Pick up the tape and put it in the cup"

# Create the robot and teleoperator configurations
robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="sourccey_v2beta")
robot = SourcceyV2BetaClient(robot_config)

policy = ACTPolicy.from_pretrained("outputs/train/act__sourccey_v2beta-001__tape-a__set000/checkpoints/020000/pretrained_model")

# Configure the dataset features
action_features = hw_to_dataset_features(robot.action_features, "action")
obs_features = hw_to_dataset_features(robot.observation_features, "observation")
dataset_features = {**action_features, **obs_features}

# Create the dataset
dataset = LeRobotDataset.create(
    repo_id="local/eval__sourccey_v2beta-001__tape-a__set000",
    fps=FPS,
    features=dataset_features,
    robot_type=robot.name,
    use_videos=True,
    image_writer_threads=4,
)

# To connect you already should have this script running on Sourccey V2 Beta: `python -m lerobot.common.robots.sourccey_v2beta.sourccey_v2beta_host --robot.id=sourccey_v2beta`
robot.connect()

_init_rerun(session_name="recording")

listener, events = init_keyboard_listener()

if not robot.is_connected:
    raise ValueError("Robot is not connected!")

recorded_episodes = 0
while recorded_episodes < NUM_EPISODES and not events["stop_recording"]:
    log_say(f"Running inference, recording eval episode {recorded_episodes} of {NUM_EPISODES}")

    # Run the policy inference loop
    record_loop(
        robot=robot,
        events=events,
        fps=FPS,
        policy=policy,
        dataset=dataset,
        control_time_s=EPISODE_TIME_SEC,
        single_task=TASK_DESCRIPTION,
        display_data=True,
    )

    # Logic for reset env
    if not events["stop_recording"] and (
        (recorded_episodes < NUM_EPISODES - 1) or events["rerecord_episode"]
    ):
        log_say("Reset the environment")
        record_loop(
            robot=robot,
            events=events,
            fps=FPS,
            control_time_s=EPISODE_TIME_SEC,
            single_task=TASK_DESCRIPTION,
            display_data=True,
        )

    if events["rerecord_episode"]:
        log_say("Re-record episode")
        events["rerecord_episode"] = False
        events["exit_early"] = False
        dataset.clear_episode_buffer()
        continue

    dataset.save_episode()
    recorded_episodes += 1

# Upload to hub and clean up
# dataset.push_to_hub()

robot.disconnect()
listener.stop()
