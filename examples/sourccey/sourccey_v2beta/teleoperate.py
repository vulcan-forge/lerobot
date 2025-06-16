import time
import numpy as np
import rerun as rr
from lerobot.common.utils.visualization_utils import _init_rerun
from lerobot.common.robots.sourccey.sourccey_v2beta import SourcceyV2BetaClient, SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_leader.config_sourccey_v2beta_leader import SourcceyV2BetaLeaderConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_leader.sourccey_v2beta_leader import SourcceyV2BetaLeader

# Initialize Rerun for visualization
_init_rerun(session_name="sourccey_v2beta_teleoperation")

# Create a container for camera views
rr.log("cameras", rr.Transform3D())

robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="my_sourccey_v2beta")

teleop_arm_config = SourcceyV2BetaLeaderConfig(
    port="COM28",
    id="my_sourccey_v2beta_teleop",
)

teleop_keyboard_config = KeyboardTeleopConfig(
    id="my_laptop_keyboard",
)

robot = SourcceyV2BetaClient(robot_config)
teleop_arm = SourcceyV2BetaLeader(teleop_arm_config)
telep_keyboard = KeyboardTeleop(teleop_keyboard_config)
robot.connect()
teleop_arm.connect()
telep_keyboard.connect()

last_print_time = time.time()
try:
    while True:
        observation = robot.get_observation()

        arm_action = teleop_arm.get_action()
        arm_action = {k: v for k, v in arm_action.items() if k.startswith(("left_arm", "right_arm"))}

        keyboard_keys = telep_keyboard.get_action()
        base_action = robot._from_keyboard_to_base_action(keyboard_keys)

        # Log observations and actions to Rerun
        for obs, val in observation.items():
            if isinstance(val, float):
                rr.log(f"observation_{obs}", rr.Scalar(val))
            elif isinstance(val, np.ndarray):
                if len(val.shape) == 2 or len(val.shape) == 3:  # Only log 2D or 3D arrays as images
                    # Log camera images in the cameras container
                    if obs in ["front_left", "front_right", "wrist_left", "wrist_right"]:
                        rr.log(f"cameras/{obs}", rr.Image(val), static=True)
                    else:
                        rr.log(f"observation_{obs}", rr.Image(val), static=True)
                else:  # Log 1D arrays as individual scalars
                    for i, v in enumerate(val):
                        rr.log(f"observation_{obs}_{i}", rr.Scalar(v))

        # Log arm actions
        for act, val in arm_action.items():
            if isinstance(val, float):
                rr.log(f"action_{act}", rr.Scalar(val))
            elif isinstance(val, np.ndarray):
                for i, v in enumerate(val):
                    rr.log(f"action_{act}_{i}", rr.Scalar(v))

        # Log base actions
        for act, val in base_action.items():
            if isinstance(val, float):
                rr.log(f"action_{act}", rr.Scalar(val))
            elif isinstance(val, np.ndarray):
                for i, v in enumerate(val):
                    rr.log(f"action_{act}_{i}", rr.Scalar(v))

        robot.send_action(arm_action | base_action)

except KeyboardInterrupt:
    pass
finally:
    rr.rerun_shutdown()
    robot.disconnect()
    teleop_arm.disconnect()
    telep_keyboard.disconnect()
