from dataclasses import asdict, dataclass
import logging
from pprint import pformat
import time
import draccus
import numpy as np
import torch
import rerun as rr
from lerobot.common.utils.utils import init_logging
from lerobot.common.utils.visualization_utils import _init_rerun
from lerobot.common.robots.sourccey.sourccey_v2beta import SourcceyV2BetaClient, SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_leader.config_sourccey_v2beta_leader import SourcceyV2BetaLeaderConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_leader.sourccey_v2beta_leader import SourcceyV2BetaLeader

@dataclass
class TeleoperateConfig:
    # Display all cameras on screen
    display_data: bool = False
    # Limit the maximum frames per second
    fps: int = 60

def display_data(observation, arm_action, base_action):
    """Display all data in Rerun."""
    # Log observations
    for obs, val in observation.items():
        if isinstance(val, float):
            rr.log(f"observation_{obs}", rr.Scalars(val))
        elif isinstance(val, (np.ndarray, torch.Tensor)):
            if isinstance(val, torch.Tensor):
                val = val.cpu().numpy()
            if len(val.shape) == 1:  # 1D array - log as individual scalars
                for i, v in enumerate(val):
                    rr.log(f"observation_{obs}_{i}", rr.Scalars(v))
            else:  # 2D or 3D array - log as image
                rr.log(f"observation_{obs}", rr.Image(val), static=True)

    # Log arm actions
    for act, val in arm_action.items():
        if isinstance(val, float):
            rr.log(f"action_{act}", rr.Scalars(val))
        elif isinstance(val, np.ndarray):
            for i, v in enumerate(val):
                rr.log(f"action_{act}_{i}", rr.Scalars(v))

    # Log base actions
    for act, val in base_action.items():
        if isinstance(val, float):
            rr.log(f"action_{act}", rr.Scalars(val))
        elif isinstance(val, np.ndarray):
            for i, v in enumerate(val):
                rr.log(f"action_{act}_{i}", rr.Scalars(v))


@draccus.wrap()
def teleoperate(cfg: TeleoperateConfig):
    if cfg.display_data:
        _init_rerun(session_name="sourccey_v2beta_teleoperation")

    # Initialize robot and teleop
    robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="my_sourccey_v2beta")
    teleop_arm_config = SourcceyV2BetaLeaderConfig(port="COM29", id="my_sourccey_v2beta_teleop")
    teleop_keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

    robot = SourcceyV2BetaClient(robot_config)
    teleop_arm = SourcceyV2BetaLeader(teleop_arm_config)
    telep_keyboard = KeyboardTeleop(teleop_keyboard_config)

    robot.connect()
    teleop_arm.connect()
    telep_keyboard.connect()

    print("Sourccey V2 Beta Teleoperation initialized")
    try:
        while True:
            observation = robot.get_observation()

            arm_action = teleop_arm.get_action()
            arm_action = {k: v for k, v in arm_action.items() if k.startswith(("left_arm", "right_arm"))}

            keyboard_keys = telep_keyboard.get_action()
            base_action = robot._from_keyboard_to_base_action(keyboard_keys)

            # Display all data in Rerun
            display_data(observation, arm_action, base_action)

            robot.send_action(arm_action | base_action)

    except KeyboardInterrupt:
        pass
    finally:
        rr.rerun_shutdown()
        robot.disconnect()
        teleop_arm.disconnect()
        telep_keyboard.disconnect()


if __name__ == "__main__":


    # Just call teleoperate() without any arguments
    teleoperate()
