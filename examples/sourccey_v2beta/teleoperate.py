import time

from lerobot.common.robots.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_teleop.config_sourccey_v2beta_teleop import SourcceyV2BetaTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_teleop.sourccey_v2beta_teleop import SourcceyV2BetaTeleop
from lerobot.common.utils.robot_utils import busy_wait
from lerobot.common.utils.visualization_utils import _init_rerun, log_rerun_data

FPS = 30

# Create the robot and teleoperator configurations
robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="sourccey_v2beta")
teleop_arm_config = SourcceyV2BetaTeleopConfig(left_arm_port="COM13", right_arm_port="COM14", id="sourccey_v2beta_teleop")
keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

robot = SourcceyV2BetaClient(robot_config)
leader_arm = SourcceyV2BetaTeleop(teleop_arm_config)
keyboard = KeyboardTeleop(keyboard_config)

# To connect you already should have this script running on Sourccey V2 Beta: `python -m lerobot.common.robots.sourccey_v2beta.sourccey_v2beta_host --robot.id=sourccey_v2beta`
robot.connect()
leader_arm.connect()
keyboard.connect()

_init_rerun(session_name="sourccey_v2beta_teleop")

if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
    raise ValueError("Robot, leader arm of keyboard is not connected!")

while True:
    t0 = time.perf_counter()

    observation = robot.get_observation()

    arm_action = leader_arm.get_action()

    keyboard_keys = keyboard.get_action()
    base_action = robot._from_keyboard_to_base_action(keyboard_keys)

    log_rerun_data(observation, {**arm_action, **base_action})

    action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

    robot.send_action(action)

    busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
