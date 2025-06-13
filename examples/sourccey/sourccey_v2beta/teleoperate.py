from lerobot.common.robots.sourccey.sourccey_v2beta import SourcceyV2BetaClient, SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig

robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="my_sourccey_v2beta")

teleop__arm_config = SO100LeaderConfig(
    port="COM17",
    id="my_sourccey_v2beta_leader_arm",
)

teleop_keyboard_config = KeyboardTeleopConfig(
    id="my_laptop_keyboard",
)

robot = SourcceyV2BetaClient(robot_config)
teleop_arm = SO100Leader(teleop__arm_config)
telep_keyboard = KeyboardTeleop(teleop_keyboard_config)
robot.connect()
teleop_arm.connect()
telep_keyboard.connect()

while True:
    observation = robot.get_observation()

    arm_action = teleop_arm.get_action()
    arm_action = {f"arm_{k}": v for k, v in arm_action.items()}

    keyboard_keys = telep_keyboard.get_action()
    base_action = robot._from_keyboard_to_base_action(keyboard_keys)

    robot.send_action(arm_action | base_action)
