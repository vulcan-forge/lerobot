import time
from lerobot.common.robots.sourccey.sourccey_v2beta import SourcceyV2BetaClient, SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.sourccey_v2beta_leader.config_sourccey_v2beta_leader import SourcceyV2BetaLeaderConfig
from lerobot.common.teleoperators.sourccey_v2beta_leader.sourccey_v2beta_leader import SourcceyV2BetaLeader

robot_config = SourcceyV2BetaClientConfig(remote_ip="192.168.1.191", id="my_sourccey_v2beta")

teleop__arm_config = SourcceyV2BetaLeaderConfig(
    port="COM26",
    id="my_sourccey_v2beta_leader_arm",
)

teleop_keyboard_config = KeyboardTeleopConfig(
    id="my_laptop_keyboard",
)

robot = SourcceyV2BetaClient(robot_config)
teleop_arm = SourcceyV2BetaLeader(teleop__arm_config)
telep_keyboard = KeyboardTeleop(teleop_keyboard_config)
robot.connect()
teleop_arm.connect()
telep_keyboard.connect()

def convert_arm_action_to_left_right(arm_action: dict[str, float]) -> dict[str, float]:
    result = {}
    for k, v in arm_action.items():
        # Remove 'arm_' prefix from key
        new_key = k.replace('arm_', '')
        # Add left_arm_ or right_arm_ prefix
        result[f"left_arm_{new_key}"] = v
        result[f"right_arm_{new_key}"] = v
    return result


last_print_time = time.time()
while True:
    observation = robot.get_observation()

    arm_action = teleop_arm.get_action()
    arm_action = convert_arm_action_to_left_right(arm_action)

    # Split arm actions into left and right
    # arm_action = {
    #     f"left_arm_{k}": v for k, v in arm_action.items() if "left" in k
    # } | {
    #     f"right_arm_{k}": v for k, v in arm_action.items() if "right" in k
    # }

    # print arm_action every 3 seconds
    if time.time() - last_print_time > 3:
        print(arm_action)
        last_print_time = time.time()

    keyboard_keys = telep_keyboard.get_action()
    base_action = robot._from_keyboard_to_base_action(keyboard_keys)

    robot.send_action(arm_action | base_action)
