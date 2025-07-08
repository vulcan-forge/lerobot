import time

from lerobot.common.robots.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaConfig
from lerobot.common.robots.sourccey_v2beta.sourccey_v2beta import SourcceyV2Beta
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_teleop.config_sourccey_v2beta_teleop import SourcceyV2BetaTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_teleop.sourccey_v2beta_teleop import SourcceyV2BetaTeleop
from lerobot.common.utils.robot_utils import busy_wait
from lerobot.common.utils.visualization_utils import _init_rerun, log_rerun_data

FPS = 30

# Create the robot and teleoperator configurations
robot_config = SourcceyV2BetaConfig(left_arm_port="COM28", right_arm_port="COM29", id="sourccey_v2beta")
teleop_arm_config = SourcceyV2BetaTeleopConfig(left_arm_port="COM28", right_arm_port="COM29", id="sourccey_v2beta_teleop")
keyboard_config = KeyboardTeleopConfig(id="my_laptop_keyboard")

# Use direct connection instead of client-server
robot = SourcceyV2Beta(robot_config)
leader_arm = SourcceyV2BetaTeleop(teleop_arm_config)
keyboard = KeyboardTeleop(keyboard_config)

# Connect directly to the robot (no server needed)
robot.connect()
leader_arm.connect()
keyboard.connect()

_init_rerun(session_name="sourccey_v2beta_teleop_direct")

if not robot.is_connected or not leader_arm.is_connected or not keyboard.is_connected:
    raise ValueError("Robot, leader arm or keyboard is not connected!")

print("Direct teleoperation started!")
print("Move the leader arm to control the robot arms.")
print("Note: Base movement is not implemented in direct mode yet.")

while True:
    t0 = time.perf_counter()

    observation = robot.get_observation()

    arm_action = leader_arm.get_action()

    # For now, we'll skip base movement since it's not implemented in direct mode
    # You can add base movement logic here if needed
    base_action = {}

    log_rerun_data(observation, {**arm_action, **base_action})

    action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

    robot.send_action(action)

    busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0)) 