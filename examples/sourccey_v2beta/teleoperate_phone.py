import time

from lerobot.common.robots.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaConfig
from lerobot.common.robots.sourccey_v2beta.sourccey_v2beta import SourcceyV2Beta
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_phone_teleop.config_sourccey_v2beta_phone_teleop import SourcceyV2BetaPhoneTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_phone_teleop.sourccey_v2beta_phone_teleop import SourcceyV2BetaPhoneTeleop
from lerobot.common.utils.robot_utils import busy_wait
from lerobot.common.utils.visualization_utils import _init_rerun, log_rerun_data

FPS = 30

# Create the robot and teleoperator configurations
robot_config = SourcceyV2BetaConfig(
    left_arm_port="COM11", 
    right_arm_port="COM29", 
    enable_left_arm=True,    # Only enable left arm
    enable_right_arm=False,  # Disable right arm since it's not connected
    cameras={},              # Disable cameras - no cameras needed for phone teleop
    id="sourccey_v2beta"
)
phone_teleop_config = SourcceyV2BetaPhoneTeleopConfig(
    id="sourccey_v2beta_phone_teleop",
    control_left_arm=True,   # Only control left arm
    control_right_arm=False, # Don't control right arm
    enable_visualization=True,
    viser_port=8080,
    grpc_port=8765
)

# Use direct connection with phone teleoperation
robot = SourcceyV2Beta(robot_config)
phone_teleop = SourcceyV2BetaPhoneTeleop(phone_teleop_config)

# Connect directly to the robot and phone
robot.connect()
phone_teleop.connect()

_init_rerun(session_name="sourccey_v2beta_phone_teleop")

if not robot.is_connected or not phone_teleop.is_connected:
    raise ValueError("Robot or phone teleoperator is not connected!")

print("Phone teleoperation started!")
print("1. Open the VirtualManipulator app on your phone")
print("2. Connect to the gRPC server (default port 8765)")
print("3. Use your phone to control the LEFT robot arm")
print("4. Make a fist gesture to reset the mapping")
print("5. Use precision mode for fine control")
print("6. Open http://localhost:8080 for 3D visualization")
print("Note: Right arm is disabled - only left arm will be controlled")
print("Note: Cameras are disabled - no video feed needed for phone teleop")

while True:
    t0 = time.perf_counter()

    observation = robot.get_observation()

    # Get action from phone teleoperation (left arm only)
    arm_action = phone_teleop.get_action(observation)

    # For phone teleop, we only control the left arm (no base movement)
    base_action = {}

    log_rerun_data(observation, {**arm_action, **base_action})

    action = {**arm_action, **base_action} if len(base_action) > 0 else arm_action

    robot.send_action(action)

    busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0)) 