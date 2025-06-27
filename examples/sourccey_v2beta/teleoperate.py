from dataclasses import dataclass
import draccus
import rerun as rr
from examples.sourccey_v2beta.utils import display_data
from lerobot.common.robots.sourccey_v2beta.config_sourccey_v2beta import SourcceyV2BetaClientConfig
from lerobot.common.robots.sourccey_v2beta.sourccey_v2beta_client import SourcceyV2BetaClient
from lerobot.common.utils.visualization_utils import _init_rerun
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_teleop.config_sourccey_v2beta_teleop import SourcceyV2BetaTeleopConfig
from lerobot.common.teleoperators.sourccey.sourccey_v2beta_teleop.sourccey_v2beta_teleop import SourcceyV2BetaTeleop

@dataclass
class TeleoperateConfig:
    # Limit the maximum frames per second
    fps: int = 60
    # Robot configuration
    robot_config_id: str | None = "sourccey_v2beta_client"
    robot_ip: str | None = None # "192.168.1.191" # (First robot) # "192.168.1.169" # (Second robot)
    robot_id: str | None = None # "sourccey_v2beta"
    # Teleop configuration
    teleop_config_id: str | None = "sourccey_v2beta_teleop"
    teleop_port: str | None = None # "/dev/ttyUSB0"
    teleop_id: str | None = None # "sourccey_v2beta_teleop"
    # Keyboard configuration
    keyboard_id: str | None = "keyboard"
    # Rerun session name
    display_data: bool = False
    rerun_session_name: str = "sourccey_v2beta_teleoperation"

@draccus.wrap()
def teleoperate(cfg: TeleoperateConfig):
    if cfg.display_data:
        _init_rerun(session_name=cfg.rerun_session_name)

    # Initialize robot and teleop devices
    if cfg.robot_config_id is not None:
        robot_config = SourcceyV2BetaClientConfig(robot_config_id=cfg.robot_config_id)
        print(f"Using robot configuration: {cfg.robot_config_id}")
    else:
        robot_config = SourcceyV2BetaClientConfig(
            remote_ip=cfg.robot_ip,
            id=cfg.robot_id
        )
        print(f"Using command line configuration - IP: {cfg.robot_ip}, ID: {cfg.robot_id}")

    if cfg.teleop_config_id is not None:
        teleop_arm_config = SourcceyV2BetaTeleopConfig(teleop_config_id=cfg.teleop_config_id)
        print(f"Using teleop configuration: {cfg.teleop_config_id}")
    else:
        teleop_arm_config = SourcceyV2BetaTeleopConfig(
            port=cfg.teleop_port,
            id=cfg.teleop_id
        )
        print(f"Using command line configuration - Port: {cfg.teleop_port}, ID: {cfg.teleop_id}")

    teleop_keyboard_config = KeyboardTeleopConfig(id=cfg.keyboard_id)

    robot = SourcceyV2BetaClient(robot_config)
    teleop_arm = SourcceyV2BetaTeleop(teleop_arm_config)
    # telep_keyboard = KeyboardTeleop(teleop_keyboard_config)

    # Connect to all devices
    robot.connect()
    teleop_arm.connect()
    # telep_keyboard.connect()

    # Check connection status
    print(f"Robot: {robot.is_connected}")
    print(f"Leader Arm: {teleop_arm.is_connected}")
    if not all([robot.is_connected, teleop_arm.is_connected]):
        print("Failed to connect to one or more devices:")
        print(f"  Robot: {robot.is_connected}")
        print(f"  Leader Arm: {teleop_arm.is_connected}")
        return

    print()
    print("Sourccey V2 Beta Teleoperation initialized")
    print(f"Robot IP: {robot_config.remote_ip}")
    print(f"Robot ID: {robot_config.id}")
    print(f"Leader Arm Port: {cfg.teleop_port}")
    print(f"Display Data: {cfg.display_data}")
    print("Press Ctrl+C to stop teleoperation")

    try:
        while True:
            observation = robot.get_observation()
            arm_action = teleop_arm.get_action()
            # keyboard_keys = telep_keyboard.get_action()

            # Display all data in Rerun
            if cfg.display_data:
                display_data(observation, arm_action)

            action = arm_action
            robot.send_action(action)

    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user")
    finally:
        print("Cleaning up...")
        rr.rerun_shutdown()
        robot.disconnect()
        teleop_arm.disconnect()
        # telep_keyboard.disconnect()
        print("Teleoperation ended")


if __name__ == "__main__":
    teleoperate()
