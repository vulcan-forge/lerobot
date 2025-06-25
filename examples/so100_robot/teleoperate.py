from dataclasses import asdict, dataclass
from pprint import pformat
import draccus
import rerun as rr
from examples.so100_robot.utils import display_data
from lerobot.common.robots.so100_robot.config_so100_robot import SO100RobotClientConfig
from lerobot.common.robots.so100_robot.so100_robot_client import SO100RobotClient
from lerobot.common.teleoperators.so100_leader.config_so100_leader import SO100LeaderConfig
from lerobot.common.teleoperators.so100_leader.so100_leader import SO100Leader
from lerobot.common.teleoperators.so100_leader.config_so100_leader import SO100LeaderConfig
from lerobot.common.teleoperators.so100_leader.so100_leader import SO100Leader
from lerobot.common.utils.utils import init_logging
from lerobot.common.utils.visualization_utils import _init_rerun
from lerobot.common.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig

@dataclass
class TeleoperateConfig:
    # Limit the maximum frames per second
    fps: int = 60
    # Robot configuration
    robot_ip: str = "192.168.1.208" # 192.168.1.191 # (First robot) # 192.168.1.169 # (Second robot)
    robot_id: str = "so100_robot"
    # Leader arm configuration
    leader_arm_port: str = "/dev/ttyUSB0"
    leader_arm_id: str = "so100_robot_teleop"
    # Keyboard configuration
    keyboard_id: str = "my_laptop_keyboard"
    # Rerun session name
    display_data: bool = False
    rerun_session_name: str = "so100_robot_teleoperation"


@draccus.wrap()
def teleoperate(cfg: TeleoperateConfig):
    if cfg.display_data:
        _init_rerun(session_name=cfg.rerun_session_name)

    # Initialize robot and teleop devices
    robot_config = SO100RobotClientConfig(
        remote_ip=cfg.robot_ip,
        id=cfg.robot_id
    )
    teleop_arm_config = SO100LeaderConfig(
        port=cfg.leader_arm_port,
        id=cfg.leader_arm_id
    )
    teleop_keyboard_config = KeyboardTeleopConfig(id=cfg.keyboard_id)

    robot = SO100RobotClient(robot_config)
    teleop_arm = SO100Leader(teleop_arm_config)
    telep_keyboard = KeyboardTeleop(teleop_keyboard_config)

    # Connect to all devices
    robot.connect()
    teleop_arm.connect()
    telep_keyboard.connect()

    # Check connection status
    if not all([robot.is_connected, teleop_arm.is_connected, telep_keyboard.is_connected]):
        print("Failed to connect to one or more devices:")
        print(f"  Robot: {robot.is_connected}")
        print(f"  Leader Arm: {teleop_arm.is_connected}")
        print(f"  Keyboard: {telep_keyboard.is_connected}")
        return

    print("Sourccey V2 Beta Teleoperation initialized")
    print(f"Robot IP: {cfg.robot_ip}")
    print(f"Leader Arm Port: {cfg.leader_arm_port}")
    print(f"Display Data: {cfg.display_data}")
    print("Press Ctrl+C to stop teleoperation")

    try:
        while True:
            observation = robot.get_observation()

            arm_action = teleop_arm.get_action()
            arm_action = {k: v for k, v in arm_action.items() if k.startswith(("arm",))}

            keyboard_keys = telep_keyboard.get_action()
            base_action = robot._from_keyboard_to_base_action(keyboard_keys)

            # Display all data in Rerun
            if cfg.display_data:
                display_data(observation, arm_action, base_action)

            action = arm_action | base_action if len(base_action) > 0 else arm_action
            robot.send_action(action)

    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user")
    finally:
        print("Cleaning up...")
        rr.rerun_shutdown()
        robot.disconnect()
        teleop_arm.disconnect()
        telep_keyboard.disconnect()
        print("Teleoperation ended")


if __name__ == "__main__":
    teleoperate()
