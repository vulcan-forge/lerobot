# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.common.cameras.configs import CameraConfig, Cv2Rotation
from lerobot.common.cameras.opencv.configuration_opencv import OpenCVCameraConfig

from lerobot.common.robots.config import RobotConfig
from lerobot.common.constants import HF_LEROBOT_CONFIGURATION


def sourccey_v2beta_cameras_config() -> dict[str, CameraConfig]:
    return {
        "front_left": OpenCVCameraConfig(
            index_or_path="/dev/video12", fps=30, width=320, height=240
        ),
        "front_right": OpenCVCameraConfig(
            index_or_path="/dev/video4", fps=30, width=320, height=240
        ),
        "wrist_left": OpenCVCameraConfig(
            index_or_path="/dev/video0", fps=30, width=320, height=240
        ),
        "wrist_right": OpenCVCameraConfig(
            index_or_path="/dev/video8", fps=30, width=320, height=240
        ),
    }

@RobotConfig.register_subclass("sourccey_v2beta")
@dataclass
class SourcceyV2BetaConfig(RobotConfig):
    robot_config_id: str = "sourccey_v2beta"

    # Default Values that are overridden by the robot_config_id
    port: str = "/dev/ttyACM0"

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=sourccey_v2beta_cameras_config)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False

    def __init__(self, **kwargs):
        """
        Initialize SourcceyV2BetaConfig with configuration loading.

        Args:
            **kwargs: Additional arguments to override loaded or default values
        """

        if 'robot_config_id' in kwargs:
            config_data = self._load_configuration(kwargs['robot_config_id'])
            self._override_config_values(config_data)

        # Call parent's __init__ with any additional kwargs
        super().__init__(**kwargs)

    def _load_configuration(self, robot_config_id: str) -> dict:
        """Load configuration from file and return as dictionary."""
        config_file = HF_LEROBOT_CONFIGURATION / "robots" / "sourccey_v2beta" / f"{robot_config_id}.json"

        if config_file.exists():
            try:
                import json
                with open(config_file, 'r') as f:
                    config_data = json.load(f)
                return config_data

            except Exception as e:
                print(f"Error loading configuration: {e}")
        return {}

    def _override_config_values(self, config_data: dict):
        """Override configuration values from configuration file."""

        # Override robot_id
        if 'id' in config_data:
            self.id = config_data['id']

        # Override port
        if 'port' in config_data:
            self.port = config_data['port']

        # Override disable_torque_on_disconnect
        if 'disable_torque_on_disconnect' in config_data:
            self.disable_torque_on_disconnect = config_data['disable_torque_on_disconnect']

        # Override max_relative_target
        if 'max_relative_target' in config_data:
            self.max_relative_target = config_data['max_relative_target']

        # Override use_degrees
        if 'use_degrees' in config_data:
            self.use_degrees = config_data['use_degrees']

        # Override camera values from config file
        if 'cameras' in config_data:
            self._override_cameras_from_config(config_data['cameras'])

        print("Configuration values updated successfully")

    def _override_cameras_from_config(self, cameras_config: dict):
        """Override camera values from configuration file."""

        converted_cameras = {}

        for camera_name, camera_data in cameras_config.items():
            if isinstance(camera_data, dict):
                camera_type = camera_data.get("type", "opencv")
                if camera_type == "opencv":
                    camera_config_data = {k: v for k, v in camera_data.items() if k != "type"}
                    converted_cameras[camera_name] = OpenCVCameraConfig(**camera_config_data)
                else:
                    raise ValueError(f"Unsupported camera type: {camera_type}")
            else:
                converted_cameras[camera_name] = camera_data

        self.cameras = converted_cameras


@dataclass
class SourcceyV2BetaHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Duration of the application
    connection_time_s: int = 86400

    # Watchdog: stop the robot if no command is received for over 1 hour.
    watchdog_timeout_ms: int = 3600000

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30


@RobotConfig.register_subclass("sourccey_v2beta_client")
@dataclass
class SourcceyV2BetaClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    cameras: dict[str, CameraConfig] = field(default_factory=sourccey_v2beta_cameras_config)

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5

    def __init__(self, **kwargs):
        """
        Initialize SourcceyV2BetaClientConfig with configuration loading.

        Args:
            **kwargs: Additional arguments to override loaded or default values
        """
        if 'robot_config_id' in kwargs:
            config_data = self._load_configuration(kwargs['robot_config_id'])
            self._override_config_values(config_data)

        # Call parent's __init__ with any additional kwargs
        super().__init__(**kwargs)

    def _load_configuration(self, robot_config_id: str) -> dict:
        """Load configuration from file and return as dictionary."""
        config_file = HF_LEROBOT_CONFIGURATION / "robots" / "sourccey_v2beta" / f"{robot_config_id}.json"

        if config_file.exists():
            try:
                import json
                with open(config_file, 'r') as f:
                    config_data = json.load(f)
                return config_data

            except Exception as e:
                print(f"Error loading configuration: {e}")
        return {}

    def _override_config_values(self, config_data: dict):
        """Override configuration values from configuration file."""

        # Override remote_ip
        if 'remote_ip' in config_data:
            self.remote_ip = config_data['remote_ip']

        # Override robot_id
        if 'id' in config_data:
            self.id = config_data['id']

        # Override camera values from config file
        if 'cameras' in config_data:
            self._override_cameras_from_config(config_data['cameras'])

    def _override_cameras_from_config(self, cameras_config: dict):
        """Override camera values from configuration file."""

        converted_cameras = {}
        for camera_name, camera_data in cameras_config.items():
            if isinstance(camera_data, dict):
                camera_type = camera_data.get("type", "opencv")
                if camera_type == "opencv":
                    camera_config_data = {k: v for k, v in camera_data.items() if k != "type"}
                    converted_cameras[camera_name] = OpenCVCameraConfig(**camera_config_data)
                else:
                    raise ValueError(f"Unsupported camera type: {camera_type}")
            else:
                converted_cameras[camera_name] = camera_data

        self.cameras = converted_cameras
