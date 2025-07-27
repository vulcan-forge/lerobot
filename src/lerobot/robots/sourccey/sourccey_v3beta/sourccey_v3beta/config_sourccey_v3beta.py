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

from lerobot.cameras.configs import CameraConfig, Cv2Rotation
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

from lerobot.robots.config import RobotConfig
from lerobot.constants import HF_LEROBOT_CONFIGURATION


def sourccey_v3beta_cameras_config() -> dict[str, CameraConfig]:
    config = {
        "front_left": OpenCVCameraConfig(
            index_or_path="/dev/video0", fps=10, width=640, height=480
        ),
        "front_right": OpenCVCameraConfig(
            index_or_path="/dev/video2", fps=10, width=640, height=480
        ),
        "wrist_left": OpenCVCameraConfig(
            index_or_path="/dev/video4", fps=10, width=640, height=480
        ),
        "wrist_right": OpenCVCameraConfig(
            index_or_path="/dev/video6", fps=10, width=640, height=480
        ),
    }
    return config

@RobotConfig.register_subclass("sourccey_v3beta")
@dataclass
class SourcceyV3BetaConfig(RobotConfig):
    left_arm_port: str = "/dev/ttyACM0"
    right_arm_port: str = "/dev/ttyACM1"

    # Optional
    left_arm_disable_torque_on_disconnect: bool = True
    left_arm_max_relative_target: int | None = None
    left_arm_use_degrees: bool = False
    right_arm_disable_torque_on_disconnect: bool = True
    right_arm_max_relative_target: int | None = None
    right_arm_use_degrees: bool = False

    cameras: dict[str, CameraConfig] = field(default_factory=sourccey_v3beta_cameras_config)

@dataclass
class SourcceyV3BetaHostConfig:
    # Network Configuration
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    # Duration of the application
    connection_time_s: int = 86400

    # Watchdog: stop the robot if no command is received for over 1 hour.
    watchdog_timeout_ms: int = 3600000

    # If robot jitters decrease the frequency and monitor cpu load with `top` in cmd
    max_loop_freq_hz: int = 30


@RobotConfig.register_subclass("sourccey_v3beta_client")
@dataclass
class SourcceyV3BetaClientConfig(RobotConfig):
    # Network Configuration
    remote_ip: str
    port_zmq_cmd: int = 5555
    port_zmq_observations: int = 5556

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "w",
            "backward": "s",
            "left": "a",
            "right": "d",
            "rotate_left": "z",
            "rotate_right": "x",
            # Speed control
            "speed_up": "r",
            "speed_down": "f",
            # quit teleop
            "quit": "q",
        }
    )

    cameras: dict[str, CameraConfig] = field(default_factory=sourccey_v3beta_cameras_config)

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5
