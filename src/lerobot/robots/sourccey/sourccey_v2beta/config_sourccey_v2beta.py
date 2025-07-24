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
from typing import Optional

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
    # Arm Configuration
    left_arm_port: str
    right_arm_port: str
    enable_left_arm: bool = True
    enable_right_arm: bool = True  # Set to False if right arm is not connected
    
    # Motor Configuration
    use_degrees: bool = False
    
    # Safety Configuration
    max_relative_target: Optional[float] = None
    
    # Disconnect Configuration
    disable_torque_on_disconnect: bool = True
    
    # Camera Configuration
    cameras: dict[str, CameraConfig] = field(default_factory=sourccey_v2beta_cameras_config)


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

    cameras: dict[str, CameraConfig] = field(default_factory=sourccey_v2beta_cameras_config)

    polling_timeout_ms: int = 15
    connect_timeout_s: int = 5
