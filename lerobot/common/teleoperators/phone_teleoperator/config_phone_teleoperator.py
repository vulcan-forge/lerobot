#!/usr/bin/env python

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

from dataclasses import dataclass
from typing import Optional

from lerobot.common.teleoperators.config import TeleoperatorConfig


@dataclass
class PhoneTeleoperatorConfig(TeleoperatorConfig):
    """Configuration for phone teleoperation."""
    _target_: str = "lerobot.common.teleoperators.phone_teleoperator.phone_teleoperator.PhoneTeleoperator"
    
    # gRPC server settings
    grpc_port: int = 8765  # Default port to match phone app
    grpc_timeout: float = 100.0
    
    # Robot model paths  
    urdf_path: str = ""
    mesh_path: str = ""
    
    # IK solver settings
    target_link_name: str = "Fixed_Jaw"
    rest_pose: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # radians - conservative middle position
    
    # Phone mapping settings
    rotation_sensitivity: float = 1.0
    sensitivity_normal: float = 0.5
    sensitivity_precision: float = 0.2
    
    # Initial robot pose (when connecting phone)
    initial_position: tuple[float, ...] = (0.0, -0.17, 0.237)  # meters
    initial_wxyz: tuple[float, ...] = (0, 0, 1, 0)  # quaternion (w,x,y,z)
    
    # Visualization settings
    enable_visualization: bool = True
    viser_port: int = 8080

    # Safety settings
    max_relative_target: Optional[float] = None 