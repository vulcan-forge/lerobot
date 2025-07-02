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
from pathlib import Path
from typing import Optional

from ..config import TeleoperatorConfig
# Register config as ChoiceRegistry subclass so it can be instantiated via cfg.teleop.type="phone_teleoperator"
@TeleoperatorConfig.register_subclass("phone_teleoperator")
@dataclass
class PhoneTeleoperatorConfig(TeleoperatorConfig):
    """Configuration for phone teleoperation."""
    _target_: str = "lerobot.common.teleoperators.phone_teleoperator.phone_teleoperator.PhoneTeleoperator"
    
    # gRPC server settings
    grpc_port: int = 8765  # Default port to match phone app
    grpc_timeout: float = 100.0
    
    # Robot model paths. If not provided, we attempt to auto-resolve them from the `daxie` package.
    urdf_path: str = ""
    mesh_path: str = ""
    
    # IK solver settings
    target_link_name: str = "Fixed_Jaw"
    # rest_pose: tuple[float, ...] = (0.0, 2.733588, 2.411134, 0.322072, -1.570779, 0.849988)  # radians - initial position from IK solution
    rest_pose: tuple[float, ...] = (0.017499, -1.661131, 1.659391, 1.130985, 0.004688, 0.010240)  # radians - initial robot positions

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
    
    # Gripper settings
    gripper_min_pos: float = 0.0    # Gripper closed position (0% slider)
    gripper_max_pos: float = 0.875  # Gripper open position (100% slider)

    # Safety settings
    max_relative_target: Optional[float] = None

    # ---------------------------------------------------------------------
    # Post-init logic: ensure URDF and mesh paths are set, attempting to find
    # them automatically if the user did not supply any.
    # ---------------------------------------------------------------------
    def __post_init__(self):
        # If either path is missing, try to resolve them again (lazy), giving
        # precedence to any user-supplied values.
        if not self.urdf_path or not self.mesh_path:
            try:
                from daxie import get_so100_path

                resolved_urdf, resolved_mesh = get_so100_path()
                if not self.urdf_path:
                    self.urdf_path = str(resolved_urdf)
                if not self.mesh_path:
                    self.mesh_path = str(resolved_mesh)
            except Exception:
                # Leave as is; PhoneTeleoperator.connect() will raise a clear
                # error if paths remain invalid.
                pass 