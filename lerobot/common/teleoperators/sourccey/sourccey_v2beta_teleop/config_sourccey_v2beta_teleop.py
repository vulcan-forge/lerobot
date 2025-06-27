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

import os
from dataclasses import dataclass

from lerobot.common.constants import HF_LEROBOT_CONFIGURATION

from ...config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("sourccey_v2beta_teleop")
@dataclass
class SourcceyV2BetaTeleopConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str

    def __init__(self, **kwargs):
        """
        Initialize SourcceyV2BetaTeleopConfig with configuration loading.

        Args:
            **kwargs: Additional arguments to override loaded or default values
        """
        # Load configuration from file
        if 'teleop_config_id' in kwargs:
            config_data = self._load_configuration(kwargs['teleop_config_id'])
            self._override_config_values(config_data)

        # Call parent's __init__ with any additional kwargs
        super().__init__(**kwargs)

    def _load_configuration(self, teleop_config_id: str) -> dict:
        """Load configuration from file and return as dictionary."""
        config_file = HF_LEROBOT_CONFIGURATION / "teleoperators" / "sourccey_v2beta" / f"{teleop_config_id}.json"

        if config_file.exists():
            try:
                import json
                with open(config_file, 'r') as f:
                    config_data = json.load(f)
                return config_data
            except Exception as e:
                return {}
        else:
            return {}

    def _override_config_values(self, config_data: dict):
        """Override configuration values from configuration file."""

        # Override port
        if 'teleop_port' in config_data:
            self.port = config_data['teleop_port']

        # Override teleop_id
        if 'teleop_id' in config_data:
            self.id = config_data['teleop_id']
