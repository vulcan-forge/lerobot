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

# Control table for DC motors with potentiometer encoding
# This is a simplified control table that mimics the feetech structure
# but adapted for DC motors with analog potentiometer feedback

# data_name: (address, size_byte)
DC_MOTOR_CONTROL_TABLE = {
    # EPROM (configuration)
    "Firmware_Major_Version": (0, 1),  # read-only
    "Firmware_Minor_Version": (1, 1),  # read-only
    "Model_Number": (3, 2),  # read-only
    "ID": (5, 1),
    "Baud_Rate": (6, 1),
    "Return_Delay_Time": (7, 1),
    "Response_Status_Level": (8, 1),
    "Min_Position_Limit": (9, 2),
    "Max_Position_Limit": (11, 2),
    "Max_Temperature_Limit": (13, 1),
    "Max_Voltage_Limit": (14, 1),
    "Min_Voltage_Limit": (15, 1),
    "Max_Torque_Limit": (16, 2),
    "Phase": (18, 1),
    "Unloading_Condition": (19, 1),
    "LED_Alarm_Condition": (20, 1),
    "P_Coefficient": (21, 1),
    "D_Coefficient": (22, 1),
    "I_Coefficient": (23, 1),
    "Minimum_Startup_Force": (24, 2),
    "CW_Dead_Zone": (26, 1),
    "CCW_Dead_Zone": (27, 1),
    "Protection_Current": (28, 2),
    "Angular_Resolution": (30, 1),
    "Homing_Offset": (31, 2),
    "Operating_Mode": (33, 1),
    "Protective_Torque": (34, 1),
    "Protection_Time": (35, 1),
    "Overload_Torque": (36, 1),
    "Velocity_closed_loop_P_proportional_coefficient": (37, 1),
    "Over_Current_Protection_Time": (38, 1),
    "Velocity_closed_loop_I_integral_coefficient": (39, 1),
    # SRAM (runtime)
    "Torque_Enable": (40, 1),
    "Acceleration": (41, 1),
    "Goal_Position": (42, 2),
    "Goal_Time": (44, 2),
    "Goal_Velocity": (46, 2),
    "Torque_Limit": (48, 2),
    "Lock": (55, 1),
    "Present_Position": (56, 2),  # read-only (from potentiometer)
    "Present_Velocity": (58, 2),  # read-only
    "Present_Load": (60, 2),  # read-only
    "Present_Voltage": (62, 1),  # read-only
    "Present_Temperature": (63, 1),  # read-only
    "Status": (65, 1),  # read-only
    "Moving": (66, 1),  # read-only
    "Present_Current": (69, 2),  # read-only
    "Goal_Position_2": (71, 2),  # read-only
    # Factory
    "Moving_Velocity": (80, 1),
    "Moving_Velocity_Threshold": (80, 1),
    "DTs": (81, 1),  # (ms)
    "Velocity_Unit_factor": (82, 1),
    "Hts": (83, 1),  # (ns) valid for firmware >= 2.54, other versions keep 0
    "Maximum_Velocity_Limit": (84, 1),
    "Maximum_Acceleration": (85, 1),
    "Acceleration_Multiplier ": (86, 1),  # Acceleration multiplier in effect when acceleration is 0
}

# Baudrate table (same as feetech for compatibility)
DC_MOTOR_BAUDRATE_TABLE = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    57_600: 5,
    38_400: 6,
    19_200: 7,
}

# Model-specific tables
MODEL_CONTROL_TABLE = {
    "dc_motor": DC_MOTOR_CONTROL_TABLE,
    "dc_motor_pot": DC_MOTOR_CONTROL_TABLE,
    "shoulder_lift_dc": DC_MOTOR_CONTROL_TABLE,
}

MODEL_RESOLUTION = {
    "dc_motor": 4096,  # 12-bit ADC typical for potentiometers
    "dc_motor_pot": 4096,
    "shoulder_lift_dc": 4096,
}

MODEL_BAUDRATE_TABLE = {
    "dc_motor": DC_MOTOR_BAUDRATE_TABLE,
    "dc_motor_pot": DC_MOTOR_BAUDRATE_TABLE,
    "shoulder_lift_dc": DC_MOTOR_BAUDRATE_TABLE,
}

# Encoding table (how data is encoded/decoded)
MODEL_ENCODING_TABLE = {
    "dc_motor": {
        "Goal_Position": "sign_magnitude",
        "Present_Position": "sign_magnitude",
        "Goal_Velocity": "sign_magnitude",
        "Present_Velocity": "sign_magnitude",
        "Goal_Time": "unsigned",
        "Torque_Limit": "unsigned",
        "Present_Load": "sign_magnitude",
        "Present_Current": "sign_magnitude",
        "Homing_Offset": "sign_magnitude",
        "Min_Position_Limit": "unsigned",
        "Max_Position_Limit": "unsigned",
        "Max_Torque_Limit": "unsigned",
        "Protection_Current": "unsigned",
        "Minimum_Startup_Force": "unsigned",
    },
    "dc_motor_pot": {
        "Goal_Position": "sign_magnitude",
        "Present_Position": "sign_magnitude",
        "Goal_Velocity": "sign_magnitude",
        "Present_Velocity": "sign_magnitude",
        "Goal_Time": "unsigned",
        "Torque_Limit": "unsigned",
        "Present_Load": "sign_magnitude",
        "Present_Current": "sign_magnitude",
        "Homing_Offset": "sign_magnitude",
        "Min_Position_Limit": "unsigned",
        "Max_Position_Limit": "unsigned",
        "Max_Torque_Limit": "unsigned",
        "Protection_Current": "unsigned",
        "Minimum_Startup_Force": "unsigned",
    },
    "shoulder_lift_dc": {
        "Goal_Position": "sign_magnitude",
        "Present_Position": "sign_magnitude",
        "Goal_Velocity": "sign_magnitude",
        "Present_Velocity": "sign_magnitude",
        "Goal_Time": "unsigned",
        "Torque_Limit": "unsigned",
        "Present_Load": "sign_magnitude",
        "Present_Current": "sign_magnitude",
        "Homing_Offset": "sign_magnitude",
        "Min_Position_Limit": "unsigned",
        "Max_Position_Limit": "unsigned",
        "Max_Torque_Limit": "unsigned",
        "Protection_Current": "unsigned",
        "Minimum_Startup_Force": "unsigned",
    },
}

# Model number table (unique identifiers for each model)
MODEL_NUMBER_TABLE = {
    "dc_motor": 1001,
    "dc_motor_pot": 1002,
    "shoulder_lift_dc": 1003,
}

# Protocol version table
MODEL_PROTOCOL = {
    "dc_motor": 0,
    "dc_motor_pot": 0,
    "shoulder_lift_dc": 0,
}

# Scan baudrates for motor discovery
SCAN_BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200, 57_600, 38_400, 19_200]

# Firmware version constants
FIRMWARE_MAJOR_VERSION = (0, 1)
FIRMWARE_MINOR_VERSION = (1, 1)
MODEL_NUMBER = (3, 2) 