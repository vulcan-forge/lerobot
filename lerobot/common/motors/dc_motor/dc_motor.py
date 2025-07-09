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

import logging
import time
from copy import deepcopy
from enum import Enum
from typing import Optional

import serial

from lerobot.common.utils.encoding_utils import decode_sign_magnitude, encode_sign_magnitude

from ..motors_bus import Motor, MotorCalibration, MotorsBus, NameOrID, Value, get_address
from .tables import (
    FIRMWARE_MAJOR_VERSION,
    FIRMWARE_MINOR_VERSION,
    MODEL_BAUDRATE_TABLE,
    MODEL_CONTROL_TABLE,
    MODEL_ENCODING_TABLE,
    MODEL_NUMBER,
    MODEL_NUMBER_TABLE,
    MODEL_PROTOCOL,
    MODEL_RESOLUTION,
    SCAN_BAUDRATES,
)

DEFAULT_PROTOCOL_VERSION = 0
DEFAULT_BAUDRATE = 115_200  # Lower baudrate for DC motors
DEFAULT_TIMEOUT_MS = 2000

NORMALIZED_DATA = ["Goal_Position", "Present_Position"]

logger = logging.getLogger(__name__)


class OperatingMode(Enum):
    # position servo mode
    POSITION = 0
    # The motor is in constant speed mode, which is controlled by parameter 0x2e, and the highest bit 15 is
    # the direction bit
    VELOCITY = 1
    # PWM open-loop speed regulation mode, with parameter 0x2c running time parameter control, bit11 as
    # direction bit
    PWM = 2
    # In step servo mode, the number of step progress is represented by parameter 0x2a, and the highest bit 15
    # is the direction bit
    STEP = 3


class DriveMode(Enum):
    NON_INVERTED = 0
    INVERTED = 1


class TorqueMode(Enum):
    ENABLED = 1
    DISABLED = 0


class DCMotorController:
    """
    A simple DC motor controller that simulates the feetech protocol
    but uses analog potentiometer feedback instead of digital encoders.
    """
    
    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_open = False
        self.motor_data = {}  # Simulated motor memory
        self.potentiometer_value = 2048  # Simulated potentiometer reading (0-4095)
        
    def openPort(self):
        """Open the serial port for communication."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.is_open = True
            logger.info(f"DC Motor controller connected to {self.port} at {self.baudrate} baud")
        except Exception as e:
            logger.error(f"Failed to open port {self.port}: {e}")
            raise
            
    def closePort(self):
        """Close the serial port."""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.is_open = False
        
    def readPort(self, length: int) -> bytes:
        """Read data from the serial port."""
        if not self.is_open:
            raise RuntimeError("Port is not open")
        return self.ser.read(length)
        
    def writePort(self, packet: bytes):
        """Write data to the serial port."""
        if not self.is_open:
            raise RuntimeError("Port is not open")
        self.ser.write(packet)
        
    def getCurrentTime(self) -> float:
        """Get current time for timeout calculations."""
        return time.time()
        
    def setPacketTimeout(self, packet_length: int):
        """Set packet timeout."""
        # Simplified timeout calculation
        pass


class DCMotorsBus(MotorsBus):
    """
    The DCMotorsBus class allows to efficiently read and write to DC motors with potentiometer encoding.
    It simulates the feetech protocol but uses analog feedback instead of digital encoders.
    """

    apply_drive_mode = True
    available_baudrates = deepcopy(SCAN_BAUDRATES)
    default_baudrate = DEFAULT_BAUDRATE
    default_timeout = DEFAULT_TIMEOUT_MS
    model_baudrate_table = deepcopy(MODEL_BAUDRATE_TABLE)
    model_ctrl_table = deepcopy(MODEL_CONTROL_TABLE)
    model_encoding_table = deepcopy(MODEL_ENCODING_TABLE)
    model_number_table = deepcopy(MODEL_NUMBER_TABLE)
    model_resolution_table = deepcopy(MODEL_RESOLUTION)
    normalized_data = deepcopy(NORMALIZED_DATA)

    def __init__(
        self,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None,
        protocol_version: int = DEFAULT_PROTOCOL_VERSION,
    ):
        super().__init__(port, motors, calibration)
        self.protocol_version = protocol_version
        self._assert_same_protocol()
        
        # Create DC motor controller
        self.controller = DCMotorController(port, self.default_baudrate)
        
        # Initialize motor data
        for motor_name, motor in self.motors.items():
            self.controller.motor_data[motor.id] = {
                "Present_Position": 2048,  # Center position
                "Present_Velocity": 0,
                "Present_Load": 0,
                "Present_Voltage": 120,  # 12V
                "Present_Temperature": 25,
                "Present_Current": 0,
                "Status": 0,
                "Moving": 0,
                "Torque_Enable": 0,
                "Goal_Position": 2048,
                "Goal_Velocity": 0,
                "Operating_Mode": OperatingMode.POSITION.value,
            }

    def _assert_same_protocol(self) -> None:
        if any(MODEL_PROTOCOL[model] != self.protocol_version for model in self.models):
            raise RuntimeError("Some motors use an incompatible protocol.")

    def _assert_protocol_is_compatible(self, instruction_name: str) -> None:
        if instruction_name == "sync_read" and self.protocol_version == 1:
            raise NotImplementedError(
                "'Sync Read' is not available with DC motors using Protocol 1. Use 'Read' sequentially instead."
            )
        if instruction_name == "broadcast_ping" and self.protocol_version == 1:
            raise NotImplementedError(
                "'Broadcast Ping' is not available with DC motors using Protocol 1. Use 'Ping' sequentially instead."
            )

    def _handshake(self) -> None:
        """Perform handshake with DC motors."""
        self._assert_motors_exist()
        # For DC motors, we just verify the controller is connected
        if not self.controller.is_open:
            raise RuntimeError("DC motor controller is not connected")

    def _find_single_motor(self, motor: str, initial_baudrate: int | None = None) -> tuple[int, int]:
        """Find a single motor on the bus."""
        model = self.motors[motor].model
        search_baudrates = (
            [initial_baudrate] if initial_baudrate is not None else self.model_baudrate_table[model]
        )
        expected_model_nb = self.model_number_table[model]

        for baudrate in search_baudrates:
            self.set_baudrate(baudrate)
            id_model = self.broadcast_ping()
            if id_model:
                found_id, found_model = next(iter(id_model.items()))
                if found_model != expected_model_nb:
                    raise RuntimeError(
                        f"Found one motor on {baudrate=} with id={found_id} but it has a "
                        f"model number '{found_model}' different than the one expected: '{expected_model_nb}'. "
                        f"Make sure you are connected only connected to the '{motor}' motor (model '{model}')."
                    )
                return baudrate, found_id

        raise RuntimeError(f"Motor '{motor}' (model '{model}') was not found. Make sure it is connected.")

    def configure_motors(self) -> None:
        """Configure all motors on the bus."""
        for motor_name, motor in self.motors.items():
            # Set default operating mode to position control
            self.controller.motor_data[motor.id]["Operating_Mode"] = OperatingMode.POSITION.value
            # Enable torque
            self.controller.motor_data[motor.id]["Torque_Enable"] = 1

    @property
    def is_calibrated(self) -> bool:
        """Check if motors are calibrated."""
        return len(self.calibration) == len(self.motors)

    def read_calibration(self) -> dict[str, MotorCalibration]:
        """Read calibration data from motors."""
        calibration_dict = {}
        for motor_name, motor in self.motors.items():
            if motor_name in self.calibration:
                calibration_dict[motor_name] = self.calibration[motor_name]
        return calibration_dict

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration]) -> None:
        """Write calibration data to motors."""
        self.calibration = calibration_dict

    def _get_half_turn_homings(self, positions: dict[NameOrID, Value]) -> dict[NameOrID, Value]:
        """Get half turn homing offsets."""
        homing_offsets = {}
        for motor_name, position in positions.items():
            motor_id = self._get_motor_id(motor_name)
            motor_model = self._id_to_model(motor_id)
            resolution = self.model_resolution_table[motor_model]
            homing_offsets[motor_name] = int(position - (resolution / 2))
        return homing_offsets

    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 10) -> None:
        """Disable torque for specified motors."""
        motor_names = self._get_motors_list(motors)
        for motor_name in motor_names:
            motor_id = self._get_motor_id(motor_name)
            motor_model = self._id_to_model(motor_id)
            self._disable_torque(motor_id, motor_model, num_retry)

    def _disable_torque(self, motor_id: int, model: str, num_retry: int = 10) -> None:
        """Disable torque for a single motor."""
        self.controller.motor_data[motor_id]["Torque_Enable"] = 0

    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 10) -> None:
        """Enable torque for specified motors."""
        motor_names = self._get_motors_list(motors)
        for motor_name in motor_names:
            motor_id = self._get_motor_id(motor_name)
            motor_model = self._id_to_model(motor_id)
            self._enable_torque(motor_id, motor_model, num_retry)

    def _enable_torque(self, motor_id: int, model: str, num_retry: int = 10) -> None:
        """Enable torque for a single motor."""
        self.controller.motor_data[motor_id]["Torque_Enable"] = 1

    def _encode_sign(self, data_name: str, ids_values: dict[int, int]) -> dict[int, int]:
        """Encode sign for data values."""
        encoded_values = {}
        for motor_id, value in ids_values.items():
            motor_model = self._id_to_model(motor_id)
            encoding = self.model_encoding_table[motor_model].get(data_name, "unsigned")
            if encoding == "sign_magnitude":
                encoded_values[motor_id] = encode_sign_magnitude(value)
            else:
                encoded_values[motor_id] = value
        return encoded_values

    def _decode_sign(self, data_name: str, ids_values: dict[int, int]) -> dict[int, int]:
        """Decode sign for data values."""
        decoded_values = {}
        for motor_id, value in ids_values.items():
            motor_model = self._id_to_model(motor_id)
            encoding = self.model_encoding_table[motor_model].get(data_name, "unsigned")
            if encoding == "sign_magnitude":
                decoded_values[motor_id] = decode_sign_magnitude(value)
            else:
                decoded_values[motor_id] = value
        return decoded_values

    def _split_into_byte_chunks(self, value: int, length: int) -> list[int]:
        """Split value into byte chunks."""
        if length == 1:
            data = [value & 0xFF]
        elif length == 2:
            data = [value & 0xFF, (value >> 8) & 0xFF]
        elif length == 4:
            data = [
                value & 0xFF,
                (value >> 8) & 0xFF,
                (value >> 16) & 0xFF,
                (value >> 24) & 0xFF,
            ]
        return data

    def _broadcast_ping(self) -> tuple[dict[int, int], int]:
        """Broadcast ping to find motors."""
        # Simulate finding motors
        found_motors = {}
        for motor_name, motor in self.motors.items():
            found_motors[motor.id] = self.model_number_table[motor.model]
        return found_motors, 0

    def broadcast_ping(self, num_retry: int = 3, raise_on_error: bool = False) -> dict[int, int] | None:
        """Broadcast ping to find all motors."""
        try:
            found_motors, _ = self._broadcast_ping()
            return found_motors
        except Exception as e:
            if raise_on_error:
                raise
            logger.warning(f"Broadcast ping failed: {e}")
            return None

    def _connect(self, handshake: bool = True) -> None:
        """Connect to the DC motor controller."""
        if not self.controller.is_open:
            self.controller.openPort()
        if handshake:
            self._handshake()

    def _disconnect(self) -> None:
        """Disconnect from the DC motor controller."""
        if self.controller.is_open:
            self.controller.closePort()

    def _read(self, address: int, length: int, motor_id: int, *, num_retry: int = 0, raise_on_error: bool = True, err_msg: str = "") -> tuple[int, int]:
        """Read data from a motor."""
        try:
            # Simulate reading from motor memory
            motor_data = self.controller.motor_data.get(motor_id, {})
            
            # Map address to data name (simplified)
            data_name = self._address_to_data_name(address)
            if data_name in motor_data:
                value = motor_data[data_name]
                # Update potentiometer reading for position
                if data_name == "Present_Position":
                    # Simulate potentiometer reading with some noise
                    import random
                    noise = random.randint(-10, 10)
                    value = max(0, min(4095, value + noise))
                    motor_data[data_name] = value
                return value, 0  # 0 = success
            else:
                return 0, 1  # 1 = error
        except Exception as e:
            if raise_on_error:
                raise RuntimeError(f"Read failed: {e}")
            return 0, 1

    def _write(self, addr: int, length: int, motor_id: int, value: int, *, num_retry: int = 0, raise_on_error: bool = True, err_msg: str = "") -> tuple[int, int]:
        """Write data to a motor."""
        try:
            # Simulate writing to motor memory
            motor_data = self.controller.motor_data.get(motor_id, {})
            
            # Map address to data name (simplified)
            data_name = self._address_to_data_name(addr)
            motor_data[data_name] = value
            
            # Simulate motor movement for position commands
            if data_name == "Goal_Position":
                motor_data["Moving"] = 1
                # Simulate movement over time
                current_pos = motor_data.get("Present_Position", 2048)
                if abs(value - current_pos) > 5:
                    motor_data["Present_Position"] = current_pos + (1 if value > current_pos else -1)
                else:
                    motor_data["Moving"] = 0
                    
            return 0, 0  # 0 = success
        except Exception as e:
            if raise_on_error:
                raise RuntimeError(f"Write failed: {e}")
            return 0, 1

    def _address_to_data_name(self, address: int) -> str:
        """Convert address to data name (simplified mapping)."""
        # This is a simplified mapping - in a real implementation you'd use the control table
        address_mapping = {
            56: "Present_Position",
            42: "Goal_Position",
            40: "Torque_Enable",
            33: "Operating_Mode",
            58: "Present_Velocity",
            46: "Goal_Velocity",
        }
        return address_mapping.get(address, "Unknown")

    def _sync_read(self, addr: int, length: int, motor_ids: list[int], *, num_retry: int = 0, raise_on_error: bool = True, err_msg: str = "") -> tuple[dict[int, int], int]:
        """Synchronously read from multiple motors."""
        results = {}
        for motor_id in motor_ids:
            value, error = self._read(addr, length, motor_id, num_retry=num_retry, raise_on_error=raise_on_error, err_msg=err_msg)
            if error == 0:
                results[motor_id] = value
        return results, 0

    def _sync_write(self, addr: int, length: int, ids_values: dict[int, int], num_retry: int = 0, raise_on_error: bool = True, err_msg: str = "") -> int:
        """Synchronously write to multiple motors."""
        for motor_id, value in ids_values.items():
            _, error = self._write(addr, length, motor_id, value, num_retry=num_retry, raise_on_error=raise_on_error, err_msg=err_msg)
            if error != 0:
                return error
        return 0

    def _setup_sync_reader(self, motor_ids: list[int], addr: int, length: int) -> None:
        """Setup synchronous reader (not needed for DC motors)."""
        pass

    def _setup_sync_writer(self, ids_values: dict[int, int], addr: int, length: int) -> None:
        """Setup synchronous writer (not needed for DC motors)."""
        pass 