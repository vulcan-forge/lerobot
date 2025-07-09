# DC Motor with Potentiometer Encoding

This module provides a DC motor implementation that can be used as a drop-in replacement for Feetech motors in the LeRobot framework. It uses analog potentiometer feedback instead of digital encoders.

## Overview

The DC motor implementation (`DCMotorsBus`) follows the same interface as `FeetechMotorsBus`, making it easy to replace Feetech motors with DC motors that have potentiometer position feedback.

## Features

- **Drop-in replacement**: Uses the same API as Feetech motors
- **Analog feedback**: Reads position from potentiometer (0-4095 range)
- **Position control**: Supports position servo mode
- **Calibration**: Supports motor calibration and range of motion recording
- **Normalization**: Supports normalized position values (degrees, 0-100, -100 to 100)

## Hardware Requirements

- DC motor with gearbox
- Potentiometer (10kΩ linear recommended)
- Motor driver (H-bridge or similar)
- Microcontroller (Arduino, ESP32, etc.) to read potentiometer and control motor
- Serial communication interface

## Usage

### Basic Setup

```python
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dc_motor import DCMotorsBus

# Define your DC motor
motors = {
    "shoulder_lift": Motor(2, "shoulder_lift_dc", MotorNormMode.DEGREES),
}

# Create the motor bus
bus = DCMotorsBus(
    port="/dev/ttyUSB0",  # Your serial port
    motors=motors,
    calibration=None,
)

# Connect and configure
bus.connect()
bus.configure_motors()
bus.enable_torque()
```

### Reading Position

```python
# Read current position
position = bus.read("Present_Position", "shoulder_lift", normalize=True)
print(f"Position: {position:.2f} degrees")

# Sync read for multiple motors
positions = bus.sync_read("Present_Position", normalize=True)
```

### Controlling Position

```python
# Move to a specific position
bus.write("Goal_Position", "shoulder_lift", 45.0, normalize=True)

# Move to center
bus.write("Goal_Position", "shoulder_lift", 0.0, normalize=True)
```

### Calibration

```python
# Calibrate the motor
bus.disable_torque()

# Set homing offsets
homing_offsets = bus.set_half_turn_homings()

# Record range of motion
range_mins, range_maxes = bus.record_ranges_of_motion()

# Save calibration
calibration = bus.read_calibration()
```

## Integration with Existing Robots

To replace a Feetech motor with a DC motor in an existing robot configuration:

### 1. Update Robot Configuration

```python
# Instead of:
motors = {
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
}

# Use:
motors = {
    "shoulder_lift": Motor(2, "shoulder_lift_dc", MotorNormMode.DEGREES),
}
```

### 2. Update Motor Bus Type

```python
# Instead of:
from lerobot.common.motors.feetech import FeetechMotorsBus
bus = FeetechMotorsBus(port=port, motors=motors, calibration=calibration)

# Use:
from lerobot.common.motors.dc_motor import DCMotorsBus
bus = DCMotorsBus(port=port, motors=motors, calibration=calibration)
```

## Hardware Implementation

### Microcontroller Code Example (Arduino)

```cpp
#include <Servo.h>

// Pin definitions
const int MOTOR_PIN1 = 9;
const int MOTOR_PIN2 = 10;
const int POT_PIN = A0;

// Motor control
int motorSpeed = 0;
int targetPosition = 2048;  // Center position
int currentPosition = 2048;

void setup() {
  Serial.begin(115200);
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
}

void loop() {
  // Read potentiometer
  currentPosition = analogRead(POT_PIN);
  
  // Simple proportional control
  int error = targetPosition - currentPosition;
  motorSpeed = constrain(error / 10, -255, 255);
  
  // Drive motor
  if (motorSpeed > 0) {
    analogWrite(MOTOR_PIN1, motorSpeed);
    analogWrite(MOTOR_PIN2, 0);
  } else {
    analogWrite(MOTOR_PIN1, 0);
    analogWrite(MOTOR_PIN2, -motorSpeed);
  }
  
  // Send position data over serial
  Serial.println(currentPosition);
  
  delay(10);
}
```

### Communication Protocol

The DC motor controller should implement a simple serial protocol that responds to commands similar to Feetech motors:

- **Read commands**: Respond with current position, velocity, etc.
- **Write commands**: Accept target position and update motor control
- **Status commands**: Return motor status and error codes

## Configuration

### Motor Models

Available motor models:
- `"dc_motor"`: Generic DC motor
- `"dc_motor_pot"`: DC motor with potentiometer
- `"shoulder_lift_dc"`: Specific model for shoulder lift applications

### Control Table

The DC motor uses the same control table structure as Feetech motors:

- `Present_Position` (address 56): Current position from potentiometer
- `Goal_Position` (address 42): Target position
- `Torque_Enable` (address 40): Enable/disable motor
- `Operating_Mode` (address 33): Control mode (position, velocity, etc.)

### Normalization Modes

- `MotorNormMode.DEGREES`: Position in degrees (-180 to 180)
- `MotorNormMode.RANGE_0_100`: Position as percentage (0 to 100)
- `MotorNormMode.RANGE_M100_100`: Position as percentage (-100 to 100)

## Limitations

1. **Lower precision**: Potentiometer feedback is less precise than digital encoders
2. **Analog noise**: Potentiometer readings may have noise and drift
3. **Limited speed control**: Basic implementation focuses on position control
4. **No feedback**: No current or temperature feedback (simulated)

## Troubleshooting

### Common Issues

1. **Serial connection fails**: Check port name and permissions
2. **Position readings unstable**: Check potentiometer wiring and add filtering
3. **Motor doesn't move**: Check motor driver connections and power supply
4. **Calibration fails**: Ensure motor can move through full range

### Debug Tips

- Use the example script to test basic functionality
- Check serial communication with a terminal program
- Verify potentiometer readings with a multimeter
- Test motor control independently before integration

## Example

See `examples/dc_motor_example.py` for a complete working example. 