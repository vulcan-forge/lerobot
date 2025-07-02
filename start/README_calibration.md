# 2-Motor Calibration Script

This script calibrates a 2-motor leader-follower setup where:
- **Motor ID 1** is the leader (teleop/input device)
- **Motor ID 2** is the follower (robot/actuator)

## Features

- **Automatic motor type detection**: Detects if motors are single or double shaft
- **Adaptive calibration ranges**: 
  - Single shaft: Up to 1 rotation (360°)
  - Double shaft: Up to 7 rotations (2520°)
- **Proportional movement**: Follower motor moves proportionally to leader motor
- **Interactive calibration**: Guides you through the calibration process

## Usage

### Basic Usage

```bash
# Windows
python start/calibrate_2_motors.py \
    --port=COM3 \
    --leader_model=sts3235 \
    --follower_model=sts3215

# Linux/Mac
python start/calibrate_2_motors.py \
    --port=/dev/tty.usbmodem58760431551 \
    --leader_model=sts3235 \
    --follower_model=sts3215
```

### Advanced Usage

```bash
# Windows
python start/calibrate_2_motors.py \
    --port=COM3 \
    --leader_model=sts3235 \
    --follower_model=sts3215 \
    --motor_type=feetech \
    --leader_id=1 \
    --follower_id=2 \
    --baudrate=115200

# Linux/Mac
python start/calibrate_2_motors.py \
    --port=/dev/tty.usbmodem58760431551 \
    --leader_model=sts3235 \
    --follower_model=sts3215 \
    --motor_type=feetech \
    --leader_id=1 \
    --follower_id=2 \
    --baudrate=115200
```

## Parameters

- `--port`: Serial port for motor communication
- `--leader_model`: Model of the leader motor (e.g., "sts3235" for multi-shaft, "sts3215" for single-shaft)
- `--follower_model`: Model of the follower motor (e.g., "sts3215")
- `--motor_type`: Motor type ("dynamixel" or "feetech", default: "feetech")
- `--leader_id`: Leader motor ID (default: 1)
- `--follower_id`: Follower motor ID (default: 2)
- `--baudrate`: Communication baudrate (default: 115200)

## Calibration Process

1. **Connection**: Script connects to both motors
2. **Motor Detection**: Automatically detects single vs double shaft motors
3. **Home Position**: Moves both motors to 0% position
4. **Minimum Recording**: You manually move leader to minimum position
5. **Maximum Recording**: You manually move leader to maximum position
6. **Proportional Testing**: Tests proportional movement at 25%, 50%, 75%
7. **Calibration Save**: Saves calibration data to motors

## Proportional Movement

The follower motor moves proportionally to the leader motor:

- If leader moves 30% of its range → follower moves 30% of its range
- Works even if motors have different ranges of motion
- Handles single vs double shaft differences automatically

## Finding Your Port

To find the correct serial port:

```bash
python -m lerobot.find_port.py
```

This will show available ports and help you identify the correct one.

## Example Output

```
Starting 2-motor calibration
Connecting to motors...
Successfully connected to motors
Torque enabled on both motors
Detecting motor types...
Leader motor (ID 1): double shaft
Follower motor (ID 2): single shaft
Leader motor range: 2520 degrees
Follower motor range: 360 degrees
Moving to home position...
Recording minimum positions...
Move leader motor to minimum position and press Enter...
Leader minimum: 5.23%
Follower minimum: 2.15%
Recording maximum positions...
Move leader motor to maximum position and press Enter...
Leader maximum: 94.78%
Follower maximum: 97.85%
Testing proportional movement...
Testing 25% position...
  Leader: 25.00% -> Follower: 25.00%
Testing 50% position...
  Leader: 50.00% -> Follower: 50.00%
Testing 75% position...
  Leader: 75.00% -> Follower: 75.00%
Returning to home position...
Saving calibration...
Calibration saved successfully!
Calibration summary:
  Leader motor (ID 1): double shaft, range 5.2% - 94.8%
  Follower motor (ID 2): single shaft, range 2.2% - 97.9%
  Proportional movement: follower moves proportionally to leader
Disconnecting...
```

## Troubleshooting

### Import Errors
If you get import errors, make sure you're running from the project root:
```bash
cd /path/to/lerobot
python start/calibrate_2_motors.py --port=...
```

### Connection Issues
- Check that the port is correct
- Ensure motors are powered and connected
- Try different baudrates if needed

### Motor Type Detection
If motor type detection fails, the script defaults to "single shaft" for safety.
You can manually verify motor types in the motor documentation. 