# Flexible Dual SO100 Calibration and Teleop

This directory contains scripts for calibrating and teleoperating dual SO100 robotic arms with flexible motor ID ranges. Unlike the standard LeRobot calibration which expects motor IDs 1-6, these scripts support both 1-6 and 7-12 motor ID ranges.

## Files

- `flexible_calibrate.py` - Flexible calibration script that asks for port and motor ID range for each arm
- `dual_teleop_flexible.py` - Flexible dual teleop script that supports different motor ID ranges
- `dual_record_flexible.py` - Flexible dual recording script for collecting training data
- `test_calibration_loading.py` - Test script to verify calibration file loading
- `README_flexible.md` - This file

## Setup Instructions

### 1. Calibration

Run the flexible calibration script:

```bash
cd lerobot/common/robots/so100_Double_follower
python flexible_calibrate.py
```

The script will:
- Ask how many arms you want to calibrate (1-4)
- For each arm, ask for:
  - COM port (e.g., COM3, /dev/ttyACM0)
  - Motor ID range (1-6 or 7-12)
- Calibrate each arm individually
- Save calibration files to `~/.lerobot/calibrations/`

### 2. Teleop

Run the flexible teleop script:

```bash
python dual_teleop_flexible.py
```

The script will:
- Ask for configuration for each arm (port and motor ID range)
- Load the corresponding calibration files
- Connect to all arms
- Start teleop (leader arms control follower arms)

### 3. Recording Training Data

Run the flexible recording script:

```bash
python dual_record_flexible.py --interactive
```

Or with a configuration file:

```bash
python dual_record_flexible.py --config recording_config.json
```

The script will:
- Ask for configuration for each arm (port and motor ID range)
- Ask for dataset configuration (repo ID, task description, number of episodes, etc.)
- Load the corresponding calibration files
- Connect to all arms
- Record training data from both leader and follower arms
- Save the dataset locally and optionally push to Hugging Face Hub

## Calibration File Structure

The flexible calibration system creates individual calibration files for each arm:

```
~/.lerobot/calibrations/
├── left_follower_calibration.json
├── right_follower_calibration.json
├── left_leader_calibration.json
├── right_leader_calibration.json
└── dual_so100_combined_calibration.json
```

Each calibration file contains:
- Motor calibration data (ID, drive mode, homing offset, range min/max)
- Metadata (port, motor IDs, ID range, arm name)

## Example Configuration

For a typical dual-arm setup:

- **Left Follower**: COM3, motor IDs 1-6
- **Right Follower**: COM4, motor IDs 7-12
- **Left Leader**: COM1, motor IDs 1-6
- **Right Leader**: COM2, motor IDs 7-12

## Recording Configuration Example

```json
{
  "arms": {
    "left_follower": {
      "port": "COM3",
      "motor_ids": [1, 2, 3, 4, 5, 6],
      "id_range": "1-6"
    },
    "right_follower": {
      "port": "COM4",
      "motor_ids": [7, 8, 9, 10, 11, 12],
      "id_range": "7-12"
    },
    "left_leader": {
      "port": "COM1",
      "motor_ids": [1, 2, 3, 4, 5, 6],
      "id_range": "1-6"
    },
    "right_leader": {
      "port": "COM2",
      "motor_ids": [7, 8, 9, 10, 11, 12],
      "id_range": "7-12"
    }
  },
  "dataset": {
    "repo_id": "username/dual_so100_dataset",
    "single_task": "Pick up objects with both arms",
    "num_episodes": 10,
    "episode_time_s": 60,
    "reset_time_s": 10,
    "push_to_hub": true
  },
  "fps": 30,
  "display_data": false
}
```

## Testing

Use the test script to verify calibration file loading:

```bash
python test_calibration_loading.py
```

This will:
- Test loading existing calibration files
- Verify MotorCalibration object creation
- Create sample calibration files for testing

## Troubleshooting

### Motor ID Issues
- Ensure each arm has unique motor IDs
- Left arm typically uses 1-6, right arm uses 7-12
- Check motor connections and firmware

### Port Issues
- Verify COM ports are correct
- Check USB connections
- Ensure no other programs are using the ports

### Calibration Issues
- Run calibration for each arm individually if needed
- Check that calibration files are saved correctly
- Use the test script to verify file loading

### Recording Issues
- Ensure all arms are properly calibrated before recording
- Check that dataset repository ID is valid
- Verify network connection if pushing to Hugging Face Hub

## Differences from Standard LeRobot

The standard LeRobot calibration expects:
- Motor IDs 1-6 only
- Single calibration file per robot
- Fixed port configuration

The flexible system supports:
- Motor IDs 1-6 or 7-12
- Individual calibration files per arm
- User-specified ports and motor ID ranges
- Metadata for easy identification
- Dual-arm recording for training data collection

## Usage Tips

1. **Calibrate one arm at a time** to avoid confusion
2. **Use consistent naming** (left_follower, right_follower, etc.)
3. **Test calibration loading** before running teleop or recording
4. **Keep track of your configuration** (ports and motor IDs)
5. **Backup calibration files** if needed
6. **Start with short recording sessions** to test the setup
7. **Use descriptive task descriptions** for better dataset organization

## File Locations

- Calibration files: `~/.lerobot/calibrations/`
- Scripts: `lerobot/common/robots/so100_Double_follower/`
- Test files: Same directory as scripts
- Recorded datasets: Local storage and optionally Hugging Face Hub 