#!/bin/bash

# Permission has been automatically added from git:
# git update-index --chmod=+x --add start/start_robot.sh
# git add start/start_robot.sh
# git commit -m "Add robot start script with executable permissions"

# Kill any existing robot control processes
echo "Stopping any existing robot servers..."
pkill -f "control_robot.py" || true

# Activate the virtual environment
source .venv/bin/activate

# Run the robot control software
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=remote_robot
