#!/bin/bash

# Permission has been automatically added from git:
# git update-index --chmod=+x --add start/start_robot.sh
# git add start/start_robot.sh
# git commit -m "Add robot start script with executable permissions"

# Activate the virtual environment
source .venv/bin/activate

# Run the robot control software
python lerobot/scripts/control_robot.py \
  --robot.type=sourccey_v1beta \
  --control.type=remote_robot
