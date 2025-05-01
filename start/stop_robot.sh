#!/bin/bash

# Permission has been automatically added from git:
# git update-index --chmod=+x --add start/stop_robot.sh
# git add start/stop_robot.sh
# git commit -m "Add robot stop script with executable permissions"

# Kill any existing robot control processes
echo "Stopping any existing robot servers..."
pkill -f "control_robot.py" || true
