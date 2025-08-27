#!/bin/bash

# This script provides a convenient way to run the cmd_vel_forwarder node
# with the correct ROS 2 and Python virtual environments sourced.

# Source the ROS 2 workspace
source /home/tony/dev/robotics/shared_ros2/install/setup.bash

# Source the Python virtual environment
source /home/tony/dev/robotics/shared_ros2/src/ros1_cmd_vel_bridge/.venv/bin/activate

# Launch the node, passing along any additional arguments
ros2 launch ros1_cmd_vel_bridge cmd_vel_forwarder.launch.py "$@"