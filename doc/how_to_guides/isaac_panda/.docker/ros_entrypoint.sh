#!/bin/bash

# Source ROS and the MoveIt 2 workspace
source /opt/ros/humble/setup.bash
source /root/ws_moveit/install/setup.bash
echo "Sourced ROS & MoveIt Humble"

# Execute the command passed into this entrypoint
exec "$@"
