#!/bin/bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash
echo "Sourced ROS 2 Humble"

# Execute the command passed into this entrypoint
exec "$@"
