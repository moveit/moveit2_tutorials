#!/bin/bash

set -e

# Source ROS underlay and MoveIt workspace dynamically
if [ -n "${ROS_DISTRO}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
elif [ -f "/opt/ros/rolling/setup.bash" ]; then
    source "/opt/ros/rolling/setup.bash"
fi

if [ -f "/root/ws_moveit/install/setup.bash" ]; then
    source "/root/ws_moveit/install/setup.bash"
fi

echo "Sourced ROS (${ROS_DISTRO:-unknown}) & MoveIt"

# Execute the command passed into this entrypoint
exec "$@"
