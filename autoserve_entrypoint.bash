#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source "/colcon_ws/install/setup.bash"
exec "$@"