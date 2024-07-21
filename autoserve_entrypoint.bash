#!/bin/bash
set -e

# setup ros2 environment
export GAZEBO_MODEL_PATH=/root/.gazebo/models/
source /opt/ros/humble/setup.bash
source /usr/share/gazebo-11/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /colcon_ws/install/setup.bash
exec "$@"