#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Uncomment for debugging
# set -x
# env

cd /colcon_ws

. /opt/ros/humble/setup.sh
. install/setup.bash
colcon build \
    --symlink-install \
    --executor sequential
