#!/bin/bash

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "${WORKSPACE}/install/setup.bash" --

# execute docker command
exec "$@"
