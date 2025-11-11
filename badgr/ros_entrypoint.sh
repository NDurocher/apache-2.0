#!/bin/bash
set -e 
# set Carla python package pythonpath 
export PYTHONPATH=$PYTHONPATH:/opt/capra/overlay_ws/src/capra-badgr/capra-ros-badgr/carla-0.9.12-py3.7-linux-x86_64.egg
# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
chmod +x /opt/capra/overlay_ws/src/capra-badgr/scripts/Real_Life_Test.py
exec "$@"