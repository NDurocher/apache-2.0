#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source /ros_ws/devel/setup.bash

#if ! pgrep -x "roscore" > /dev/null
#then
#    roscore &
#    sleep 3
#fi

if [ $# -eq 0 ]
then
   echo "running default ROS node..."
   #rosrun ros_pkg ros-gstreamer-node.py
   python src/ros_pkg/scripts/ros-gstreamer-node.py
else
   echo "Running custom command: $@"
   exec "$@"
fi
