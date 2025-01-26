sudo docker run --runtime nvidia --network host \
--device=/dev/video0 \
-v /tmp/argus_socket:/tmp/argus_socket \
jetson-ros-gstreamer \ rosrun ros_pkg ros-gstreamer-node.py
