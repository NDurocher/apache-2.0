# Use L4T 32.7.1 base image
FROM nvcr.io/nvidia/l4t-base:r32.7.1

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install ROS Melodic (for Ubuntu 18.04)
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list

RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base \
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    build-essential \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    python-gst-1.0 \
    python-pip \
    python-yaml \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Create ROS workspace
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

# Copy your ROS package
COPY ros_pkg src/ros_pkg

# Build the workspace
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make"

# Setup environment
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

COPY docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh

# Create the ROS package and node
#COPY src/ros_pkg/scripts ./ros-gstreamer-node.py src/ros_pkg/scripts/
RUN chmod +x src/ros_pkg/scripts/ros-gstreamer-node.py

# Set Python 2 as default since ROS Melodic uses Python 2
RUN update-alternatives --install /usr/bin/python python /usr/bin/python2 1 

ENV DEBIAN_FRONTEND=dialog

ENTRYPOINT ["/docker-entrypoint.sh"]
