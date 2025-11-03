FROM nvcr.io/nvidia/l4t-base:r32.7.1

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list

# Install minimal Python and ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python \
    python-pip \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    ros-melodic-ros-base \
    ros-melodic-geometry-msgs \
    ros-melodic-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install gamepad support
RUN apt-get update && apt-get install -y \
    build-essential \
    python-dev \
    && rm -rf /var/lib/apt/lists/*
#    libsdl1.2-dev \
#    libsdl-image1.2-dev \
#    libsdl-mixer1.2-dev \
#    libsdl-ttf2.0-dev \
#    libfreetype6-dev \
#    libportmidi-dev \

RUN pip install numpy smbus

# Initialize rosdep
RUN rosdep init && rosdep update

# Copy your script
COPY pub_imu.py /app/pub_imu.py
COPY mpu6050.py /app/mpu6050.py

# Copy entrypoint
COPY imu_entrypoint.sh /app/imu_entrypoint.sh
RUN chmod +x /app/imu_entrypoint.sh

# Setup environment
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /app

# Set Python 2 as default since ROS Melodic uses Python 2
RUN update-alternatives --install /usr/bin/python python /usr/bin/python2 1

ENV DEBIAN_FRONTEND=dialog

# This container needs privileged access for GPIO
CMD ["./imu_entrypoint.sh"]
