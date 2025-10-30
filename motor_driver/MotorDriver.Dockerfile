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

# Install Jetson.GPIO
RUN pip install "Jetson.GPIO==2.0.17"

# Initialize rosdep
RUN rosdep init && rosdep update

# Copy your script
COPY motor_control.py /app/motor_control.py

# Copy entrypoint
COPY motor_driver_entrypoint.sh /app/motor_driver_entrypoint.sh
RUN chmod +x /app/motor_driver_entrypoint.sh

# Setup environment
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /app

# Set Python 2 as default since ROS Melodic uses Python 2
RUN update-alternatives --install /usr/bin/python python /usr/bin/python2 1 

ENV DEBIAN_FRONTEND=dialog

# This container needs privileged access for GPIO
#CMD ["source", "/opt/ros/melodic/setup.bash", "&&", "python", "motor_control.py"]
CMD ["./motor_driver_entrypoint.sh"]
