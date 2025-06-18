# Use ROS2 Jazzy base image
FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    i2c-tools \
    libi2c-dev \
    python3-smbus \
    python3-numpy \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
#RUN pip3 install \
#    mpu6050-raspberrypi \
#    numpy

# Create workspace
RUN mkdir -p /ros2_ws/src/imu_mpu6050

# Create package structure
RUN mkdir -p /ros2_ws/src/imu_mpu6050/imu_mpu6050 \
    /ros2_ws/src/imu_mpu6050/launch \
    /ros2_ws/src/imu_mpu6050/config \
    /ros2_ws/src/imu_mpu6050/resource

# Copy package files
COPY package.xml /ros2_ws/src/imu_mpu6050/
COPY setup.py /ros2_ws/src/imu_mpu6050/
COPY __init__.py /ros2_ws/src/imu_mpu6050/imu_mpu6050/
COPY mpu6050.py /ros2_ws/src/imu_mpu6050/imu_mpu6050/
COPY imu_publisher.py /ros2_ws/src/imu_mpu6050/imu_mpu6050/
COPY imu_launch.py /ros2_ws/src/imu_mpu6050/launch/
COPY imu_params.yaml /ros2_ws/src/imu_mpu6050/config/

# Create resource file
RUN touch /ros2_ws/src/imu_mpu6050/resource/imu_mpu6050

# Set working directory
WORKDIR /ros2_ws

# Source ROS2 and build the package
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select imu_mpu6050 --symlink-install"

# After building, create the expected directory and symlink
RUN mkdir -p /ros2_ws/install/imu_mpu6050/lib/imu_mpu6050/
RUN ln -sf ../../bin/imu_publisher /ros2_ws/install/imu_mpu6050/lib/imu_mpu6050/imu_publisher

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS2 setup\n\
source /opt/ros/jazzy/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
\n\
# Execute the command\n\
exec "$@"' > /ros2_entrypoint.sh && \
    chmod +x /ros2_entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/ros2_entrypoint.sh"]

# Default command
CMD ["ros2", "launch", "imu_mpu6050", "imu_launch.py"]

# Expose I2C device (will need --device flag when running)
# docker run --device=/dev/i2c-1 your_image_name
