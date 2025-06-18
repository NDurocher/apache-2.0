FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:0

# Install only essential dependencies
RUN apt-get update && apt-get install -y \
    # Basic GUI support
    x11-apps \
    # Development tools
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Setup ROS 2 environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# Copy source code if it exists (this will be overridden by volume mount)
#COPY src/ /ros2_ws/src/ 2>/dev/null || true

# Build workspace if packages exist
#RUN if [ "$(ls -A /ros2_ws/src 2>/dev/null)" ]; then \
#        . /opt/ros/jazzy/setup.bash && \
#        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; \
#    fi

# Create entrypoint script that handles workspace setup
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS 2\n\
source /opt/ros/jazzy/setup.bash\n\
\n\
# Check if workspace needs to be built\n\
if [ -d "/ros2_ws/src" ] && [ "$(ls -A /ros2_ws/src 2>/dev/null)" ]; then\n\
    echo "Found packages in workspace..."\n\
    cd /ros2_ws\n\
    \n\
    # Build if install directory doesnt exist or is older than src\n\
    if [ ! -d "install" ] || [ "src" -nt "install" ]; then\n\
        echo "Building workspace..."\n\
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release\n\
        echo "Workspace built successfully"\n\
    fi\n\
    \n\
    # Source workspace\n\
    if [ -f "/ros2_ws/install/setup.bash" ]; then\n\
        source /ros2_ws/install/setup.bash\n\
    fi\n\
fi\n\
\n\
exec "$@"' > /ros_entrypoint.sh && \
chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
