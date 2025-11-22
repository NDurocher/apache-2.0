ARG ROS_DISTRO=noetic
ARG FROM_IMAGE=dustynv/ros:noetic-ros-base-l4t-r32.7.1
ARG OVERLAY_WS=/opt/capra/overlay_ws

ARG BADGR_INPUT_TOPIC=/cam0/image_raw
ARG BADGR_OUTPUT_CONTROL_TOPIC=/cmd_vel
ARG BADGR_OUTPUT_IMAGE_TOPIC=/herdr/output_image
ARG BADGR_MODEL_NAME=carla23-04-2022--14:57--from09:34.pth
ARG BADGR_VELOCITY_TOPIC=/herdr/linear_vel_cmd
ARG BADGR_CONTROL_FREQ=5
ARG BADGR_SAMPLE_BATCHES=50
ARG BADGR_PLANNING_HORIZON=10
ARG BADGR_INITIAL_VELOCITY=0.5
ARG BADGR_INITIAL_STEERING_ANGLE=0.0
ARG BADGR_UPDATE_WEIGHTING=20
ARG BADGR_SAMPLE_VELOCITY_VARIANCE=0.3
ARG BADGR_SAMPLE_STEERING_VARIANCE=1.35
ARG BADGR_GOAL_GAIN=0.25
ARG BADGR_ACTION_GAIN=0.2
ARG BADGR_WHEEL_BASE=1.0

# MULTI-STAGE FOR CACHING
FROM $FROM_IMAGE AS cacher

ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./capra-ros-badgr/ capra-badgr/

# Copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" | \
    xargs cp --parents -t /tmp/opt || true

# MULTI-STAGE FOR BUILDING
FROM $FROM_IMAGE AS builder
ARG DEBIAN_FRONTEND=noninteractive

ARG OVERLAY_WS
ARG ROS_DISTRO
WORKDIR $OVERLAY_WS


# Fix expired ROS GPG key
RUN apt-get update || true && \
    apt-get install -y curl gnupg2 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros-latest.list > /dev/null && \
    rm -rf /var/lib/apt/lists/*

# Build geometry_msgs and sensor_msgs from source with Python 3 support
RUN mkdir -p /tmp/msgs_ws && \
    cd /tmp/msgs_ws && \
    rosinstall_generator geometry_msgs sensor_msgs --rosdistro noetic --deps --tar > msgs.rosinstall && \
    mkdir src && \
    vcs import --input msgs.rosinstall ./src && \
    apt-get update && \
    rosdep install -y \
      --from-paths ./src \
      --ignore-packages-from-source \
      --rosdistro noetic && \
    python3 ./src/catkin/bin/catkin_make_isolated --install --install-space /opt/ros/noetic -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF && \
    cd / && \
    rm -rf /tmp/msgs_ws && \
    rm -rf /var/lib/apt/lists/*

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ccache \
    lcov \
    git \
    net-tools \
    iputils-ping \
    python3-pip \
    libeigen3-dev \
    libnlopt-dev \
    build-essential \
    unzip \
    g++ \
    nano \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install TensorRT Python bindings (should already be in JetPack)
# Verify with: python3 -c "import tensorrt; print(tensorrt.__version__)"
#RUN apt-get update && apt-get install -y --no-install-recommends \
#    python3-libnvinfer \
#    python3-libnvinfer-dev \
#    && rm -rf /var/lib/apt/lists/*

# Install CPU-only PyTorch and other Python deps
RUN pip3 install --upgrade pip && \
    pip3 install \
    torch \
    torchvision \
    #opencv-python-headless \
    matplotlib \
    pycuda

# Copy workspace and install rosdep dependencies
COPY --from=cacher ${OVERLAY_WS} ./

RUN apt-get update && \
    rosdep update && \
    rosdep install -q -y \
    --from-paths src \
    --rosdistro=${ROS_DISTRO} \
    --ignore-src || true && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make --pkg capra_ros_badgr && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

# RUNTIME STAGE
FROM builder AS runner

ARG OVERLAY_WS
ARG BADGR_INPUT_TOPIC
ARG BADGR_OUTPUT_CONTROL_TOPIC
ARG BADGR_OUTPUT_IMAGE_TOPIC
ARG BADGR_MODEL_NAME
ARG BADGR_VELOCITY_TOPIC
ARG BADGR_CONTROL_FREQ
ARG BADGR_SAMPLE_BATCHES
ARG BADGR_PLANNING_HORIZON
ARG BADGR_INITIAL_VELOCITY
ARG BADGR_INITIAL_STEERING_ANGLE
ARG BADGR_UPDATE_WEIGHTING
ARG BADGR_SAMPLE_VELOCITY_VARIANCE
ARG BADGR_SAMPLE_STEERING_VARIANCE
ARG BADGR_GOAL_GAIN
ARG BADGR_ACTION_GAIN
ARG BADGR_WHEEL_BASE

ENV PYTHONPATH="/opt/ros/noetic/lib/python3.6/site-packages:${PYTHONPATH}"
ENV BADGR_INPUT_TOPIC=$BADGR_INPUT_TOPIC \
    BADGR_OUTPUT_CONTROL_TOPIC=$BADGR_OUTPUT_CONTROL_TOPIC \
    BADGR_OUTPUT_IMAGE_TOPIC=$BADGR_OUTPUT_IMAGE_TOPIC \
    BADGR_MODEL_NAME=$BADGR_MODEL_NAME \
    BADGR_VELOCITY_TOPIC=$BADGR_VELOCITY_TOPIC \
    BADGR_CONTROL_FREQ=$BADGR_CONTROL_FREQ \
    BADGR_SAMPLE_BATCHES=$BADGR_SAMPLE_BATCHES \
    BADGR_PLANNING_HORIZON=$BADGR_PLANNING_HORIZON \
    BADGR_INITIAL_VELOCITY=$BADGR_INITIAL_VELOCITY \
    BADGR_INITIAL_STEERING_ANGLE=$BADGR_INITIAL_STEERING_ANGLE \
    BADGR_UPDATE_WEIGHTING=$BADGR_UPDATE_WEIGHTING \
    BADGR_SAMPLE_VELOCITY_VARIANCE=$BADGR_SAMPLE_VELOCITY_VARIANCE \
    BADGR_SAMPLE_STEERING_VARIANCE=$BADGR_SAMPLE_STEERING_VARIANCE \
    BADGR_GOAL_GAIN=$BADGR_GOAL_GAIN \
    BADGR_ACTION_GAIN=$BADGR_ACTION_GAIN \
    BADGR_WHEEL_BASE=$BADGR_WHEEL_BASE

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sed --in-place \
    "s|^source .*|source '${OVERLAY_WS}/devel/setup.bash'|" \
    /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["roslaunch", "capra_ros_badgr", "badgr.launch", "--wait"]
