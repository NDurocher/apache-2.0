ARG ROS_DISTRO=noetic
#ARG FROM_IMAGE=nvidia/cuda:11.4.2-cudnn8-runtime-ubuntu20.04 #nvcc doesnt work on this one
#ARG FROM_IMAGE=nvidia/cuda:11.4.2-cudnn8-devel-ubuntu20.04
ARG FROM_IMAGE=ros:$ROS_DISTRO-ros-base
ARG OVERLAY_WS=/opt/capra/overlay_ws
ARG ROS_SETUP=/opt/ros/$ROS_DISTRO/setup.sh

ARG BADGR_INPUT_TOPIC=/cam0/image_raw
ARG BADGR_OUTPUT_CONTROL_TOPIC=/cmd_vel
ARG BADGR_OUTPUT_IMAGE_TOPIC=/herdr/output_image
ARG BADGR_MODEL_NAME=carla23-04-2022--14:57--from09:34.pth
ARG BADGR_VELOCITY_TOPIC=/herdr/linear_vel_cmd
ARG BADGR_CONTROL_FREQ=15.
ARG BADGR_SAMPLE_BATCHES=25.
ARG BADGR_PLANNING_HORIZON=4
ARG BADGR_INITIAL_VELOCITY=1.5
ARG BADGR_INITIAL_STEERING_ANGLE=0.0
ARG BADGR_UPDATE_WEIGHTING=20
ARG BADGR_SAMPLE_VELOCITY_VARIANCE=0.3
ARG BADGR_SAMPLE_STEERING_VARIANCE=1.35
ARG BADGR_GOAL_GAIN=0.25
ARG BADGR_ACTION_GAIN=0.2
ARG BADGR_WHEEL_BASE=1.0

# MULTI-STAGE FOR CACHING
FROM $FROM_IMAGE AS cacher

# copy overlay source   
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./capra-ros-badgr/ capra-badgr/
# # COPY ./capra/ capra/
# COPY ./thirdparty/ thirdparty/

# copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
  find ./ -name "package.xml" | \
  xargs cp --parents -t /tmp/opt \
  || true

# MULTI-STAGE FOR BUILDING
FROM $FROM_IMAGE AS builder
ARG DEBIAN_FRONTEND=noninteractive

# install overlay dependencies
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
ARG ROS_DISTRO
# install ros
#RUN apt-get update && apt-get install -y lsb-release && apt-get clean all \
# && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
# && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && apt-get update && apt-get install  -q -y --no-install-recommends ros-$ROS_DISTRO-desktop-full && apt-get install  -q -y --no-install-recommends python3-rosdep && rosdep init && rosdep update

# install CI dependencies
ARG ROS_DISTRO
RUN apt-get update && apt-get install -q -y --no-install-recommends\
  ccache \
  lcov \
  git \
  python \
  net-tools \
  iputils-ping \
  python3-pip \
  python-numpy \
  python-yaml \
  libeigen3-dev \
  libnlopt-dev \
  build-essential \
  unzip \
  g++ \
  nano \
  ros-$ROS_DISTRO-vision-msgs \
  ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-tf-conversions \
  ros-$ROS_DISTRO-nlopt \
  ros-$ROS_DISTRO-actionlib-msgs \
  && rm -rf /var/lib/apt/lists/*
  
#RUN pip install scikit-learn typing numpy 
#RUN pip3 install scikit-learn typing numpy 
RUN pip3 install torch==1.11.0 torchvision==0.12.0 torchaudio==0.11.0
RUN pip3 install --upgrade numpy
RUN pip3 install opencv-python-headless matplotlib
# RUN pip3 install h5py
# tensorflow-gpu

# install tensorflow cc
#RUN apt-get update && apt-get install -q -y --no-install-recommends cmake curl g++-7 git python3-dev python3-numpy sudo wget


# RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel.gpg && \
# mv bazel.gpg /etc/apt/trusted.gpg.d/ && \
# echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list

#RUN apt-get update && apt-get install -q -y --no-install-recommends bazel bazel-3.7.2


ARG ROS_SETUP
ARG ROS_DISTRO
RUN . $ROS_SETUP && \
  apt-get update && \
  rosdep update && \
  rosdep install -q -y \
  --from-paths src \
  --rosdistro=$ROS_DISTRO \
  --ignore-src ; exit 0\
  && rm -rf /var/lib/apt/lists/*


#ENV LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
COPY --from=cacher $OVERLAY_WS ./
RUN rm /usr/bin/python
RUN ln -s /usr/bin/python3 /usr/bin/python

ARG OVERLAY_MIXINS="release ccache"
RUN . $ROS_SETUP && catkin_make --pkg capra_ros_badgr && catkin_make -DCMAKE_BUILD_TYPE=Release


FROM builder as runner

ARG ROS_SETUP
ARG WORKSPACE
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

ENV BADGR_INPUT_TOPIC $BADGR_INPUT_TOPIC
ENV BADGR_OUTPUT_CONTROL_TOPIC $BADGR_OUTPUT_CONTROL_TOPIC
ENV BADGR_OUTPUT_IMAGE_TOPIC $BADGR_OUTPUT_IMAGE_TOPIC
ENV BADGR_MODEL_NAME $BADGR_MODEL_NAME
ENV BADGR_VELOCITY_TOPIC $BADGR_VELOCITY_TOPIC
ENV BADGR_CONTROL_FREQ $BADGR_CONTROL_FREQ
ENV BADGR_SAMPLE_BATCHES $BADGR_SAMPLE_BATCHES
ENV BADGR_PLANNING_HORIZON $BADGR_PLANNING_HORIZON
ENV BADGR_INITIAL_VELOCITY $BADGR_INITIAL_VELOCITY
ENV BADGR_INITIAL_STEERING_ANGLE $BADGR_INITIAL_STEERING_ANGLE
ENV BADGR_UPDATE_WEIGHTING $BADGR_UPDATE_WEIGHTING
ENV BADGR_SAMPLE_VELOCITY_VARIANCE $BADGR_SAMPLE_VELOCITY_VARIANCE
ENV BADGR_SAMPLE_STEERING_VARIANCE $BADGR_SAMPLE_STEERING_VARIANCE
ENV BADGR_GOAL_GAIN $BADGR_GOAL_GAIN
ENV BADGR_ACTION_GAIN $BADGR_ACTION_GAIN
ENV BADGR_WHEEL_BASE $BADGR_WHEEL_BASE


COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sed --in-place \
  "s|^source .*|source '$OVERLAY_WS/devel/setup.bash'|" \
  /ros_entrypoint.sh
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT [ "/ros_entrypoint.sh" ] 
CMD ["roslaunch", "capra_ros_badgr", "badgr.launch", "--wait"]

