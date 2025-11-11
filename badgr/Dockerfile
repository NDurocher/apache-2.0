ARG ROS_DISTRO=melodic
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
ARG BADGR_CONTROL_FREQ=5.
ARG BADGR_SAMPLE_BATCHES=25.
ARG BADGR_PLANNING_HORIZON=10
ARG BADGR_INITIAL_VELOCITY=1.5
ARG BADGR_INITIAL_STEERING_ANGLE=0.0
ARG BADGR_UPDATE_WEIGHTING=20
ARG BADGR_SAMPLE_VELOCITY_VARIANCE=0.3
ARG BADGR_SAMPLE_STEERING_VARIANCE=1.5
ARG BADGR_GOAL_GAIN=0.25
ARG BADGR_ACTION_GAIN=0.2
ARG BADGR_WHEEL_BASE=0.7

# MULTI-STAGE FOR CACHING
FROM $FROM_IMAGE AS cacher

# copy overlay source   
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./capra-ros-badgr/ capra-badgr/

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

# install CI dependencies
ARG ROS_DISTRO
RUN apt-get update && apt-get install -q -y --no-install-recommends\
  ccache \
  lcov \
  git \
  python \
  python3-dev \
  net-tools \ 
  iputils-ping \
  python3-pip \
  python-numpy \
  python-yaml \
  python3-yaml \
  python3-numpy \
  python3-scipy \
  python3-setuptools \  
  libeigen3-dev \
  libnlopt-dev \
  build-essential \
  unzip \
  g++ \
  ros-$ROS_DISTRO-vision-msgs \
  ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-tf-conversions \
  ros-$ROS_DISTRO-nlopt \
  ros-$ROS_DISTRO-actionlib-msgs \
  && rm -rf /var/lib/apt/lists/*
  
RUN pip3 install cython wheel
RUN pip3 install typing-extensions==3.6.6
#RUN pip3 install scikit-learn==0.22.2.post1 typing 
#RUN pip3 install torch==1.12.1+cu113 torchvision==0.13.1+cu113 torchaudio==0.12.1 --extra-index-url https://download.pytorch.org/whl/cu113
RUN pip3 install torch==1.9.0 torchvision==0.2.2.post3 -f https://download.pytorch.org/whl/torch_stable.html 
#RUN pip3 install h5py
# tensorflow-gpu
RUN pip3 install catkin_pkg
RUN pip3 install matplotlib==2.0.0
RUN pip3 install scikit-build

## Install Opencv
# RUN pip3 install opencv-python-headless==4.5.5.64


# Clone OpenCV and OpenCV contrib
# WORKDIR /opt
# RUN git clone https://github.com/opencv/opencv.git && \
#     git clone https://github.com/opencv/opencv_contrib.git
    
# # Set OpenCV version (adjust as needed)
# WORKDIR /opt/opencv
# RUN git checkout 4.7.0
# WORKDIR /opt/opencv_contrib
# RUN git checkout 4.7.0

# # Prepare build
# WORKDIR /opt/opencv/build
# RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
#     -D CMAKE_INSTALL_PREFIX=/usr/local \
#     -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
#     -D ENABLE_NEON=ON \
#     -D ENABLE_VFPV3=OFF \
#     -D WITH_GSTREAMER=ON \
#     -D WITH_GSTREAMER_0_10=OFF \
#     -D WITH_LIBV4L=OFF \
#     -D WITH_V4L=OFF \
#     -D WITH_TBB=ON \
#     -D WITH_GTK=ON \
#     -D WITH_GTK3=ON \
#     -D WITH_EIGEN=ON \
#     -D WITH_OPENMP=ON \
#     -D BUILD_EXAMPLES=OFF \
#     -D BUILD_DOCS=OFF \
#     -D BUILD_PERF_TESTS=OFF \
#     -D BUILD_TESTS=OFF \
#     -D BUILD_opencv_python3=ON \
#     -D PYTHON3_EXECUTABLE=/usr/bin/python3 \
#     -D PYTHON_EXECUTABLE=/usr/bin/python3 \
#     -D INSTALL_PYTHON_EXAMPLES=OFF \
#     ..

# # Build and install
# RUN make -j$(nproc) && \
#     make install && \
#     ldconfig



RUN apt-get update && apt-get install -q -y --no-install-recommends cmake curl \
   g++-7 \
   git \ 
   sudo \
   wget \
   && rm -rf /var/lib/apt/lists/*


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
#RUN rm /usr/bin/python
#RUN ln -s /usr/bin/python3 /usr/bin/python

ARG OVERLAY_MIXINS="release ccache"
#RUN . $ROS_SETUP && catkin_make --pkg capra_ros_badgr && catkin_make -DCMAKE_BUILD_TYPE=Release
RUN . $ROS_SETUP && \
  catkin_make --pkg capra_ros_badgr -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
  catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3

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

#RUN update-alternatives --install /usr/bin/python python /usr/bin/python2 1
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1
RUN chmod +x src/capra-badgr/scripts/Real_Life_Test.py

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN sed --in-place \
  "s|^source .*|source '$OVERLAY_WS/devel/setup.bash'|" \
  /ros_entrypoint.sh
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
ENTRYPOINT [ "/ros_entrypoint.sh" ] 
CMD ["roslaunch", "capra_ros_badgr", "badgr.launch", "--wait"]
