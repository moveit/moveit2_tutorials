ARG ROS_DISTRO=jazzy
FROM moveit/moveit2:${ROS_DISTRO}-source

SHELL ["/bin/bash", "-c", "-o", "pipefail"]

ARG ROS_DISTRO
ARG UBUNTU_VERSION=ubuntu2404
ARG CUDA_MAJOR_VERSION=12
ARG CUDA_MINOR_VERSION=4

# Bring the container up to date to get the latest ROS sync
# hadolint ignore=DL3008, DL3013
RUN apt-get update && apt-get upgrade -y && rosdep update

# Install packages required to run the demo
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    python3-pip \
    ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# Build topic_based_ros2_control from source since it is not yet released as an apt package for ${ROS_DISTRO}
RUN git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git -b main src/topic_based_ros2_control && \
    apt-get update && \
    rosdep update && \
    rosdep install -r --from-paths src/topic_based_ros2_control --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /root/ws_moveit/install/setup.sh && \
    colcon build --packages-select topic_based_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# Install CUDA
ENV DEBIAN_FRONTEND=noninteractive
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/${UBUNTU_VERSION}/x86_64/cuda-keyring_1.1-1_all.deb && \
    dpkg -i cuda-keyring_1.1-1_all.deb && \
    rm cuda-keyring_1.1-1_all.deb && \
    apt-get update && \
    apt-get -y install --no-install-recommends \
      cuda-cudart-${CUDA_MAJOR_VERSION}-${CUDA_MINOR_VERSION} \
      cuda-compat-${CUDA_MAJOR_VERSION}-${CUDA_MINOR_VERSION} \
      cuda-toolkit-${CUDA_MAJOR_VERSION}-${CUDA_MINOR_VERSION} && \
    rm -rf /var/lib/apt/lists/*
ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:$LD_LIBRARY_PATH

# Install pytorch
RUN pip3 install --no-cache-dir --break-system-packages \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cu${CUDA_MAJOR_VERSION}${CUDA_MINOR_VERSION}

# Use Fast DDS as middleware and load the required config for NVIDIA Isaac Sim.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN mkdir -p /opt/.ros
COPY ./doc/how_to_guides/isaac_panda/.docker/fastdds.xml /opt/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/.ros/fastdds.xml

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV NVIDIA_REQUIRE_CUDA="cuda>=${CUDA_MAJOR_VERSION}.${CUDA_MINOR_VERSION}"

# Set up the entrypoint for both container start and interactive terminals.
COPY ./doc/how_to_guides/isaac_panda/.docker/ros_entrypoint.sh /opt/.ros/
RUN chmod +x /opt/.ros/ros_entrypoint.sh && \
    echo "source /opt/.ros/ros_entrypoint.sh" >> ~/.bashrc
ENTRYPOINT [ "/opt/.ros/ros_entrypoint.sh" ]
