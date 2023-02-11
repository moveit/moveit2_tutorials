FROM osrf/ros:humble-desktop-jammy

SHELL ["/bin/bash", "-c", "-o", "pipefail"]

RUN echo "deb [trusted=yes] https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/ ./" | sudo tee /etc/apt/sources.list.d/moveit_moveit2_packages.list
RUN echo "yaml https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-moveit_moveit2_packages.list

# Bring the container up to date to get the latest ROS2 humble sync on 01/27/23
# hadolint ignore=DL3008, DL3013
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get upgrade -y

RUN rosdep update

# Install moveit and ros2_control to run the demo
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# Create Colcon workspace and clone the needed source code to run the demo
RUN mkdir -p /root/isaac_moveit_tutorial_ws/src
WORKDIR /root/isaac_moveit_tutorial_ws/src
RUN git clone https://github.com/PickNikRobotics/isaac_ros2_control.git
RUN git clone https://github.com/JafarAbdi/moveit_resources.git -b pr-hardware_type
# COPY ./../../../* ./moveit2_tutorials ## this is not working so you have to run rosdep again after starting the container
WORKDIR /root/isaac_moveit_tutorial_ws
# hadolint ignore=SC1091
RUN apt update
# For some reason this fails to get several moveit dependencies and you have to run it again inside the container! 
RUN rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
 #  && rm -rf /var/lib/apt/lists/*

# Use Fast DDS as middleware and load the required config for NVIDIA Isaac Sim.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN mkdir -p /opt/.ros
COPY ./.docker/fastdds.xml /opt/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/.ros/fastdds.xml

# Build the Colcon workspace for the user
# RUN source /opt/ros/humble/setup.bash && colcon build

# Set up the entrypoint for both container start and interactive terminals.
COPY .docker/ros_entrypoint.sh /opt/.ros/
RUN echo "source /opt/.ros/ros_entrypoint.sh" >> ~/.bashrc
ENTRYPOINT [ "/opt/.ros/ros_entrypoint.sh" ]
