FROM osrf/ros:humble-desktop-jammy

SHELL ["/bin/bash", "-c", "-o", "pipefail"]

RUN echo "deb [trusted=yes] https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/ ./" \
    | sudo tee /etc/apt/sources.list.d/moveit_moveit2_packages.list
RUN echo "yaml https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/local.yaml humble" \
    | sudo tee /etc/ros/rosdep/sources.list.d/1-moveit_moveit2_packages.list

# Bring the container up to date to get the latest ROS2 humble sync on 01/27/23
# hadolint ignore=DL3008, DL3013
RUN apt-get update && apt-get upgrade -y && rosdep update

# Install packages required to run the demo
RUN apt-get install -y --no-install-recommends \
    ros-humble-moveit \
    ros-humble-moveit-resources \
    ros-humble-moveit2-tutorials

# Use Fast DDS as middleware and load the required config for NVIDIA Isaac Sim.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN mkdir -p /opt/.ros
COPY ./.docker/fastdds.xml /opt/.ros/fastdds.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/.ros/fastdds.xml

# Set up the entrypoint for both container start and interactive terminals.
COPY .docker/ros_entrypoint.sh /opt/.ros/
RUN echo "source /opt/.ros/ros_entrypoint.sh" >> ~/.bashrc
ENTRYPOINT [ "/opt/.ros/ros_entrypoint.sh" ]
