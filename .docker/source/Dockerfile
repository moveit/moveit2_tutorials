# syntax = docker/dockerfile:1.3

# ghcr.io/ros-planning/moveit2:${ROS_DISTRO}-source
# Downloads the moveit source code and install remaining debian dependencies

ARG ROS_DISTRO=rolling
FROM moveit/moveit2:${ROS_DISTRO}-source

WORKDIR /root/ws_moveit
# Copy MoveIt sources from docker context
COPY . src/moveit2_tutorials

# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
# https://docs.docker.com/develop/develop-images/dockerfile_best-practices/#minimize-the-number-of-layers
RUN --mount=type=cache,target=/root/.ccache/ \
    # Enable ccache
    PATH=/usr/lib/ccache:$PATH && \
    # Fetch required upstream sources for building
    vcs import src < /root/ws_moveit/src/moveit2_tutorials/moveit2_tutorials.repos && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
     . "install/setup.sh" &&\
    sudo apt update && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
            --event-handlers desktop_notification- status- && \
    ccache -s

ENTRYPOINT ["sh","-c","sleep infinity"]