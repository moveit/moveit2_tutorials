# syntax = docker/dockerfile:1.3

# ghcr.io/ros-planning/moveit2:${ROS_DISTRO}-source
# Downloads the moveit source code and install remaining debian dependencies

ARG ROS_DISTRO=rolling

######################### Base Tutorial Image  #########################################

FROM moveit/moveit2:${ROS_DISTRO}-source as base_image

# Export ROS_UNDERLAY for downstream docker containers
ENV ROS_UNDERLAY /root/ws_moveit/install
WORKDIR $ROS_UNDERLAY/..

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
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Debug \
            --event-handlers desktop_notification- status- && \
    ccache -s && \
    # Update /ros_entrypoint.sh to source our new workspace
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.bash#g" /ros_entrypoint.sh

######################### Hello World Tutorial Image  #########################################

FROM base_image as hello_world_image

#Make a new package using the command in the tuotrial
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
    . "install/setup.sh" &&\
    cd "src" &&\
    ros2 pkg create \
    --build-type ament_cmake \
    --dependencies moveit_ros_planning_interface rclcpp \
    --node-name hello_moveit hello_moveit

#Remove the empty cpp file and replace it with the example file
RUN rm src/hello_moveit/src/hello_moveit.cpp
COPY ./doc/tutorials/your_first_project/hello_moveit.cpp src/hello_moveit/src/hello_moveit.cpp
#Build the tutorial
RUN --mount=type=cache,target=/root/.ccache/ \
    # Enable ccache
    PATH=/usr/lib/ccache:$PATH && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
     . "install/setup.sh" &&\
    sudo apt update && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Debug \
            --event-handlers desktop_notification- status- && \
    ccache -s

######################### Planning Around Objects Image  #########################################

FROM hello_world_image as planning_around_objects_image

#Remove the hellow_world tutorial cpp file and replace it with the planning_around_objects file
RUN rm src/hello_moveit/src/hello_moveit.cpp
COPY ./doc/tutorials/planning_around_objects/hello_moveit.cpp src/hello_moveit/src/hello_moveit.cpp
#Build the tutorial
RUN --mount=type=cache,target=/root/.ccache/ \
    # Enable ccache
    PATH=/usr/lib/ccache:$PATH && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
     . "install/setup.sh" &&\
    sudo apt update && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Debug \
            --event-handlers desktop_notification- status- && \
    ccache -s

######################### Pick and Place (MTC) Image  #########################################

FROM planning_around_objects_image as pick_and_place_image

#Make a new package using the command in the tuotrial
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
    . "install/setup.sh" &&\
    ros2 pkg create \
        --build-type ament_cmake \
        --destination-directory src \
        --dependencies moveit_task_constructor_core rclcpp \
        --node-name mtc_node mtc_tutorial
#Remove the empty cpp file and replace it with the example file
RUN rm src/mtc_tutorial/src/mtc_node.cpp
COPY ./doc/tutorials/pick_and_place_with_moveit_task_constructor/src/mtc_node.cpp src/mtc_tutorial/src/mtc_node.cpp
#Add the launch folder to the tutorial package and CMakeLists.txt
COPY ./doc/tutorials/pick_and_place_with_moveit_task_constructor/launch src/mtc_tutorial/launch
RUN   sed -i "s|ament_package()|install(DIRECTORY launch  DESTINATION share/\${PROJECT_NAME})\nament_package()|g" src/mtc_tutorial/CMakeLists.txt
#Build the tutorial
RUN --mount=type=cache,target=/root/.ccache/ \
    # Enable ccache
    PATH=/usr/lib/ccache:$PATH && \
    . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
     . "install/setup.sh" &&\
    sudo apt update && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            --ament-cmake-args -DCMAKE_BUILD_TYPE=Debug \
            --event-handlers desktop_notification- status- && \
    ccache -s
