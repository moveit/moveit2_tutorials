# syntax = docker/dockerfile:1.3

ARG ROS_DISTRO=rolling
ARG GZ_VERSION=harmonic

######################### Tutorial Image  #################################################

FROM moveit/moveit2:${ROS_DISTRO}-source as tutorial_image

LABEL org.opencontainers.image.description "This container has working versions of the tutorials discussed here: https://moveit.picknik.ai/main/doc/tutorials/tutorials.html"

ARG GZ_VERSION

# Copy sources from docker context
COPY . src/moveit2_tutorials

# Fetch required upstream sources for building
RUN vcs import src < src/moveit2_tutorials/.github/upstream.repos

######################### Hello World Tutorial  #########################################

# Create a new package using the command from the tutorial
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
    . "install/setup.sh" &&\
    cd "src" &&\
    ros2 pkg create \
    --build-type ament_cmake \
    --dependencies moveit_ros_planning_interface moveit_visual_tools rclcpp \
    --node-name hello_moveit hello_moveit

# Replace template hello_moveit.cpp with implementation from planning_around_objects
COPY ./doc/tutorials/planning_around_objects/hello_moveit.cpp src/hello_moveit/src/hello_moveit.cpp

######################### Pick and Place (MTC) Image  #########################################

# Create a new package using the command from the tutorial
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
    . "install/setup.sh" &&\
    ros2 pkg create \
        --build-type ament_cmake \
        --destination-directory src \
        --dependencies moveit_task_constructor_core rclcpp \
        --node-name mtc_node mtc_tutorial

# Replace template mtc_node.cpp with the example file
COPY ./doc/tutorials/pick_and_place_with_moveit_task_constructor/src/mtc_node.cpp src/mtc_tutorial/src/mtc_node.cpp

# Add the launch folder to the tutorial package and CMakeLists.txt
COPY ./doc/tutorials/pick_and_place_with_moveit_task_constructor/launch src/mtc_tutorial/launch

# Install Gazebo, which is needed by some dependencies.
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    sudo apt update && \
    sudo apt-get install -y "gz-${GZ_VERSION}"

# Add install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME}) to CMakeLists.txt
RUN sed -i "s|ament_package()|install(DIRECTORY launch  DESTINATION share/\${PROJECT_NAME})\nament_package()|g" src/mtc_tutorial/CMakeLists.txt
# Build the tutorials and set up the entrypoint/bashrc
RUN --mount=type=cache,target=/root/.ccache/ \
    # Enable ccache
    . "/opt/ros/${ROS_DISTRO}/setup.sh" &&\
    . "install/setup.sh" &&\
    sudo apt update && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                     -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
        --event-handlers desktop_notification- status- && \
    ccache -s && \
    rm -rf /var/lib/apt/lists/* && \
    # Update /ros_entrypoint.sh to source our new workspace
    sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/install/setup.bash#g" /ros_entrypoint.sh && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws_moveit/install/setup.bash" >> /root/.bashrc
