# <launch>
#   <!-- Debug Info -->
#   <arg name="debug" default="false" />
#   <arg unless="$(arg debug)" name="launch_prefix" value="" />
#   <arg     if="$(arg debug)" name="launch_prefix" value="gdb --ex run --args" />
#   <arg name="planning_plugin" value="ompl_interface/OMPLPlanner" />
#   <arg name="planning_adapters" value="default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints" />

#   <node name="motion_planning_pipeline_tutorial" pkg="moveit_tutorials" type="motion_planning_pipeline_tutorial" respawn="false" launch-prefix="$(arg launch_prefix)" output="screen">
#     <param name="planning_plugin" value="$(arg planning_plugin)" />
#     <param name="request_adapters" value="$(arg planning_adapters)" />
#     <param name="start_state_max_bounds_error" value="0.1" />
#   </node>
# </launch>

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
    robot_description_config = load_file(
        "moveit_resources_panda_description", "urdf/panda.urdf"
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )

    motion_planning_pipeline_tutorial = Node(
        name="motion_planning_pipeline_tutorial",
        package="moveit2_tutorials",
        executable="motion_planning_pipeline_tutorial",
        prefix="xterm -e",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"planning_plugin": "ompl_interface/OMPLPlanner"},
            {
                "planning_adapters": "default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints"
            },
            {"start_state_max_bounds_error": 0.1},
        ],
    )

    return LaunchDescription([motion_planning_pipeline_tutorial])
