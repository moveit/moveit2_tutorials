import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def absolute_path(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    return os.path.join(package_path, file_path)


def generate_launch_description():

    robot_description_path = absolute_path(
        "moveit_resources_panda_moveit_config", "config/panda.urdf.xacro"
    )
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = robot_description_config.toxml()

    robot_description_semantic_path = absolute_path(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )
    robot_description_semantic_file = open(robot_description_semantic_path, "r")
    robot_description_semantic = robot_description_semantic_file.read()

    tutorial_node = Node(
        package="moveit2_tutorials",
        executable="robot_model_and_robot_state_tutorial",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
        ],
    )

    return LaunchDescription([tutorial_node])
