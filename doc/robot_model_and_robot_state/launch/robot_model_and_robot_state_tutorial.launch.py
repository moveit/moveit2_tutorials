import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


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

    robot_description_semantic_config = load_file(
        "moveit_resources_panda_moveit_config", "config/panda.srdf"
    )

    kinematics_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
    )

    tutorial_node = Node(
        package="moveit2_tutorials",
        executable="robot_model_and_robot_state_tutorial",
        output="screen",
        parameters=[
            {"robot_description": robot_description_config},
            {"robot_description_semantic": robot_description_semantic_config},
            kinematics_yaml,
        ],
    )

    return LaunchDescription([tutorial_node])
