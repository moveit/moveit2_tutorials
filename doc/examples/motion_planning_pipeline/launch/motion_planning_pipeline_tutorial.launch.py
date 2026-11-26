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

        robot_description_config = load_file(
            "moveit_resources_panda_description", "urdf/panda.urdf"
        )

        robot_description_semantic_config = load_file(
            "moveit_resources_panda_moveit_config", "config/panda.srdf"
        )

        kinematics_yaml = load_yaml(
            "moveit_resources_panda_moveit_config", "config/kinematics.yaml"
        )

        ompl_planning_pipeline_config = {
            "ompl": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
                "start_state_max_bounds_error": 0.1,
            }
        }
        ompl_planning_yaml = load_yaml(
            "moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"
        )
        ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

        # Start the actual node
        motion_planning_node = Node(
            package="moveit2_tutorials",
            executable="motion_planning_pipeline_tutorial",
            prefix="xterm -e",
            output="screen",
            parameters=[
                {"robot_description": robot_description_config},
                {"robot_description_semantic": robot_description_semantic_config},
                {"planning_plugin": "ompl_interface/OMPLPlanner"},
                ompl_planning_pipeline_config,
                kinematics_yaml,
            ],
        )
