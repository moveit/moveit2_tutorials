import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

moveit_config = (
    MoveItConfigsBuilder("moveit_resources_panda")
    .robot_description(file_path="config/panda.urdf.xacro")
    .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    # OMPL required for example.
    .planning_pipelines(default_planning_pipeline="ompl")
    .to_moveit_configs()
)

configurable_parameters = {
    # Cache DB
    "cache_db_plugin": {
        "default": "warehouse_ros_sqlite::DatabaseConnection",
        "description": "Plugin to use for the trajectory cache database.",
    },
    "cache_db_host": {
        "default": '":memory:"',
        "description": 'Host for the trajectory cache database. Use ":memory:" for an in-memory database.',
    },
    "cache_db_port": {
        "default": "0",
        "description": "Port for the trajectory cache database.",
    },
    # Reconfigurable (these can be set at runtime and will update)
    "start_tolerance": {  # Trajectory cache param
        "default": "0.025",
        "description": "Reconfigurable cache param. Tolerance for the start state of the trajectory.",
    },
    "goal_tolerance": {  # Trajectory cache param
        "default": "0.001",
        "description": "Reconfigurable cache param. Tolerance for the goal state of the trajectory.",
    },
    "prune_worse_trajectories": {  # Trajectory cache param
        "default": "false",
        "description": "Reconfigurable cache param. Whether to delete trajectories that are worse than the current trajectory.",
    },
    "planner": {
        "default": "RRTstar",
        "description": "Reconfigurable. Planner to use for trajectory planning.",
    },
    # Tutorial
    "num_target_poses": {
        "default": "4",
        "description": "Number of target poses to generate.",
    },
    "num_cartesian_target_paths_per_target_pose": {
        "default": "2",
        "description": "Number of Cartesian paths to generate for each target pose.",
    },
    "cartesian_path_distance_m": {
        "default": "0.10",
        "description": "Length of the Cartesian path to set the goal for.",
    },
    # Trajectory Cache
    "exact_match_precision": {
        # Purposely set a relatively high value to make pruning obvious.
        "default": "0.0001",
        "description": "Precision for checking if two trajectories are exactly the same.",
    },
    "cartesian_max_step": {
        "default": "0.001",
        "description": "Maximum step size for the Cartesian path.",
    },
    "cartesian_jump_threshold": {
        "default": "0.0",
        "description": "Threshold for the jump distance between points in the Cartesian path.",
    },
}


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(
            param_name, default_value=param["default"], description=param["description"]
        )
        for param_name, param in parameters.items()
    ]


def set_configurable_parameters(parameters):
    return {
        param_name: LaunchConfiguration(param_name) for param_name in parameters.keys()
    }


def generate_launch_description():
    trajectory_cache_demo = Node(
        name="trajectory_cache_demo",
        package="moveit2_tutorials",
        executable="trajectory_cache_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            set_configurable_parameters(configurable_parameters),
            {"warehouse_plugin": LaunchConfiguration("cache_db_plugin")},
        ],
    )

    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters)
        + [trajectory_cache_demo]
    )
