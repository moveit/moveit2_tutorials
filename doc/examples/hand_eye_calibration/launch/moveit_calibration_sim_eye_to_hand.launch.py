#!/usr/bin/env -S ros2 launch
"""MoveIt2 hand-eye calibration example inside Gazebo simulation (eye-to-hand variant)"""

import inspect
from math import pi, radians
from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CAMERA_NAME: str = "camera"
CALIBRATION_PATTERN: str = "charuco"  # "aruco" or "charuco"


def generate_camera_sdf(
    static: bool = True,
    update_rate: float = 15.0,
    width: int = 1280,
    height: int = 720,
    horizontal_fov: float = 0.5 * pi,
    vertical_fov: float = 0.5 * pi,
    noise_mean: float = 0.0,
    noise_stddev: float = 0.025,
    clip_near: float = 0.01,
    clip_far: float = 1000.0,
) -> str:
    """
    Generate SDF description for a camera model.
    """

    return inspect.cleandoc(
        f"""
        <sdf version="1.9">
          <model name="{CAMERA_NAME}">
            <static>{static}</static>
            <link name="{CAMERA_NAME}_link">
              <sensor name="{CAMERA_NAME}_sensor" type="{CAMERA_NAME}">
                <topic>{CAMERA_NAME}</topic>
                <always_on>true</always_on>
                <update_rate>{update_rate}</update_rate>
                <camera name="{CAMERA_NAME}_camera">
                  <image>
                    <width>{width}</width>
                    <height>{height}</height>
                    <format>R8G8B8</format>
                  </image>
                  <horizontal_fov>{horizontal_fov}</horizontal_fov>
                  <vertical_fov>{vertical_fov}</vertical_fov>
                  <clip>
                    <near>{clip_near}</near>
                    <far>{clip_far}</far>
                  </clip>
                  <noise>
                    <type>gaussian</type>
                    <mean>{noise_mean}</mean>
                    <stddev>{noise_stddev}</stddev>
                  </noise>
                </camera>
                <visualize>true</visualize>
              </sensor>
              <visual name="{CAMERA_NAME}_visual_lens">
                <pose>-0.01 0 0 0 1.5707963 0</pose>
                <geometry>
                  <cylinder>
                    <radius>0.02</radius>
                    <length>0.02</length>
                  </cylinder>
                </geometry>
                <material>
                  <diffuse>0.0 0.0 0.8</diffuse>
                  <specular>0.2 0.2 0.2</specular>
                </material>
              </visual>
              <visual name="{CAMERA_NAME}_visual_body">
                <pose>-0.05 0 0 0 0 0</pose>
                <geometry>
                  <box>
                    <size>0.06 0.05 0.05</size>
                  </box>
                </geometry>
                <material>
                  <diffuse>0.0 0.0 0.8</diffuse>
                  <specular>0.2 0.2 0.2</specular>
                </material>
              </visual>
            </link>
          </model>
        </sdf>"""
    )


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    robot = LaunchConfiguration("robot")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")

    # List of included launch descriptions
    launch_descriptions = [
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_ign_gazebo"),
                        "launch",
                        "ign_gazebo.launch.py",
                    ]
                )
            ),
            launch_arguments=[("ign_args", [world, " -r -v ", ign_verbosity])],
        ),
        # TODO: Update launch include of move group once implemented for MoveIt 2 + Gazebo under moveit repositories
        # Note: https://github.com/AndrejOrsula/panda_ign_moveit2 is currently required
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare([robot, "_moveit_config"]),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("ros2_control_plugin", "ign"),
                ("ros2_control_command_interface", "effort"),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # TODO: Update robot model/path once compable with MoveIt 2 + Gazebo under moveit repositories
        # Note: https://github.com/AndrejOrsula/panda_ign_moveit2 is currently required
        # ros_ign_gazebo_create (robot)
        Node(
            package="ros_ign_gazebo",
            executable="create",
            output="log",
            arguments=["-file", robot, "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # ros_ign_gazebo_create (calibration pattern)
        Node(
            package="ros_ign_gazebo",
            executable="create",
            output="log",
            arguments=[
                "-file",
                "https://fuel.ignitionrobotics.org/1.0/AndrejOrsula/models/aruco_default"
                if "aruco" == CALIBRATION_PATTERN
                else "https://fuel.ignitionrobotics.org/1.0/AndrejOrsula/models/charuco_default",
                "-x",
                "0.125",
                "-y",
                "0.05",
                "-z",
                "1.0",
                "-R",
                "0.0",
                "-P",
                f"{pi/2}",
                "-Y",
                f"{pi/4}",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # ros_ign_gazebo_create (camera)
        Node(
            package="ros_ign_gazebo",
            executable="create",
            output="log",
            arguments=[
                "-string",
                generate_camera_sdf(),
                "-x",
                LaunchConfiguration("camera_x"),
                "-y",
                LaunchConfiguration("camera_y"),
                "-z",
                LaunchConfiguration("camera_z"),
                "-R",
                LaunchConfiguration("camera_roll"),
                "-P",
                LaunchConfiguration("camera_pitch"),
                "-Y",
                LaunchConfiguration("camera_yaw"),
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # static_transform_publisher (identity; camera link -> camera sensor)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                [CAMERA_NAME, "/", CAMERA_NAME, "_link"],
                "--child-frame-id",
                [CAMERA_NAME, "/", CAMERA_NAME, "_link/", CAMERA_NAME, "_sensor"],
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # ros_ign_bridge (clock -> ROS 2; image -> ROS 2; camera info -> ROS 2)
        Node(
            package="ros_ign_bridge",
            executable="parameter_bridge",
            output="log",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                ["/", CAMERA_NAME, "@sensor_msgs/msg/Image[ignition.msgs.Image"],
                [
                    "/",
                    CAMERA_NAME,
                    "_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                ],
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # World and model for Ignition Gazebo
        DeclareLaunchArgument(
            "world",
            default_value="https://fuel.ignitionrobotics.org/1.0/AndrejOrsula/worlds/default_with_world_plugins_ogre",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "robot",
            default_value="panda",
            description="Name or filepath of model to load.",
        ),
        # Camera properties
        DeclareLaunchArgument(
            "camera_x",
            default_value="1.0",
            description="Position of the camera along x.",
        ),
        DeclareLaunchArgument(
            "camera_y",
            default_value="-0.5",
            description="Position of the camera along y.",
        ),
        DeclareLaunchArgument(
            "camera_z",
            default_value="0.5",
            description="Position of the camera along z.",
        ),
        DeclareLaunchArgument(
            "camera_roll",
            default_value="0.0",
            description="Orientation of the camera (roll).",
        ),
        DeclareLaunchArgument(
            "camera_pitch",
            default_value=f"{radians(5.0)}",
            description="Orientation of the camera (pitch).",
        ),
        DeclareLaunchArgument(
            "camera_yaw",
            default_value=f"{radians(150.0)}",
            description="Orientation of the camera (yaw).",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("moveit2_tutorials"),
                "rviz",
                "moveit_calibration_eye_to_hand.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "ign_verbosity",
            default_value="3",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
