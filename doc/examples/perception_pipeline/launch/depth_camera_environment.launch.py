import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
import math


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("moveit2_tutorials"),
        "urdf/realsense_d435/camera.urdf.xacro",
    )

    # The list presents the pose in euler coordinates ordered x, y, z, yaw, pitch, roll respectively.
    camera_1_pose = ["-1", "-1", "0", f"{math.pi/2}", "0.0", "0.0"]
    camera_2_pose = ["-1", "1", "0", f"-{math.pi/2}", "0.0", "0.0"]

    # It is necessary for gazebo spawner to use the description of camera_1 in gazebo environment
    camera_1_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": Command(
                    [f"xacro {urdf_file}", " camera_name:='camera_1'"]
                ),
            }
        ],
        remappings=[
            ("robot_description", "/camera_1/robot_description"),
        ],
    )

    # It is necessary for spawning camera_1 in gazebo environment
    camera_1_gazebo_spawner_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "mr_camera",
            "-topic",
            "/camera_1/robot_description",
            "-x",
            camera_1_pose[0],
            "-y",
            camera_1_pose[1],
            "-z",
            camera_1_pose[2],
            "-Y",
            camera_1_pose[3],
            "-P",
            camera_1_pose[4],
            "-R",
            camera_1_pose[5],
        ],
        output="screen",
    )

    # It is necessary to make transformation between world frame and camera frames enable later.
    camera_1_tf_from_world_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[*camera_1_pose, "world", "camera_1_base_link"],
    )

    # It is necessary for gazebo spawner to use the description of camera_2 in gazebo environment
    camera_2_gazebo_spawner_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher2",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": Command(
                    [f"xacro {urdf_file}", " camera_name:='camera_2'"]
                ),
            }
        ],
        remappings=[
            ("robot_description", "/camera_2/robot_description"),
        ],
    )

    # It is necessary for spawning camera_2 in gazebo environment
    camera_2_robot_state_publisher_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn2",
        arguments=[
            "-entity",
            "mr_camera2",
            "-topic",
            "/camera_2/robot_description",
            "-x",
            camera_2_pose[0],
            "-y",
            camera_2_pose[1],
            "-z",
            camera_2_pose[2],
            "-Y",
            camera_2_pose[3],
            "-P",
            camera_2_pose[4],
            "-R",
            camera_2_pose[5],
        ],
        output="screen",
    )

    # It is necessary to make transformation between world frame and camera frames enable later.
    camera_2_tf_from_world_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[*camera_2_pose, "world", "camera_2_base_link"],
    )

    # It is necessary to open previously created gazebo world for perception pipeline demo.
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
        ),
        launch_arguments={
            "world": os.path.join(
                get_package_share_directory("moveit2_tutorials"),
                "worlds",
                "perception_pipeline_demo.world",
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            camera_1_robot_state_publisher_node,
            camera_1_gazebo_spawner_node,
            camera_1_tf_from_world_publisher_node,
            camera_2_robot_state_publisher_node,
            camera_2_gazebo_spawner_node,
            camera_2_tf_from_world_publisher_node,
            gazebo_launch,
        ]
    )
