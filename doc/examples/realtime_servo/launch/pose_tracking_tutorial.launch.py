import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    # Launch Servo as a standalone node or as a "node component" for better latency/efficiency
    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="false"
    )

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("moveit2_tutorials")
        .yaml("config/panda_simulated_config.yaml")
        .to_dict()
    }

    # This filter parameter should be >1. Increase it for greater smoothing but slower motion.
    low_pass_filter_coeff = {"butterworth_filter_coeff": 1.5}

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit2_tutorials")
        + "/config/demo_rviz_config.rviz"
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "-c", "/controller_manager"],
    )

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo_params,
                    low_pass_filter_coeff,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            launch_ros.descriptions.ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/panda_link0", "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            low_pass_filter_coeff,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    demo_node = launch_ros.actions.Node(
        package="moveit2_tutorials",
        executable="pose_tracking_tutorial",
        name="pose_tracking_tutorial",
        output="screen",
    )

    return launch.LaunchDescription(
        [
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            servo_node,
            container,
            launch.actions.TimerAction(period=10.0, actions=[demo_node]),
        ]
    )
