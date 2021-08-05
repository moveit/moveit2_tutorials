import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    # rviz_arg = LaunchConfiguration('rviz', default=True)
    pkg_tutorials = get_package_share_directory("moveit2_tutorials")

    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use sim time"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Launch rviz or not"
        )
    )
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type", default_value="ur5e", description="Type/series of used UR robot."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    # UR5e Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # UR5e arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            ur_type,
            "default_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            ur_type,
            "physical_parameters.yaml",
        ]
    )
    visual_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            ur_type,
            "visual_parameters.yaml",
        ]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "ros_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            ur_type,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Ignition gazebo
    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo")
    empty_world_str = "-r " + os.path.join(pkg_tutorials, "worlds", "empty_world.sdf")
    ignition_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py")
        ),
        launch_arguments={"ign_args": empty_world_str}.items(),
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        # arguments=['-d', os.path.join(pkg_tutorials, 'rviz', 'ur_ignition.rviz')],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Spawn
    spawn_node = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name",
            "ur5e",
            "-topic",
            "robot_description",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    # Ign Bridge
    bridge_node = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            # JointTrajectory bridge (ROS2 -> IGN)
            "/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory",
            # Joint states (IGN -> ROS2)
            "/world/default/model/ur5e/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            # JointTrajectoryProgress bridge (IGN -> ROS2)
            "/joint_trajectory_progress@std_msgs/msg/Float32[ignition.msgs.Float",
            # Clock (IGN -> ROS2)
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        remappings=[
            ("/world/default/model/ur5e/joint_state", "joint_states"),
        ],
        output="screen",
    )

    # Controllers
    # TODO (vatanaksoytezer): Use ros_ign_control when it is ready
    ur_ignition_control_node = Node(
        package="ur_ignition_control",
        executable="ur_ignition_control_action_server",
        name="ur_ignition_control",
        output="screen",
    )

    nodes_to_start = [
        ignition_gazebo_launch,
        robot_state_publisher_node,
        spawn_node,
        bridge_node,
        ur_ignition_control_node,
        # rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
