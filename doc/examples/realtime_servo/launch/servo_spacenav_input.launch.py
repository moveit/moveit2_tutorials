from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    teleop_config = PathJoinSubstitution(
        [
            FindPackageShare("moveit2_tutorials"),
            "config",
            "spacemouse_teleop_twist_joy.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[teleop_config],
                remappings=[("cmd_vel", "servo_node/delta_twist_cmds")],
            ),
        ]
    )
