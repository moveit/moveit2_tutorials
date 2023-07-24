from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="spacenav",
                executable="spacenav_node",
                name="spacenav",
                output="screen",
                parameters=[{"zero_when_static": True, "use_twist_stamped": True}],
                remappings=[("spacenav/twist", "servo_node/delta_twist_cmds")],
            ),
        ]
    )
