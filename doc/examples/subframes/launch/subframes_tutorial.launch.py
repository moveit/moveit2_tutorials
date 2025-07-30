from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Subframes Tutorial executable
    subframes_tutorial = Node(
        name="subframes_tutorial",
        package="moveit2_tutorials",
        executable="subframes_tutorial",
        output="screen",
    )

    return LaunchDescription([subframes_tutorial])
