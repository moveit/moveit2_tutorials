from os.path import join as pathjoin

from launch import LaunchDescription, launch_description_sources
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory as pkgpath
from launch.actions import IncludeLaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder

ROS_NAMESPACE = ''

def generate_launch_description():

    ld = LaunchDescription()
    package_panda_moveit_config = "moveit_resources_panda_moveit_config"
    package_dir_panda_moveit_config = pkgpath(package_panda_moveit_config)

    ld.add_action(IncludeLaunchDescription(
                    launch_description_sources.PythonLaunchDescriptionSource(
                        package_dir_panda_moveit_config + '/launch/demo.launch.py')))

    ld.add_action( Node(
            namespace = ROS_NAMESPACE,
            package = 'moveit2_tutorials',
            executable = 'bag_publisher_maintain_time',
            name = 'point_clouds',
            output = 'screen',
            emulate_tty = True,
        ) )

    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="to_panda",
        output="log",
        arguments=["0","0","0", "0","0","0","world","panda_link0"],
    ))

    ld.add_action(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="to_camera",
        output="log",
        arguments=["0.115","0.427","0.570","0","0.2","1.92","camera_rgb_optical_frame","world"],
    ))

    

    return ld