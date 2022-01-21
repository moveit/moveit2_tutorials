import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
<<<<<<< HEAD:doc/quickstart_in_rviz/launch/demo.launch.py

    # Command-line arguments
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )

    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("moveit_resources_panda_moveit_config"),
            "config",
            "panda.urdf.xacro",
        )
=======
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/panda_gripper_controllers.yaml")
        .to_moveit_configs()
>>>>>>> 8238b2c (Use moveit configs utils packages (#243)):doc/examples/persistent_scenes_and_states/move_group.launch.py
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
<<<<<<< HEAD:doc/quickstart_in_rviz/launch/demo.launch.py
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
=======
            moveit_config.to_dict(),
            # here
            warehouse_ros_config,
>>>>>>> 8238b2c (Use moveit configs utils packages (#243)):doc/examples/persistent_scenes_and_states/move_group.launch.py
        ],
    )

    # RViz
    tutorial_mode = LaunchConfiguration("rviz_tutorial")
    rviz_base = os.path.join(get_package_share_directory("moveit2_tutorials"), "launch")
    rviz_full_config = os.path.join(rviz_base, "panda_moveit_config_demo.rviz")
    rviz_empty_config = os.path.join(rviz_base, "panda_moveit_config_demo_empty.rviz")
    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_empty_config],
        parameters=[
<<<<<<< HEAD:doc/quickstart_in_rviz/launch/demo.launch.py
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
=======
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            # here
            warehouse_ros_config,
>>>>>>> 8238b2c (Use moveit configs utils packages (#243)):doc/examples/persistent_scenes_and_states/move_group.launch.py
        ],
        condition=IfCondition(tutorial_mode),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=UnlessCondition(tutorial_mode),
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "panda_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Warehouse mongodb server
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            tutorial_arg,
            rviz_node,
            rviz_node_tutorial,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            mongodb_server_node,
        ]
        + load_controllers
    )
