Launch Files in MoveIt
======================

Many of the MoveIt tutorials, as well as MoveIt packages you will encounter in the wild, use ROS 2 launch files.

This tutorial walks through a typical Python launch file that sets up a working MoveIt example.
We will do this by going through our :codedir:`Getting Started tutorial launch file <tutorials/quickstart_in_rviz/launch/demo.launch.py>` in detail.

If you are unfamiliar with launch files in general, refer first to `the ROS 2 documentation <https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html>`_.

Loading the MoveIt Configuration
--------------------------------

MoveIt requires several configuration parameters to include the robot description and semantic description files (:ref:`URDF and SRDF`), motion planning and kinematics plugins, trajectory execution, and more.
These parameters are usually contained in a :ref:`MoveIt Configuration` package.

A handy way to refer to a MoveIt configuration package is to use the ``MoveItConfigsBuilder`` utility in your Python launch files as follows:

.. code-block:: python

    from moveit_configs_utils import MoveItConfigsBuilder

    # Define xacro mappings for the robot description file
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "dof": "7",
    }

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

Launching Move Group
--------------------

Once all the MoveIt configuration parameters have been loaded, you can launch the :ref:`Move Group Interface` using the entire set of loaded MoveIt parameters.

.. code-block:: python

    from launch_ros.actions import Node

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

Visualizing with RViz
---------------------

As discussed in the :ref:`Quickstart in RViz` tutorial, you can visualize your robot model and perform motion planning tasks using RViz.

The following code uses a launch argument to receive an RViz configuration file name, packages it up as a relative path to a known package directory, and specifies it as an argument when launching the RViz executable.

.. code-block:: python

    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    # Get the path to the RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="kinova_moveit_config_demo.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("moveit2_tutorials"), "launch", rviz_base]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

Publishing Transforms to ``tf2``
--------------------------------

Many tools in the ROS ecosystem use the `tf2 <https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Tf2.html>`_ library to represent coordinate transforms, which are an important part of motion planning with MoveIt.

As such, our launch file includes nodes that publish both fixed (static) and dynamic names to ``tf2``.
In our case, we need:

* A static transform between the ``world`` frame and the base frame of our robot description, ``base_link``.
* A `robot state publisher <https://github.com/ros/robot_state_publisher>`_ node that listens to the robot's joint states, calculates frame transforms using the robot's URDF model, and publishes them to ``tf2``.

.. code-block:: python

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

Setting up ``ros2_control`` for trajectory execution
----------------------------------------------------

MoveIt normally generates joint trajectories that can then be executed by sending them to a robot with a controller capable of executing these trajectories.
Most commonly, we connect to the `ros2_control <https://control.ros.org/master/index.html>`_ library to achieve this.

While ``ros2_control`` allows you to connect to real robot hardware, or robots in a physics-based simulator like Gazebo or NVIDIA Isaac Sim, it also exposes a `mock components <https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html>`_ capability for simple, idealized simulations.
In our example, this is configured at the URDF level using the ``use_fake_hardware`` xacro parameter defined earlier on.
The key idea is that regardless of which hardware (simulated or real) is launched, the ``ros2_control`` launch remains the same.

Starting ``ros2_control`` involves launching a controller manager node, and then spawning a list of controllers necessary for trajectory execution.
In our example, we have:

* A joint state broadcaster, which publishes the joint states necessary for the robot state publisher to send frames to ``tf2``.
* A joint trajectory controller for the arm actuators.
* A gripper action controller for the parallel-jaw gripper.

.. code-block:: python

    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
    )

Launching all the nodes
-----------------------

Finally, we can tell our launch file to actually launch everything we described in the previous sections.

.. code-block:: python

    # ... all our imports go here

    def generate_launch_description():

        # ... all our other code goes here

        return LaunchDescription(
            [
                rviz_config_arg,
                rviz_node,
                static_tf,
                robot_state_publisher,
                run_move_group_node,
                ros2_control_node,
                joint_state_broadcaster_spawner,
                arm_controller_spawner,
                hand_controller_spawner,
            ]
        )
