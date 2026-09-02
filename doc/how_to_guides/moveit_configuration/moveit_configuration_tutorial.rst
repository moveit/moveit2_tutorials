.. _MoveIt Configuration:

MoveIt Configuration
==================================

The recommended way of configuring MoveIt for your robot is by creating a colcon package containing the MoveIt Configuration.

Suppose you would like to create a configuration package for some robot named ``my_robot``.
To do this, you can create a colcon package named ``my_robot_moveit_config``, whose structure is as follows:

.. code-block::

    my_robot_moveit_config
        config/
            kinematics.yaml
            joint_limits.yaml
            *_planning.yaml
            moveit_controllers.yaml
            moveit_cpp.yaml
            sensors_3d.yaml
            ...
        launch/
        .setup_assistant
        CMakeLists.txt
        package.xml

You can create such a package manually, or use the :doc:`MoveIt Setup Assistant </doc/examples/setup_assistant/setup_assistant_tutorial>` to automatically generate it given a robot description file (URDF or xacro).

Refer to the `moveit_resources <https://github.com/moveit/moveit_resources/tree/ros2>`_ repository for working examples of MoveIt configuration packages.


Configuration Files Overview
----------------------------

The ``config/`` folder of a MoveIt configuration package contains several files that describe parameters for various capabilities of MoveIt.

Note that several of these files are optional depending on the functionality required at runtime.

Robot Description
^^^^^^^^^^^^^^^^^

This is the most important piece of information in a MoveIt configuration package.
There must be a URDF and SRDF file present in this folder to describe the robot kinematics, planning groups, collision rules, and more.
To learn more about these files, refer to the :doc:`URDF/SRDF Overview </doc/examples/urdf_srdf/urdf_srdf_tutorial>`.

Joint Limits
^^^^^^^^^^^^

The URDF file specification allows setting joint position and velocity limits.
However, you may want to define different joint limits for motion planning with MoveIt without modifying the underlying robot description file.
Furthermore, some features in MoveIt use additional types of joint limits, such as acceleration and jerk limits, which cannot be specified in URDF.

The default location of this file is in ``config/joint_limits.yaml``.

Here is an example snippet of a joint limits file for a simple robot with two joints:

.. code-block:: yaml

    joint_limits:
        joint1:
            has_velocity_limits: true
            max_velocity: 2.0
            has_acceleration_limits: true
            max_acceleration: 4.0
            has_jerk_limits: false
        panda_joint2:
            has_velocity_limits: true
            max_velocity: 1.5
            has_acceleration_limits: true
            max_acceleration: 3.0
            has_jerk_limits: false

Inverse Kinematics (IK) Solver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Many motion planning applications in MoveIt require solving inverse kinematics.

Both the IK solver plugin used and its parameters are configured through a file whose default location is ``config/kinematics.yaml``.

For more information, refer to :doc:`Kinematics Configuration </doc/examples/kinematics_configuration/kinematics_configuration_tutorial>`.

Motion Planning Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For each type of motion planner plugin available in MoveIt, there is a corresponding ``config/*_planning.yaml`` file that describes its configuration.
For example, a robot that can use both :doc:`OMPL </doc/examples/ompl_interface/ompl_interface_tutorial>` and :doc:`Pilz Industrial Motion Planner </doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner>` will have the following folder structure:

.. code-block::

    my_robot_moveit_config
        config/
            ompl_planning.yaml
            pilz_industrial_motion_planner_planning.yaml
            ...
        ...

By default, all parameter files that match this ``config/*_planning.yaml`` pattern will be loaded.
If OMPL is configured as a planning pipeline, that will be the default; otherwise, it will be the first pipeline in the list.

To learn more about the contents of the individual planning configuration files, refer to the configuration documentation for those planners.

Trajectory Execution Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

MoveIt typically publishes manipulator motion commands to a `JointTrajectoryController <https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller>`_.
To learn more, refer to the :doc:`Low Level Controllers </doc/examples/controller_configuration/controller_configuration_tutorial>` section.

The default location for trajectory execution information is in ``config/moveit_controllers.yaml``.

MoveItCpp Configuration
^^^^^^^^^^^^^^^^^^^^^^^

If you are using :doc:`MoveItCpp </doc/examples/moveit_cpp/moveitcpp_tutorial>`, you can define a file with all the necessary parameters.

The default location of this file is in ``config/moveit_cpp.yaml``.

3D Perception Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you are using a perception sensor capable of generating 3D point clouds for motion planning, you can configure those settings for MoveIt.
For more information, refer to the :doc:`Perception Pipeline Tutorial </doc/examples/perception_pipeline/perception_pipeline_tutorial>`.

The default location of this file is in ``config/sensors_3d.yaml``.

Loading Configuration Parameters into Launch Files
--------------------------------------------------

To easily load parameters from MoveIt configuration packages for use in your ROS 2 launch files, MoveIt provides a ``MoveItConfigsBuilder`` utility.
To load the configuration parameters from your ``my_robot_moveit_config`` package:

.. code-block:: python

    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = (
        MoveItConfigsBuilder("my_robot")
        .to_moveit_configs()
    )

Then, you can either use the complete set of configuration parameters when launching a node:

.. code-block:: python

    from launch_ros.actions import Node

    my_node = Node(
        package="my_package",
        executable="my_executable",
        parameters=[moveit_config.to_dict()],
    )

or you can include selected sub-components as follows:

.. code-block:: python

    from launch_ros.actions import Node

    my_node = Node(
        package="my_package",
        executable="my_executable",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

Note that the above syntax will automatically look for configuration files that match the default file naming patterns described in this document.
If you have a different naming convention, you can use the functions available in ``MoveItConfigsBuilder`` to directly set file names.
Using the launch file from :doc:`/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial` as an example:

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


``MoveItConfigsBuilder`` (`defined here <https://github.com/moveit/moveit2/blob/main/moveit_configs_utils/moveit_configs_utils/moveit_configs_builder.py>`_) can take a few different types of arguments.

* ``MoveItConfigsBuilder(package_name="package_name")``  will search for a package named "package_name".
* ``MoveItConfigsBuilder("robot_name")`` will search for an explicitly given package name.
Both arguments can be given, in which case the robot name is stored and the package with the explicitly given name will be loaded.
As seen above, ``gen3`` is the robot name, and MoveIt looks for the package ``kinova_gen3_7dof_robotiq_2f_85_moveit_config`` instead of ``gen3_moveit_config``.

``.robot_description`` can optionally take a file path to ``robot_name.urdf`` and/or assign a dictionary of argument mappings that are passed to the robot's urdf.xacro file.
The file path to ``robot_name.urdf`` must be relative to your robot package, so if your robot package is ``/robot_package`` and the urdf (or urdf xacro) file is ``robot_package/config/robot_name.urdf``
you would pass ``.robot_description(filepath="config/robot_name.urdf")``.
If you don't provide a file path, but you do give ``MoveItConfigsBuilder`` a robot name (see above paragraph), MoveIt will look for ``robot_package/config/robot_name.urdf``.

``.trajectory_execution`` loads trajectory execution and MoveIt controller manager's parameters from an optionally provided file path.
If a file path isn't given, MoveIt looks for files in the package's ``config`` folder for files ending with ``controllers.yaml``.

``.planning_scene_monitor`` allows you to set various parameters about what scene information is published and how often is it published.

``.planning_pipelines`` allows to you to list the names of the planners you want available to be used by your robot.
If you opt to not list pipelines, as in ``.planning_pipelines()``, MoveIt will look in the config folder of your package for files that end with "_planning.yaml".
Additionally, if no pipelines are listed, MoveIt will load a set of planners from its own library - this can be disabled adding ``load_all=False`` as an argument to ``.planning_pipelines``.
Listing the planner names specifiies which planners MoveIt should load; again these should be in your config folder.
MoveIt will also pick one of your planners to be the default planner.
If OMPL is one of your planners, it will be the default planner unless you set ``default_planning_pipeline`` to your desired default planner as in

.. code-block:: python

    .planning_pipelines(
            pipelines=["ompl", "your_favorite_planner"],
            default_planning_pipeline="your_favorite_planner",
        )

If OMPL is not in your planner list and you don't set a default planner, MoveIt will pick the first planner in the list.

Now that you have read this page, you should be able to better understand the launch files available throughout the MoveIt 2 tutorials, and when encountering other MoveIt configuration packages in the wild.
