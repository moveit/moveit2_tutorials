Low Level Controllers
=====================
MoveIt typically publishes manipulator motion commands to a `JointTrajectoryController <https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller>`_. This tutorial assumes MoveGroup is being used to control the robot rather than MoveItCpp or MoveIt Servo. A minimal setup is as follows:

#. A YAML config file. As best practice, we suggest naming this :code:`moveit_controllers.yaml`. It tells MoveIt which controllers are available, which joints are associated with each, and the MoveIt controller interface type (:code:`FollowJointTrajectory` or :code:`GripperCommand`). `Example controller config file <https://github.com/moveit/moveit_resources/blob/ros2/panda_moveit_config/config/moveit_controllers.yaml>`_.

#. A launch file. This launch file must load the :code:`moveit_controllers` yaml file and specify the :code:`moveit_simple_controller_manager/MoveItSimpleControllerManager`. After these yaml files are loaded, they are passed as parameters to the Move Group node. `Example Move Group launch file <https://github.com/moveit/moveit_resources/blob/ros2/panda_moveit_config/launch/demo.launch.py>`_.

#. Launch the corresponding :code:`ros2_control` JointTrajectoryControllers. This is separate from the MoveIt 2 ecosystem. `Example ros2_control launching <https://github.com/ros-controls/ros2_control_demos>`_. Each JointTrajectoryController provides an action interface. Given the yaml file above, MoveIt automatically connects to this action interface.

#. Note: it is not required to use :code:`ros2_control` for your robot. You could write a proprietary action interface. In practice, 99% of users choose :code:`ros2_control`.

MoveIt Controller Managers
--------------------------
The base class of controller managers is called MoveItControllerManager (MICM). One of the child classes of MICM is known as Ros2ControlManager (R2CM) and it is the best way to interface with ros2_control. The R2CM can parse the joint names in a trajectory command coming from MoveIt and activate the appropriate controllers. For example, it can automatically switch between controlling two manipulators in a single joint group at once to a single manipulator. To use a R2CM, just set :code:`moveit_manage_controllers = true` in the launch file. `Example R2CM launch file <https://github.com/moveit/moveit_resources/blob/ros2/panda_moveit_config/launch/demo.launch.py>`_.

MoveIt Controller Interfaces
----------------------------

The text above describes launching of a joint trajectory controller action interface. In addition, MoveIt supports parallel-jaw gripper control via action interface. This section describes the parameters of these two options.

#. FollowJointTrajectory Controller Interface

The parameters are:
 * *name*: The name of the controller.  (See debugging information below for important notes).
 * *action_ns*: The action namespace for the controller. (See debugging information below for important notes).
 * *type*: The type of action being used (here FollowJointTrajectory).
 * *default*: The default controller is the primary controller chosen by MoveIt for communicating with a particular set of joints.
 * *joints*: Names of all the joints that are being addressed by this interface.

#. GripperCommand Controller Interface

The parameters are:
 * *name*: The name of the controller.  (See debugging information below for important notes).
 * *action_ns*: The action namespace for the controller. (See debugging information below for important notes).
 * *type*: The type of action being used (here GripperCommand).
 * *default*: The default controller is the primary controller chosen by MoveIt for communicating with a particular set of joints.
 * *joints*: Names of all the joints that are being addressed by this interface.
 * *command_joint*: The single joint, controlling the actual state of the gripper. This is the only value that is sent to the controller. Has to be one of the joints above. If not specified, the first entry in *joints* will be used instead.
 * *parallel*: When this is set, *joints* should be of size 2, and the command will be the sum of the two joints.

Optional Allowed Trajectory Execution Duration Parameters
---------------------------------------------------------

(TODO: update for ROS2)

For each controller it is optional to set the *allowed_execution_duration_scaling* and *allowed_goal_duration_margin* parameters. These are controller-specific overrides of the global values *trajectory_execution/allowed_execution_duration_scaling* and *trajectory_execution/allowed_goal_duration_margin*. As opposed to the global values, the contoller-specific ones cannot be dynamically reconfigured at runtime. The parameters are used to compute the allowed trajectory execution duration by scaling the expected execution duration and adding the margin afterwards. If this duration is exceeded the trajectory will be cancelled. The controller-specific parameters can be set as follows ::

 controller_list:
  - name: arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    allowed_execution_duration_scaling: 1.2
    allowed_goal_duration_margin: 0.5

Debugging Information
---------------------

(TODO: update for ROS2)

The ``FollowJointTrajectory`` or ``GripperCommand`` interfaces on your robot must be communicating in the namespace: ``/name/action_ns``. In the above example, you should be able to see the following topics (using *ros2 topic list*) on your robot:

 * /panda_arm_controller/follow_joint_trajectory/goal
 * /panda_arm_controller/follow_joint_trajectory/feedback
 * /panda_arm_controller/follow_joint_trajectory/result
 * /hand_controller/gripper_action/goal
 * /hand_controller/gripper_action/feedback
 * /hand_controller/gripper_action/result

You should also be able to see (using ``ros2 topic info topic_name``) that the topics are published/subscribed to by the controllers on your robot and also by the **move_group** node.

Remapping /joint_states topic
-----------------------------

(TODO: update for ROS2)

When you run a :doc:`move group node </doc/examples/move_group_interface/move_group_interface_tutorial>`, you may need to remap the topic /joint_states to /robot/joint_states, otherwise MoveIt won't have feedback from the joints. To do this remapping you could make a simple launch file for your node as follows: ::

  <node pkg="moveit_ros_move_group" type="move_group" name="any_name" output="screen">
    <remap from="joint_states" to="robot/joint_states"/>
  </node>

Or you can make a subscriber with the correct topic name and then ensure that the starting robot state for your move group corresponds to a correct joints angle by using the call back of this subscriber.

Trajectory Execution Manager Options
------------------------------------

There are several options for tuning the behavior and safety checks of the execution pipeline in MoveIt. In your ``moveit_config`` package edit the ``trajectory_execution.launch.xml`` file to change the following parameters:

 - ``execution_duration_monitoring``: when false, will not throw error is trajectory takes longer than expected to complete at the low-level controller side
 - ``allowed_goal_duration_margin``: Allow more than the expected execution time before triggering a trajectory cancel (applied after scaling)
 - ``allowed_start_tolerance``: Allowed joint-value tolerance for validation that trajectory's first point matches current robot state. If set to zero will skip waiting for robot to stop after execution

Example Controller Manager
--------------------------

MoveIt controller managers, somewhat a misnomer, are the interfaces to your custom low level controllers. A better way to think of them are *controller interfaces*. For most use cases, the included :moveit_codedir:`MoveItSimpleControllerManager <moveit_plugins/moveit_simple_controller_manager>` is sufficient if your robot controllers already provide ROS actions for FollowJointTrajectory. If you use *ros_control*, the included :moveit_codedir:`MoveItRosControlInterface <moveit_plugins/moveit_ros_control_interface>` is also ideal.

However, for some applications you might desire a more custom controller manager. An example template for starting your custom controller manager is provided :codedir:`here <examples/controller_configuration/src/moveit_controller_manager_example.cpp>`.

Simulation
----------

If you do not have a physical robot, :code:`ros2_control` makes it very easy to simulate one. Ignition or Gazebo is not required; RViz is sufficient. All examples in the `ros2_control_demos repo <https://github.com/ros-controls/ros2_control_demos>`_ are simulated.

Controller Switching and Namespaces
-----------------------------------

(TODO: update for ROS2)

All controller names get prefixed by the namespace of their ros_control node. For this reason controller names should not contain slashes, and can't be named ``/``. For a particular node MoveIt can decide which controllers to have started or stopped. Since only controller names with registered allocator plugins are handled over MoveIt, MoveIt takes care of stopping controllers based on their claimed resources if a to-be-started controller needs any of those resources.

Controllers for Multiple Nodes
------------------------------

There is a variation on the Ros2ControlManager, the Ros2ControlMultiManager. Ros2ControlMultiManager can be used for more than one ros_control nodes. It works by creating several Ros2ControlManagers, one for each node. It instantiates them with their respective namespace and takes care of proper delegation. To use it must be added to the launch file. ::

  <param name="moveit_controller_manager" value="moveit_ros_control_interface::Ros2ControlMultiManager" />
