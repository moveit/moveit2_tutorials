.. _Propagating Stages:

##################
Propagating Stages
##################

| Propagators receive solutions from one neighbor state, solve a problem, and then propagate the result to the neighbor on the opposite side.
| Depending on the implementation, this stage can pass solutions forward, backward, or in both directions.


MTC provides the following propagating stages:

* ``ModifyPlanningScene``

* ``MoveRelative``

* ``MoveTo``

* ``FixCollisionObjects``

ModifyPlanningScene
-------------------

| The ``ModifyPlanningScene`` stage applies modifications to the PlanningScene without moving the robot.
| By default, this stage propagates results in both direction.
| The default cost term is a constant of 0.

The stage contains functionality to
* Enable and disable collision checking between links
* Attach and detach objects to robot links
* Spawn and remove objects from scene

Example code to attach object

.. code-block:: c++

  auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
  stage->attachObject("object_name", "gripper_frame_name");

Example code to enable collision

.. code-block:: c++

  auto stage = std::make_unique<stages::ModifyPlanningScene>("Allow collision between object and gripper");
  stage->allowCollisions("object_name", "gripper_frame_name", true);

`API doc for ModifyPlanningScene <https://moveit.github.io/moveit_task_constructor/_static/classmoveit_1_1task__constructor_1_1stages_1_1ModifyPlanningScene.html>`_.

MoveRelative
------------

| The ``MoveRelative`` stage is used to perform a cartesian motion.
| By default, this stage propagates results in both directions.
| The default planning time for this stage is 1.0s.
| The default cost term depends on path length.

.. list-table:: Properties to be set by user
   :widths: 25 100 80
   :header-rows: 1

   * - Property Name
     - Function to set property
     - Description
   * - group
     - void setGroup(std::string group)
     - Name of planning group.
   * - ik_frame
     - void setIKFrame(std::string group)
     - Frame to be moved in Cartesian direction.
   * - min_distance
     - void setMinDistance(double distance)
     - Minimum distance to move. Default is -1.0.
   * - max_distance
     - void setMaxDistance(double distance)
     - Maximum distance to move. Default is 0.0.
   * - path_constaints
     - void setPathConstraints(moveit_msgs/Constraints path_constaints)
     - Constraints to maintain during trajectory
   * - direction
     - void setDirection(geometry_msgs/TwistStamped twist)
     - Perform twist motion on specified link.
   * - direction
     - void setDirection(geometry_msgs/Vector3Stamped direction)
     - Translate link along given direction.
   * - direction
     - void setDirection(std::map<std::string, double> direction)
     - Move specified joint variables by given amount

`API doc for MoveRelative <https://moveit.github.io/moveit_task_constructor/_static/classmoveit_1_1task__constructor_1_1stages_1_1MoveRelative.html>`_.

Example code

.. code-block:: c++

  const auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  auto approach_pose =
      std::make_unique<moveit::task_constructor::stages::MoveRelative>("Approach", cartesian_planner);
  // Propagate the solution backward only
  stage_approach_grasp->restrictDirection(moveit::task_constructor::stages::MoveRelative::BACKWARD);
  stage_approach_grasp->setGroup("manipulator");
  stage_approach_grasp->setIKFrame("tool_frame");

  // Move the end effector by 0.15m in the z direction.
  const Eigen::Vector3d approach{ 0.0, 0.0, 0.15 };
  geometry_msgs::msg::Vector3Stamped approach_vector;
  tf2::toMsg(approach, approach_vector.vector);
  approach_vector.header.frame_id = "tool_frame";
  stage_approach_grasp->setDirection(approach_vector);

MoveTo
------

| The ``MoveTo`` stage is used to move to a joint state or cartesian goal pose.
| By default, this stage propagates results in both directions.
| The default planning time for this stage is 1.0s.
| The default cost term depends on path length.

| The properties needed to be set for this stage are listed in the table below.
| The goal can be specified in different formats.

.. list-table:: Properties to be set by user
   :widths: 25 100 80
   :header-rows: 1

   * - Property Name
     - Function to set property
     - Description
   * - group
     - void setGroup(std::string group)
     - Name of planning group.
   * - ik_frame
     - void setIKFrame(geometry_msgs/PoseStamped pose)
     - Frame to be moved towards goal pose.
   * - goal
     - void setGoal(geometry_msgs/PoseStamped pose)
     - Move link to given pose
   * - goal
     - void setGoal(geometry_msgs/PointStamped point)
     - Move link to given point, keeping current orientation
   * - goal
     - void setGoal(std::string named_joint_pose)
     - Move joint model group to given named pose. The named pose should be described in the SRDF file.
   * - goal
     - void setGoal(moveit_msgs/RobotState robot_state)
     - Move joints specified in msg to their target values.
   * - goal
     - void setGoal(std::map<std::string, double> joints)
     - Move joints by name to their mapped target values.
   * - path_constaints
     - void setPathConstraints(moveit_msgs:::Constraints path_constaints)
     - Constraints to maintain during trajectory

`API doc for MoveTo <https://moveit.github.io/moveit_task_constructor/_static/classmoveit_1_1task__constructor_1_1stages_1_1MoveTo.html>`_.

Example code

.. code-block:: c++

  const auto joint_interpolation_planner =
      std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  auto stage =
        std::make_unique<moveit::task_constructor::stages::MoveTo>("close gripper", joint_interpolation_planner);
  // Set trajectory execution info. This will contain the list of controllers used to actuate gripper and arm.
  // Since this property is set during task initialization, we can inherit from it.
  stage->properties().set("trajectory_execution_info",
                          std::any_cast<moveit::task_constructor::TrajectoryExecutionInfo>(task->properties().get("trajectory_execution_info")));
  stage->setGroup("gripper"));
  stage->setGoal("closed"); // Group state named in SRDF
  stage->setTimeout(2.0);

FixCollisionObjects
-------------------

| The ``FixCollisionObjects`` stage checks for collisions and resolves them if applicable.
| By default, this stage propagates results in both directions.
| The default cost term is a constant of 0.

.. list-table:: Properties to be set by user
   :widths: 25 100 80
   :header-rows: 1

   * - Property Name
     - Function to set property
     - Description
   * - direction
     - void setDirection(geometry_msgs/Vector3 direction)
     - Direction vector to fix collision by shifting object along correction direction. A default direction is calculated if not explicitly set.
   * - penetration
     - void setMaxPenetration(double penetration)
     - Cutoff length up to which collision objects get fixed. If the object's collision length is greater than the value set, the collision will not be fixed.

Example code

.. code-block:: c++

  auto stage = std::make_unique<stages::FixCollisionObjects>();
  stage->setMaxPenetration(0.04);
