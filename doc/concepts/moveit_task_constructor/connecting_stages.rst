.. _Connecting Stages:

#################
Connecting Stages
#################

| MTC provides only one connecting stage called Connect.
| A connect stage solves for a feasible trajectory between the start and goal states.

Connect
-------

| The Connect stage connects two stages by finding a motion plan between the start and end goal given by the adjacent stages.
| The default cost term depends on path length.
| The default planning time for this stage is 1.0s.

.. list-table:: Properties to be set by user
   :widths: 25 100 80
   :header-rows: 1

   * - Property Name
     - Function to set property
     - Description
   * - merge_mode
     -
     - Define the merge strategy to use when performing planning operations. Can be SEQUENTIAL(Store sequential trajectories) or WAYPOINTS(Join trajectories by their waypoints). Default is WAYPOINTS.
   * - path_constaints
     - void setPathConstraints(moveit_msgs/Constraints path_constraints)
     - Constraints to maintain during trajectory
   * - merge_time_parameteriation
     -
     - Default is TOTG (Time-Optimal Trajectory Generation)

`API doc for Connect <https://ros-planning.github.io/moveit_task_constructor/_static/classmoveit_1_1task__constructor_1_1stages_1_1Connect.html>`_.

Code Example

.. code-block:: c++

  auto node = rclcpp::Node::make_shared("ur5");
  // planner used for connect
  auto pipeline = std::make_shared<solvers::PipelinePlanner>(node, "ompl", "RRTConnectkConfigDefault");
  // connect to pick
  stages::Connect::GroupPlannerVector planners = { { "arm", pipeline }, { "gripper", pipeline } };
  auto connect = std::make_unique<stages::Connect>("connect", planners);
