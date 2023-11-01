.. _Connecting Stages:

#################
Connecting Stages
#################

Connect
-------

| The Connect stage connects two stages by finding a motion plan between them.
| The start and goal states for this stage are passed through InterfaceState objects.
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

Code Example

.. code-block:: c++

  auto node = rclcpp::Node::make_shared("ur5");
  // planner used for connect
  auto pipeline = std::make_shared<solvers::PipelinePlanner>(node, "ompl", "RRTConnectkConfigDefault");
  // connect to pick
  stages::Connect::GroupPlannerVector planners = { { "arm", pipeline }, { "gripper", pipeline } };
  auto connect = std::make_unique<stages::Connect>("connect", planners);
