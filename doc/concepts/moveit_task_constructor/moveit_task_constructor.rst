.. _MoveIt Task Constructor:

#######################
MoveIt Task Constructor
#######################

What is MoveIt Task Constructor?
--------------------------------

| The MoveIt Task Constructor framework helps break down complex planning tasks to multiple interdependent subtasks.
| The MTC framework uses MoveIt to solve the subtasks and a common interface, based on MoveIt's PlanningScene, is used to pass solution hypotheses between stages.

MTC Stages
-----------
| A MTC stage refers to a component or step in the task execution pipeline.
| Stages can be arranged in any arbitrary order and its hierarchy is only limited by the individual stages types.
| The order in which stages can be arranged is restricted by the direction in which results are passed.

There are three possible stages relating to the result flow:

* Generators

* Propagators

* Connectors

Generator Stage
^^^^^^^^^^^^^^^
| Generators compute results and pass them in both direction - upwards and downwards.
| Execution of a MTC task starts with the Generator stages.
| The most important generator stage is `CurrentState`, which provides the current robot state as the starting point for a planning pipeline.

| Monitoring Generator - Generator that monitors the solution of another stage to make reuse of them.
| Main example of Monitoring Generator - `GeneratePose`. It usually monitors a `CurrentState` or `ModifyPlanning Scene` stage.

Currently available generator stages:

* CurrentState

* FixedState

* Monitoring Generators - GeneratePose, GenerateGraspPose, GeneratePlacePose, GenerateRandomPose

| Link for more information on generator stages available to use :ref:`Generating Stages`.

Propagating Stage
^^^^^^^^^^^^^^^^^
| Propagators receive the result from one neighbor state, solve a subproblem and then propagate the result to the neighbor on the opposite side.
| Depending on the implementation, this stage can pass solutions forward, backward or in both directions.

| Link for more information on propagating stages available :ref:`Propagating Stages`.

Connecting Stage
^^^^^^^^^^^^^^^^
Connectors do not propagate any results but attempt to bridge the gap between the resulting states of both neighbors.

| Link for more information on how to use connect stage :ref:`Connecting Stages`.

Wrapper
^^^^^^^
| Wrappers encapsulate a single stage and modify/filter the results.
| Currently available Wrappers - ComputeIK, PredicateFilter and PassThrough

| Link for more information on wrappers available to use :ref:`Wrappers`.

MTC Containers
---------------
The MTC framework enables the hierarchical organization of stages using containers, allowing for sequential as well as parallel compositions.

Serial Container
^^^^^^^^^^^^^^^^
Serial containers hold a sequence of stages (and only consider end-to-end solutions as results)
A MTC Task by default is stored as a serial container.

| Link for more information on how to use serial container : TBD

Parallel Container
^^^^^^^^^^^^^^^^^^
Parallel containers combine a set of stages in the following formats

* Alternatives - Solution of all children are collected and sorted by cost.

* Fallback - A fallback container executes children stages in order until one of them returns success or all stages return failure

* Merger - Solutions of all children (actuating disjoint groups) are planned and executed parallelly.

| Link for more information on how to use parallel container : TBD

Initializing a MTC Task
-----------------------

The top-level planning problem is specified as a MTC Task and the subproblems which are specified by Stages are added to the MTC task object.

.. code-block:: c++

  auto node = std::make_shared<rclcpp::Node>();
  auto task = std::make_unique<moveit::task_constructor::Task>();
  task->loadRobotModel(node);
  // Set controllers used to execute robot motion
  task->setProperty("trajectory_execution_info", "joint_trajectory_controller gripper_controller");


Adding containers and stages to a MTC Task
-------------------------------------------

Adding a stage to MTC task -

.. code-block:: c++

  auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current_state");
  task->add(std::move(current_state));

Containers derive from Stage and hence containers can be added to MTC task similarly

.. code-block:: c++

  auto container = std::make_unique<moveit::task_constructor::SerialContainer>("Pick Object");
  // TODO: Add stages to the container before adding the container to MTC task
  task->add(std::move(container));

Setting planning solvers
------------------------

Solvers available in MTC

* PipelinePlanner - Uses MoveIt's planning pipeline

* JointInterpolation - Interpolates between the start and goal joint states. It does not support complex motions.

* CartesianPath - Moves the end effector in a straight line in Cartesian space.

Code Example on how to initialize the solver

.. code-block:: c++

  const auto mtc_pipeline_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(
      node, "ompl", "RRTConnectkConfigDefault");
  const auto mtc_joint_interpolation_planner =
      std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();
  const auto mtc_cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();

These solvers can be passed into each stage.

Setting Properties
------------------

| Each MTC stage has configurable properties.
| Properties of different types can be set using the function below.
| Information about properties required for each stage can be found in the stages docs.

.. code-block:: c++

  void setProperty(const std::string& name, const boost::any& value);

Cost calculator for Stages
---------------------------

CostTerm is the basic interface to compute costs for solutions for MTC stages.

CostTerm available in MTC

* Constant - Adds a constant cost to each solution

* PathLength - Cost depends on trajectory length with optional weight for different joints

* TrajectoryDuration - Cost depends on execution duration of the whole trajectory

* TrajectoryCostTerm - cost terms that only work on SubTrajectory solutions?

* LamdaCostTerm - Pass in a Lamda function to calculate cost

* DistanceToReference - Cost depends on weighted joint space distance to a reference point

* LinkMotion - Cost depends on length of Cartesian trajectory of a link

* Clearance - Cost is inverse of distance to collision

Example code on how to set CostTerm using LamdaCostTerm

.. code-block:: c++

  stage->setCostTerm(moveit::task_constructor::LambdaCostTerm(
        [](const moveit::task_constructor::SubTrajectory& traj) { return 100 * traj.cost(); }));

Planning and Executing a MTC Task
---------------------------------

Planning MTC task will return a MoveItErrorCode.

.. code-block:: c++

  auto error_code = task.plan()

After planning, extract the first successful solution and send the successful plan to `execute_task_solution` action server.

.. code-block:: c++

  moveit_task_constructor_msgs::msg::Solution solution;
  task.solutions().front()->toMsg(solution);


Links to Additional Information
--------------------------------

.. toctree::
    :maxdepth: 1

    generating_stages.rst
    propagating_stages.rst
    connecting_stages.rst
    wrappers.rst
    debugging_mtc_task.rst
