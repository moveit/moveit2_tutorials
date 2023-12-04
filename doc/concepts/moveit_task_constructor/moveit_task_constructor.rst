.. _MoveIt Task Constructor:

#######################
MoveIt Task Constructor
#######################

What is MoveIt Task Constructor?
--------------------------------

| The MoveIt Task Constructor framework helps break down complex planning tasks to multiple interdependent subtasks.
| The MTC framework uses MoveIt to solve the subtasks. Information from the subtasks are passes through the InterfaceState object.

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
| Generator stages get no input from adjacent stages. They compute results and pass them in both direction - upwards and downwards.
| Execution of a MTC task starts with the Generator stages.
| The most important generator stage is ``CurrentState``, which gets the current robot state as the starting point for a planning pipeline.

| Monitoring Generator is a stage that monitors the solution of another stage (not adjacent) to use the solutions for planning.
| Example of Monitoring Generator - ``GeneratePose``. It usually monitors a ``CurrentState`` or ``ModifyPlanning Scene`` stage. By monitoring the solutions of ``CurrentState``, the ``GeneratePose`` stage can find the object or frame around which it should generate poses.

| List of generator stages provided by MTC :ref:`Generating Stages`.

Propagating Stage
^^^^^^^^^^^^^^^^^
| Propagators receive solutions from one neighbor state, solve a problem and then propagate the result to the neighbor on the opposite side.
| Depending on the implementation, this stage can pass solutions forward, backward or in both directions.
| Example of propagating stage - Move Relative to a pose. This stage is commonly use to approach close to an object to pick.

| List of propagating stages provided by MTC :ref:`Propagating Stages`.

Connecting Stage
^^^^^^^^^^^^^^^^
| Connectors do not propagate any results but attempt to connect the start and goal inputs provided by adjacent stages.
| A connect stage often solves for a feasible trajectory between the start and goal states.

| List of connecting stages provided by MTC :ref:`Connecting Stages`.

Wrapper
^^^^^^^
| Wrappers encapsulate another stage to modify or filter the results.
| Example of wrapper - Compute IK for Generate Grasp Pose stage. A Generate Grasp Pose stage will produce cartesian pose solutions. By wrapping an Compute IK stage around Generate Pose stage, the cartesian pose solutions from Generate Pose stage can be used to produce IK solutions (i.e) produce joint state configuration of robot to reach the poses.

| List of wrappers provided by MTC :ref:`Wrappers`.

MTC Containers
---------------
| The MTC framework enables the hierarchical organization of stages using containers, allowing for sequential as well as parallel compositions.
| A MTC container helps organize the order of execution of the stages.
| Programmatically, it is possible to add a container within another container.

Currently available containers:

* Serial

* Parallel

Serial Container
^^^^^^^^^^^^^^^^
| Serial containers organizes stages linearly and only consider end-to-end solutions as results.
| A MTC Task by default is stored as a serial container.

Parallel Container
^^^^^^^^^^^^^^^^^^
Parallel containers combine a set of stages to allow planning alternate solutions.

| More information on parallel containers :ref:`Parallel Containers`.

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

Stages that does motion planning need solver information.

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

These solvers will be passed into stages like MoveTo, MoveRelative and Connect.

Setting Properties
------------------

| Each MTC stage has configurable properties. Example - planning group, timeout, goal state, etc.
| Properties of different types can be set using the function below.

.. code-block:: c++

  void setProperty(const std::string& name, const boost::any& value);

| Children stages can easily inherit properties from their parents, thus reducing the configuration overhead.

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

All stages provided by MTC have default cost terms. Stages which produce trajectories as solutions usually use path length to calculate cost.

Planning and Executing a MTC Task
---------------------------------

Planning MTC task will return a MoveItErrorCode.

.. code-block:: c++

  auto error_code = task.plan()

After planning, extract the first successful solution and pass it to the execute function. This will create an ``execute_task_solution`` action client and the action server resides in ``execute_task_solution_capability`` plugin provided by MTC.
The plugin extends MoveGroupCapability. It constructs a MotionPlanRequest from the MTC solution and uses MoveIt's PlanExecution to actuate the robot.

.. code-block:: c++

  auto result = task.execute(*task.solutions().front());


Links to Additional Information
--------------------------------

.. toctree::
    :maxdepth: 1

    generating_stages.rst
    propagating_stages.rst
    connecting_stages.rst
    wrappers.rst
    parallel_containers.rst
    debugging_mtc_task.rst
