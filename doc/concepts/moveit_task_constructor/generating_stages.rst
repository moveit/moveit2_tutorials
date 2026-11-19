.. _Generating Stages:

#################
Generating Stages
#################

Generator stages get no input from adjacent stages. They compute results and pass them to adjacent stages.

MTC provides the following generator stages:

* ``CurrentState``

* ``FixedState``

* Monitoring Generators - ``GeneratePose``, ``GenerateGraspPose``, ``GeneratePlacePose`` and ``GenerateRandomPose``.

CurrentState
-------------
| The ``CurrentState`` stage fetches the current PlanningScene via the ``get_planning_scene`` service.
| This stage is often used at the beginning of the MTC task pipeline to set the start state from the current robot state.

Example code

.. code-block:: c++

  auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current_state");

`API doc for CurrentState <https://moveit.github.io/moveit_task_constructor/_static/classmoveit_1_1task__constructor_1_1stages_1_1CurrentState.html>`_.

FixedState
----------

| The ``FixedState`` stage spawns a pre-defined PlanningScene State.

.. code-block:: c++

  moveit::task_constructor::Task t;
  auto node = rclcpp::Node::make_shared("node_name");
  t.loadRobotModel(node);

  auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
  auto& state = scene->getCurrentStateNonConst();
  state.setToDefaultValues();  // initialize state
  state.setToDefaultValues(state.getJointModelGroup("left_arm"), "home");
  state.setToDefaultValues(state.getJointModelGroup("right_arm"), "home");
  state.update();
  spawnObject(scene); // User defined function to add a CollisionObject to planning scene

  auto initial = std::make_unique<stages::FixedState>();
  initial->setState(scene);

`API doc for FixedState <https://moveit.github.io/moveit_task_constructor/_static/classmoveit_1_1task__constructor_1_1stages_1_1FixedState.html>`_.

Monitoring Generators
---------------------
Monitoring Generators help monitor and use solutions of another stage.

GeneratePose
^^^^^^^^^^^^
``GeneratePose`` is a monitoring generator stage which can be used to generate poses based on solutions provided by the monitored stage.

GenerateGraspPose
^^^^^^^^^^^^^^^^^
| ``GenerateGraspPose`` stage is derived from ``GeneratePose``, which is a monitoring generator.
| This stage usually monitors the ``CurrentState`` stage since the stage requires the latest ``PlanningScene`` to find the location of object around which grasp poses will be generated.
| This stage can by used to generate poses for grasping by setting the desired attributes.
| There can be multiple ways to set the same property. For example, there are two functions to set the pre grasp pose as seen in the table below. The user can set this property by either using a string group state or by explicitly defining a RobotState.

.. list-table:: Properties to be set by user
   :widths: 25 100 80
   :header-rows: 1

   * - Property Name
     - Function to set property
     - Description
   * - eef
     - void setEndEffector(std::string eef)
     - Name of end effector
   * - object
     - void setObject(std::string object)
     - Object to grasp. This object should exist in the planning scene.
   * - angle_delta
     - void setAngleDelta(double delta)
     - Angular steps (rad). The target grasp pose is sampled around the object's z axis
   * - pregrasp
     - void setPreGraspPose(std::string pregrasp)
     - Pregrasp pose. For example, the gripper has to be in an open state before grasp. The pregrasp string here corresponds to the group state in SRDF.
   * - pregrasp
     - void setPreGraspPose(moveit_msgs/RobotState pregrasp)
     - Pregrasp pose
   * - grasp
     - void setGraspPose(std::string grasp)
     - Grasp pose
   * - grasp
     - void setGraspPose(moveit_msgs/RobotState grasp)
     - Grasp pose

Refer the API docs for the latest state of code.
`API doc for GenerateGraspPose <https://moveit.github.io/moveit_task_constructor/_static/classmoveit_1_1task__constructor_1_1stages_1_1GenerateGraspPose.html>`_.

Example code

.. code-block:: c++

  auto initial_stage = std::make_unique<stages::CurrentState>("current state");
  task->add(initial_stage);

  auto gengrasp = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
  gengrasp->setPreGraspPose("open");
  gengrasp->setObject("object");
  gengrasp->setAngleDelta(M_PI / 10.);
  gengrasp->setMonitoredStage(initial_stage);
  task->add(gengrasp);

GeneratePlacePose
^^^^^^^^^^^^^^^^^
| The ``GeneratePlacePose`` stage derives from ``GeneratePose``, which is a monitoring generator.
| This stage generates poses for the place pipeline.
| Notice that while ``GenerateGraspPose`` spawns poses with an ``angle_delta`` interval, ``GeneratePlacePose`` samples a fixed amount, which is dependent on the object's shape.

Example code

.. code-block:: c++

  // Generate Place Pose
  auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
  stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
  stage->properties().set("marker_ns", "place_pose");
  stage->setObject(params.object_name);

  // Set target pose
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = params.object_reference_frame;
  p.pose = vectorToPose(params.place_pose);
  p.pose.position.z += 0.5 * params.object_dimensions[0] + params.place_surface_offset;
  stage->setPose(p);
  stage->setMonitoredStage(pick_stage_ptr);  // hook into successful pick solutions

`API doc for GeneratePlacePose <https://moveit.github.io/moveit_task_constructor/_static/classmoveit_1_1task__constructor_1_1stages_1_1GeneratePlacePose.html>`_.


GenerateRandomPose
^^^^^^^^^^^^^^^^^^
| The ``GenerateRandomPose`` stage derives from ``GeneratePose``, which is a monitoring generator.
| This stage configures a RandomNumberDistribution (see https://en.cppreference.com/w/cpp/numeric/random) sampler for a PoseDimension (X/Y/Z/ROLL/PITCH/YAW) for randomizing the pose.

.. list-table:: Properties to be set by user
   :widths: 25 100 80
   :header-rows: 1

   * - Property Name
     - Function to set property
     - Description
   * - max_solution
     - void setMaxSolution(size_t max_solution)
     - Limit of the number of spawned solutions in case randomized sampling is enabled.

FixedCartesianPose
------------------
The ``FixedCartesianPose`` spawns a fixed Cartesian pose. This stage does no sampling.
This stage is useful for planning from an intended future state, enabling things like simultaneous planning and execution.
