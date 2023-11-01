.. _Generating Stages:

#################
Generating Stages
#################

CurrentState
-------------
| The CurrentState stage fetches the current PlanningScene via the `get_planning_scene` service.
| This stage is often used at the beginning of the MTC task pipeline to set the start state from the current robot state.

Example code

.. code-block:: c++

  auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>("current_state");

FixedState
----------

| The FixedState stage spawns a pre-defined PlanningScene State.

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
  spawnObject(scene); // Add a CollisionObject to planning scene

  auto initial = std::make_unique<stages::FixedState>();
  initial->setState(scene);

Monitoring Generators
---------------------
Monitoring Generators help monitor and use solutions of another stage. 

GeneratePose
^^^^^^^^^^^^
GeneratePose is a monitoring generator stage which can be used to generate a pose based on solutions provided by the monitored stage.

GenerateGraspPose
^^^^^^^^^^^^^^^^^
| GenerateGraspPose stage is derived from GeneratePose, which is a monitoring generator.
| This stage can by used to generate poses for grasping by setting the desired attributes.

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
     - Object to grasp?
   * - angle_delta
     - void setAngleDelta(double delta)
     - Angular steps (rad). The target grasp pose is sampled around the object's z axis?
   * - pregrasp
     - void setPreGraspPose(std::string pregrasp)
     - Pregrasp pose
   * - pregrasp
     - void setPreGraspPose(moveit_msgs/RobotState pregrasp)
     - Pregrasp pose
   * - grasp
     - void setGraspPose(std::string grasp)
     - Grasp pose
   * - grasp
     - void setGraspPose(moveit_msgs/RobotState grasp)
     - Grasp pose
  
Example code 

.. code-block:: c++

  auto initial_stage = std::make_unique<stages::CurrentState>("current state");
  auto gengrasp = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
  gengrasp->setPreGraspPose("open");
  gengrasp->setObject("object");
  gengrasp->setAngleDelta(M_PI / 10.);
  gengrasp->setMonitoredStage(initial_stage);

| Additional MTC Grasp stages provided by MoveIt Studio - GenerateCuboidGraspPose and GenerateVacuumGraspPose
| GenerateCuboidGraspPose uses the grasp generation algorithms from MoveIt Grasps (https://github.com/ros-planning/moveit_grasps) for two finger grippers
| GenerateVacuumGraspPose generates grasps normal to planar surfaces for vacuum grippers.

GeneratePlacePose
^^^^^^^^^^^^^^^^^
| The GeneratePlacePose stage derives from GeneratePose, which is a monitoring generator.
| This stage generates poses for the place pipeline.
| Notice that whilst GenerateGraspPose spawns poses with an `angle_delta` interval, GeneratePlacePose samples a fixed amount, which is dependent on the objects shape.

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

GenerateRandomPose
^^^^^^^^^^^^^^^^^^
| The GenerateRandomPose stage derives from GeneratePose, which is a monitoring generator.
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
The FixedCartesianPose spawns a fixed Cartesian pose.
