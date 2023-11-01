.. _Wrappers:

########
Wrappers
########

ComputeIK
---------

The ComputeIK is a wrapper for any pose generator stage to compute the inverse kinematics for a pose in Cartesian space.
Link to GeneratePose stage

Usually the end effector's parent link or the group's tip link is used as the inverse kinematics frame, which should be moved to the goal frame.
However, any other inverse kinematics frame can be defined (which is linked to the tip of the group)

.. list-table:: Properties to be set by user
   :widths: 25 100 80
   :header-rows: 1

   * - Property Name
     - Function to set property
     - Description
   * - eef
     - void setEndEffector(std::string eef)
     - Name of end effector group
   * - group
     - void setGroup(std::string group)
     - Name of active group. Derived from eef if not provided.
   * - max_ik_solutions
     - void setMaxIKSolutions(uint32_t n)
     - Default is 1
   * - ignore_collisions
     - void setIgnoreCollisions(bool flag)
     - Default is false.
   * - min_solution_distance
     - void setMinSolutionDistance(double distance)
     - Minimum distance between separate IK solutions for the same target. Default is 0.1.

Code Example

.. code-block:: c++

  auto stage = std::make_unique<moveit::task_constructor::stages::GenerateVacuumGraspPose>("generate pose");
  auto wrapper = std::make_unique<moveit::task_constructor::stages::ComputeIK>("pose IK", std::move(stage));
  wrapper->setTimeout(0.05);
  wrapper->setIKFrame("tool_frame");
  wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "eef", "group" });
  wrapper->properties().configureInitFrom(moveit::task_constructor::Stage::INTERFACE, { "target_pose" });

  // Add callback to publish grasp solutions so they can be displayed within the UI
  wrapper->addSolutionCallback(
      [this](const moveit::task_constructor::SolutionBase& solution) { return onGraspSolution(solution); });

PredicateFilter
---------------
PredicateFilter is a stage wrapper to filter generated solutions by custom criteria.

.. list-table:: Properties to be set by user
   :widths: 25 100 80
   :header-rows: 1

   * - Property Name
     - Function to set property
     - Description
   * - predicate
     - void setPredicate(std::function<bool(const SolutionBase, std::string)> predicate)
     - Predicate to filter solutions
   * - ignore_filter
     - void setIgnoreFilter(bool ignore)
     - Ignore predicate and forward all solutions

Code Example

.. code-block:: c++

  auto current_state = std::make_unique<moveit::task_constructor::stages::CurrentState>(kStageNameCurrentState);

  // Use Predicate filter to fail the MTC task if any links are in collision in planning scene
  auto applicability_filter =
      std::make_unique<moveit::task_constructor::stages::PredicateFilter>("current state", std::move(current_state));

  applicability_filter->setPredicate([this](const moveit::task_constructor::SolutionBase& s, std::string& comment) {
    if (s.start()->scene()->isStateColliding())
    {
      // Get links that are in collision
      std::vector<std::string> colliding_links;
      s.start()->scene()->getCollidingLinks(colliding_links);

      // Publish the results
      publishLinkCollisions(colliding_links);

      comment = "Links are in collision";
      return false;
    }
    return true;
  });

PassThrough
-----------
PassThrough is a generic wrapper that passes on a solution.
This is useful to set a custom CostTransform via Stage::setCostTerm to change a solutions's cost without losing the original value.
