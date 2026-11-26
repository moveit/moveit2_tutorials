Running Multiple Planning Pipelines in Parallel with MoveItCpp
==============================================================

MoveItCpp offers an API that allows:

1. Running multiple planning pipelines in parallel with different planners
2. Defining a custom stopping criterion that terminates the pipelines that haven't found a solution
3. Defining a custom function to select the most suitable solutions

Using multiple pipelines can be beneficial for several reasons, including:

- The planner that will produce the best solution is not known a priori
- There is a chance that the preferred planner might fail and a backup solution should be available

A general introduction to MoveItCpp can be found in the :doc:`/doc/examples/moveit_cpp/moveitcpp_tutorial`.

Parallel Planning Interface
---------------------------

Using parallel planning with MoveItCpp is similar to single pipeline planning, except that a different implementation
of the planning component's :code:`plan(...)` function is used:

.. code-block:: c++

    MotionPlanResponse PlanningComponent::plan(
      const MultiPipelinePlanRequestParameters& parameters,
      SolutionCallbackFunction solution_selection_callback,
      StoppingCriterionFunction stopping_criterion_callback)

This function tries to plan a trajectory from a start state to a goal state that satisfies a set of constraints. Based on the configuration
provided by the :code:`parameters`, multiple threads are launched and each tries to solve the planning problem with a different planning pipeline. Please keep in mind, that no solution is also a possible result. Once
all pipelines have been terminated. the :code:`solution_selection_callback` is called to determine which
solution is returned as :code:`MotionPlanResponse`. By default, all pipelines use their time budget defined by the :code:`planning_time` field of the :code:`MultiPipelinePlanRequestParameters`, but it is possible to terminate the parallel planning earlier by using the :code:`stopping_criterion_callback`. This function
is called whenever a pipeline produces a solution during the parallel planning process and, if the stopping criterion is met, terminates pipelines that have not found a solution yet.

Example
-------

The following demo shows how you can configure and use MoveItCpp's parallel planning interface. First,
run the demo: ::

  ros2 launch moveit2_tutorials parallel_planning_example.launch.py

A complex kitchen scene is loaded and two planning problems are solved. The first one is a small motion of the end effector towards the ground. This problem is likely to be solved by all three
planners, but with significant differences in the planning time. The second problem is much harder and most likely only the :code:`RRTConnect` planner will succeed. This demo
suggests that a well-configured parallel planning setup is versatile, and can be used in a broad variety of motion planning problems.

What code is necessary to use parallel planning?
------------------------------------------------

First, you need to initialize :code:`moveit_cpp` and a planning component that will solve your planning problems. Next, you need to set start state and goal constraints:

.. code-block:: c++

    planning_component_->setGoal(*goal_state);
    planning_component_->setStartStateToCurrentState();

Additionally, it is necessary to set up the :code:`MultiPipelinePlanRequestParameters`.

.. code-block:: c++

    moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters multi_pipeline_plan_request{
      node_, { "ompl_rrtc", "pilz_lin", "chomp" }
    };

The constructor of this class will initialize multiple :code:`PlanningRequestParameter` class members based on the config that is provided in the node's
parameter namespaces :code:`"ompl_rrtc"`, :code:`"pilz_lin"`, and :code:`"chomp"`. To provide these, you can extend the :code:`moveit_cpp.yaml` file:

.. code-block:: yaml

    # PlanRequestParameters for the first parallel pipeline that uses OMPL - RRTConnect
    ompl_rrtc:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: ompl
        planner_id: "RRTConnectkConfigDefault"
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0
        planning_time: 0.5

    # PlanRequestParameters for a second parallel pipeline using Pilz with the LIN planner
    pilz_lin:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: pilz_industrial_motion_planner
        planner_id: "LIN"
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0
        planning_time: 0.8

    # PlanRequestParameters for a third parallel pipeline that uses CHOMP as planner
    chomp:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: chomp
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0
        planning_time: 1.5

  # Another OMPL planner using a second OMPL pipeline named 'ompl_rrt_star'
  ompl_rrt_star:
    plan_request_params:
      planning_attempts: 1
      planning_pipeline: ompl_rrt_star # Different OMPL pipeline name!
      planner_id: "PRMkConfigDefault"
      max_velocity_scaling_factor: 1.0
      max_acceleration_scaling_factor: 1.0
      planning_time: 1.5

Optionally, it is possible to define a custom stopping criterion and/or solution selection function. If none are passed as an argument to :code:`plan(...)`,
all pipelines use their complete planning time budget, and afterwards the shortest path is chosen.

For this example, we're using the default stopping criterion and a solution selection criterion that chooses the shortest solution:

.. code-block:: c++

    planning_interface::MotionPlanResponse getShortestSolution(const std::vector<planning_interface::MotionPlanResponse>& solutions)
    {
      // Find trajectory with minimal path
      auto const shortest_solution = std::min_element(solutions.begin(), solutions.end(),
        [](const planning_interface::MotionPlanResponse& solution_a,
           const planning_interface::MotionPlanResponse& solution_b) {
          // If both solutions were successful, check which path is shorter
          if (solution_a && solution_b)
          {
            return robot_trajectory::pathLength(*solution_a.trajectory_) <
                   robot_trajectory::pathLength(*solution_b.trajectory_);
          }
          // If only solution a is successful, return a
          else if (solution_a)
          {
            return true;
          }
          // Else return solution b, either because it is successful or not
          return false;
        });
      return *shortest_solution;
    }

Here is an example of a custom stopping criterion, that terminates the other planning pipelines as soon as :code:`RRTConnect` finds a solution:

.. code-block:: c++

    // Stop parallel planning as soon as RRTConnect finds a solution
    bool stoppingCriterion(
        moveit_cpp::PlanningComponent::PlanSolutions const& plan_solutions,
        moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters const& plan_request_parameters)
    {
      // Read solutions that are found up to this point from a thread safe storage
      auto const& solutions = plan_solutions.getSolutions();

      // Stop parallel planning if the pipeline using RRTConnect finds a solution
      for (auto const& solution : solutions)
      {
          if (solution.planner_id_ == "RRTConnectkConfigDefault")
          {
            // Return true to abort the other pipelines
            return true;
          }
      }
      // Return false when parallel planning should continue
      return false;
    }

Once :code:`MultiPipelinePlanRequestParameters` and optionally :code:`SolutionCallbackFunction` and/or :code:`StoppingCriterionFunction` are defined, we call :code:`plan(...)`:

.. code-block:: c++

    auto plan_solution = planning_component_->plan(multi_pipeline_plan_request, &getShortestSolution);

Tips
----

- When you want to use different planners of the same pipeline (e.g. Pilz planner with PTP and LIN) in parallel, it is recommended to initialize multiple planning pipelines in MoveItCpp rather than using the same one in multiple parallel planning requests. In this example two OMPL pipelines are loaded.
