How to run multiple planning pipelines in parallel with MoveItCpp?
==================================================================

MoveItCpp offers an API that allows:

1. Running multiple planning pipelines in parallel with different planners
2. Defining a custom stopping criterion that terminates the pipelines that haven't found a solution
3. Defining a custom function to select the most suitable solutions

Using multiple pipelines can be beneficial for example if:

- It is unsure which planner will produce the best solution for a given planning problem
- There is a chance that the preferred planner might fail and a backup solution should be available

A general introduction to moveit_cpp can be found here: :doc:`/doc/examples/moveit_cpp/moveitcpp_tutorial`.

Parallel Planning Interface
---------------------------

Using parallel planning with moveit_cpp is very similar to single pipeline planning except that a different implementation
of the planning component's :code:`plan(...)`` function is used:

.. code-block:: c++

    MotionPlanResponse PlanningComponent::plan(
      const MultiPipelinePlanRequestParameters& parameters,
      SolutionCallbackFunction solution_selection_callback,
      StoppingCriterionFunction stopping_criterion_callback)

This function tries to plan a trajectory from the a start state, to a goal state that satisfy a set of constraints. Based on the configuration
provided by the :code:`parameters` multiple threads are launched and each tries to solve the planning problem with a different planning pipeline. Once
all pipelines found a solution (Reminder: No solution is also a possible result), the :code:`solution_selection_callback` is called to determine which
solution is returned as :code:`MotionPlanResponse`. By default all pipelines use their time budget defined by the :code:`planning_time` field of the :code:`PlanRequestParameters`` but it is possible to terminate the parallel planning earlier by using the :code:`stopping_criterion_callback`. This function
is iteratively called during the parallel planning process and terminates pipelines that have not found a solution yet, if the stopping criterion is met.

Demo
----

The following demo serves as an example how you can configure and use MoveItCpp's parallel planning interface. First of all, let's
run the demo: ::

  ros2 launch moveit2_tutorials parallel_planning_example.launch.py

Let's take a look at the code:
You need to initialize moveit_cpp and a planning component that will solve you planning problem. Setting start and goal state is also did not change

.. code-block:: c++

    planning_component_->setGoal(*goal_state);
    planning_component_->setStartStateToCurrentState();

Next it is necessary to setup the `MultiPipelinePlanRequestParameters`.

.. code-block:: c++

    moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters multi_pipeline_plan_request{
      node_, { "ompl_rrtc", "pilz_lin", "chomp" }
    };

The constructor of this function will search initialize three `PlanningRequestParameter` configurations based on the config that is provided in the node's
parameter namespaces :code:`"ompl_rrtc"`, :code:`"pilz_lin"`, :code:`"chomp"`. To provide these you can simply extend the `moveit_cpp.yaml` file, as for example done for this demo:

.. code-block:: yaml

    # PlanRequestParameters for the first parallel pipeline
    ompl_rrtc:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: ompl
        planner_id: "RRTConnectkConfigDefault"
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0
        planning_time: 0.5

    # PlanRequestParameters for the second parallel pipeline
    pilz_lin:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: pilz_industrial_motion_planner
        planner_id: "LIN"
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0
        planning_time: 0.8

    # PlanRequestParameters for the third parallel pipeline
    chomp:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: chomp
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0
        planning_time: 1.5

Optionally, it is possible to define a stopping criterion and a solution selection function. If none are passed as an argument to the :code:`plan(...)`,
all pipelines use their complete planning time budget and afterwards the shortest path is chosen.

For this example we're using the default stopping criterion and a custom solution selection criterion that choses the fastest solution:

.. code-block:: c++

    planning_interface::MotionPlanResponse getFastestSolution(const std::vector<planning_interface::MotionPlanResponse>& solutions)
    {
      // Find trajectory with minimal path
      auto const fastest_trajectory = std::min_element(solutions.begin(), solutions.end(),
          [](const planning_interface::MotionPlanResponse& solution_a,
             const planning_interface::MotionPlanResponse& solution_b) {
            // If both solutions were successful, check which trajectory is faster
            if (solution_a && solution_b)
            {
              return *solution_a.trajectory_.getDuration() <
                     *solution_b.trajectory_.getDuration();
            }
            // If only solution a is successful, return a
            else if (solution_a)
            {
              return true;
            }
            // Else return solution b, either because it is successful or not
            return false;
          });
      return *fastest_trajectory;
    }

Here is an example for a custom stopping criterion:

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

Once :code:`MultiPipelinePlanRequestParameters` and optionally :code:`SolutionCallbackFunction` and/or :code:`StoppingCriterionFunction` are defined, we call :code:`plan(...)``:

.. code-block:: c++

    auto plan_solution = planning_component_->plan(multi_pipeline_plan_request, &getFastestSolution);

Tips
----

- When you want to use multiple pipelines with the same planner parallel it is recommended to initialize multiple planning pipelines in moveit_cpp rather than using the same one in multiple parallel planning requests
