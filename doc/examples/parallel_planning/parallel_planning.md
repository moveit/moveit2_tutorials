How to customize path planning with MoveItCpp?
==============================================

This guide should help you to get the paths with MoveItCpp you want!

Learning Objectives
-------------------
- What is path (planning) quality?
- Composition of the MoveItCpp planning pipeline
- Practical tipps to customize the path quality

What is path (planning) quality?
--------------------------------
Internals of the MoveItCpp planning pipeline
--------------------------------------------
Use a distance aware IK solver
------------------------------

.. code-block:: yaml

    kinematics_solver: bio_ik/BioIKKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.005
    kinematics_solver_attempts: 1.0
    # optional bio_ik configuration parameters
    center_joints_weight: 1.0
    minimal_displacement_weight: 1.0
    avoid_joint_limits_weight: 1.0
    mode: "gd_c"

Plan with multiple planning pipelines in parallel and select the most suitable solution
---------------------------------------------------------------------------------------

.. code-block:: yaml

    one:
    plan_request_params:
      planning_attempts: 1
      planning_pipeline: ompl
      planner_id: "RRTConnectkConfigDefault"
      max_velocity_scaling_factor: 1.0
      max_acceleration_scaling_factor: 1.0

    two:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: ompl
        planner_id: "RRTConnectkConfigDefault"
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0

    three:
      plan_request_params:
        planning_attempts: 1
        planning_pipeline: ompl
        planner_id: "RRTConnectkConfigDefault"
        max_velocity_scaling_factor: 1.0
        max_acceleration_scaling_factor: 1.0

Run the demo:

.. code-block:: bash

    ros2 launch moveit2_tutorials parallel_planning_example.launch.py
