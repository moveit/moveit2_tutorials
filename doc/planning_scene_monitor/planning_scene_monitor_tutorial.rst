:moveit1:

..
   Once updated for MoveIt 2, remove all lines above title (including this comment and :moveit1: tag)

Planning Scene Monitor
==================================

The :moveit_codedir:`PlanningSceneMonitor<moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h>` is the recommended interface for maintaining an up-to-date planning scene. The relationship between :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.h>`, :moveit_codedir:`CurrentStateMonitor<moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/current_state_monitor.h>`, :moveit_codedir:`PlanningScene<moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h>`, :moveit_codedir:`PlanningSceneMonitor<moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h>`, and :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>` can be really confusing at first. This tutorial aims to make clear these key concepts.

RobotState
----------
The :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.h>` is a snapshot of a robot. It contains the :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>` and a set of joint values.

CurrentStateMonitor
-------------------
The :moveit_codedir:`CurrentStateMonitor<moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/current_state_monitor.h>` (CSM) can be thought of as a ROS wrapper for the RobotState. It subscribes to a provided topic for :common_interfaces_codedir:`JointState<sensor_msgs/msg/JointState.msg>` messages that provide up-to-date sensor values for single degree of freedom actuators, such as revolute or prismatic joints, and updates its internal RobotState with those joint values. In addition to the single degree of freedom joints, a robot can have joints with multiple degrees of freedom such as floating and planar joints. To maintain up-to-date transform information for links and other frames attached with multiple-degree-of-freedom joints, the CSM stores a TF2 :ros_documentation:`Buffer <Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html#cppsrvcli>` that uses a TF2 :ros_documentation:`TransformListener <Tutorials/Tf2/Writing-A-Tf2-Listener-Cpp.html#writingatf2listenercpp>` to set their transforms in its internal data.

PlanningScene
-------------
The :moveit_codedir:`PlanningScene<moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h>` is a snapshot of the world that includes both the RobotState and any number of collision objects. The Planning Scene can be used for collision checking as well as getting information about the environment.

PlanningSceneMonitor
--------------------
The :moveit_codedir:`PlanningSceneMonitor<moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h>` wraps a PlanningScene with ROS interfaces for keeping the PlanningScene up to date. To access the PlanningSceneMonitor's underlying PlanningScene, use the provided :moveit_codedir:`LockedPlanningSceneRW<moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h>` and :moveit_codedir:`LockedPlanningSceneRO<moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h>` classes.

The PlanningSceneMonitor has the following objects, which have their own ROS interfaces for keeping sub-components of the planning scene up to date:

 * A :moveit_codedir:`CurrentStateMonitor<moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/current_state_monitor.h>` for tracking updates to the RobotState via a ``robot_state_subscriber_`` and a ``tf_buffer_``, as well as a planning scene subscriber for listening to planning scene diffs from other publishers.
 * An OccupancyMapMonitor for tracking updates to an OccupancyMap via ROS topics and services.

The PlanningSceneMonitor has the following subscribers:

 * ``collision_object_subscriber_`` - Listens to a provided topic for :moveit_msgs_codedir:`CollisionObject<msg/CollisionObject.msg>` messages that might add, remove, or modify collision objects in the planning scene and passes them into its monitored planning scene
 * ``planning_scene_world_subscriber_`` - Listens to a provided topic for :moveit_msgs_codedir:`PlanningSceneWorld<msg/PlanningSceneWorld.msg>` messages that may contain collision object information and/or octomap information. This is useful for keeping planning scene monitors in sync
 * ``attached_collision_object_subscriber_`` - Listens on a provided topic for :moveit_msgs_codedir:`AttachedCollisionObject<msg/AttachedCollisionObject.msg>` messages that determine the attaching/detaching of objects to links in the robot state.

The PlanningSceneMonitor has the following services:

 * ``get_scene_service_`` - Which is an optional service to get the full planning scene state.

The PlanningSceneMonitor is initialized with:

 * ``startSceneMonitor`` - Which starts the ``planning_scene_subscriber_``,
 * ``startWorldGeometryMonitor`` - Which starts the ``collision_object_subscriber_``, the ``planning_scene_world_subscriber_``, and the OccupancyMapMonitor,
 * ``startStateMonitor`` - Which starts the CurrentStateMonitor and the ``attached_collision_object_subscriber_``,
 * ``startPublishingPlanningScene`` - Which starts another thread for publishing the entire planning scene on a provided topic for other PlanningSceneMonitors to subscribe to, and
 * ``providePlanningSceneService`` - Which starts the ``get_scene_service_``.

PlanningSceneInterface
----------------------
The :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>` is a useful class for publishing updates to a MoveGroup's :moveit_codedir:`PlanningSceneMonitor<moveit_ros/planning/planning_scene_monitor/include/moveit/planning_scene_monitor/planning_scene_monitor.h>` through a C++ API without creating your own subscribers and service clients. It may not work without MoveGroup or MoveItCpp.
