======================
Planning Scene Monitor
======================

The ``planning scene`` is an object used for storing the representation of the world around the robot and also the state of the robot itself.
The internal state of the ``planning_scene`` object is typically maintained by a ``planning_scene_monitor`` component that enables reading and writing the state in a thread-safe manner.

Particularly, the ``move_group`` node as well as the rviz planning scene plugin maintain their own Planning Scene Monitor (PSM).
The ``move_group`` node's PSM *listens* to the topic ``/planning_scene`` and *publishes* its planning scene state to the topic ``monitored_planning_scene``.
The latter is listened to by the rviz planning scene plugin.

.. image:: /_static/images/planning_scene_monitor.svg

World Geometry Monitor
----------------------

The world geometry monitor builds world geometry using information from the sensors on the robot such as LIDARs or depth cameras and from user input.
It uses the ``occupancy map monitor`` described below to build a 3D representation of the environment around the robot and augments that with information on the ``planning_scene`` topic for adding object information.

3D Perception
-------------

3D perception in MoveIt is handled by the ``occupancy map monitor``.
The occupancy map monitor uses a plugin architecture to handle different kinds of sensor input as shown in the Figure above.
In particular, MoveIt has inbuilt support for handling two kinds of inputs:

- **Point clouds**: handled by the ``PointCloudOccupancyMapUpdater`` plugin.

- **Depth images**: handled by the ``DepthImageOccupancyMapUpdater`` plugin.

Note that you can add your own types of updaters as a plugin to the occupancy map monitor.

Octomap
-------

The Occupancy map monitor uses an `Octomap <https://octomap.github.io/>`_ to maintain the occupancy map of the environment.
The Octomap can actually encode probabilistic information about individual cells although this information is not currently used in MoveIt.
The Octomap can directly be passed into FCL, the collision checking library that MoveIt uses.

Depth Image Occupancy Map Updater
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The depth image occupancy map updater includes its own *self-filter*, i.e. it will remove visible parts of the robot from the depth map.
It uses current information about the robot (the robot state) to carry out this operation.
