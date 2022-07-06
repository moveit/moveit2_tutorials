Teleoperation with a Controller
===============================

This tutorial will introduce you to using a controller to move the panda arm.

Prerequisites
-------------
Explain pubs, subs, and services used as well as convert joy cmd and relevant portion of launch file

If you haven't already done so, make sure you've completed the steps in :doc:`Visualizing in RViz </doc/tutorials/visualizing_in_rviz/visualizing_in_rviz>`.
This project assumes you are starting with the ``hello_moveit`` project, where the previous tutorial left off.

This guide explains how to write tutorials for the MoveIt documentation.
Tutorials are one of the most useful contributions you can make because they are the first thing many new users see.
This guide is intended for any contributor who wants to submit a new tutorial.
There are many additional quality standards and how-tos for contributing to the tutorials located in this repository's `README <https://github.com/ros-planning/moveit2_tutorials/blob/main/README.md>`_.

Requirements
------------
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- MoveIt 2 Tutorials
- `ROS2 joy <https://index.ros.org/p/joy/>`_

Steps
-----

1. Build the MoveIt2 workspace

  First, ``cd`` to the root directory of the moveit2 workspace. (if you followed the :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` tutorial, this will be ``~/ws_moveit/``).

  Then, run ``colcon build``.

2. Plug in your controller adapter.
3. Launch the ``moveit_servo`` example file.

  Run ``ros2 launch moveit_servo servo_example.launch.py``

4. Move the arm around, using the below image as a guide.

  .. image:: xboxcontroller.png
    :width: 600px

Launch File
-----------

The file that launches this example is
``ws_moveit2/src/moveit2/moveit_ros/moveit_servo/launch/servo_example.launch.py``

This launch file launches everything needed for the panda arm planning, and also launches the ``joy`` node and the ``JoyToServoPub`` node (which is explained below).

Of primary interest is the section of code that launches the joy and ``JoyToServoPub`` nodes.
They are both created as ``ComposableNode``\s. More information about ``ComposableNode``\s can be found `here <https://roscon.ros.org/2019/talks/roscon2019_composablenodes.pdf>`__ and `here <https://medium.com/@waleedmansoor/understanding-ros-nodelets-c43a11c8169e>`__.

.. code-block:: python

  ComposableNode(
      package="moveit_servo",
      plugin="moveit_servo::JoyToServoPub",
      name="controller_to_servo_node",
  ),
  ComposableNode(
      package="joy",
      plugin="joy::Joy",
      name="joy_node",
  )


test
JoyToServoPub
-------------

The node that turns controller inputs into servo commands is
``ws_moveit2/src/moveit2/moveit_ros/moveit_servo/src/teleop_demo/joystick_servo_example.cpp``

This node subscribes to the joy node (which publishes messages giving the state of the controller). It publishes ``TwistStamped`` messages, ``JointJog`` messages, and ``PlanningScene`` messages.

The ``PlanningScene`` message is only published once, when the JoyToServoPub is first constructed. It simply adds some obstacles into the planning scene.

The difference between the ``JointJog`` and ``TwistStamped`` messages is
that the inverse kinematic solver moves the joints to achieve the end
effector motions defined by the ``TwistStamped`` messages, while the
``JointJog`` messages directly move individual joints.

The ``joyCB`` function is called when a message is published to the ``joy``
topic, and translates the button presses from the controller into commands
for the arm. If both ``JointJog`` and ``TwistStamped`` messages would be
published by the inputs, only ``JointJog`` messages are published.
