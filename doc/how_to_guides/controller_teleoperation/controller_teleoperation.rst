How to Teleoperate a Robotic Arm with a Gamepad
===============================================

This guide will introduce you to using a gamepad to move the panda arm.

Prerequisites
-------------
Make sure your workspace has the required packages installed and that you
have a gamepad supported by ROS2 joy. This can be tested by launching a
``joy`` node and then running ``ros2 topic echo /joy`` to ensure your
gamepad is detected by ``joy``.

Requirements
------------
- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- MoveIt 2 Tutorials
- `ROS2 joy <https://index.ros.org/p/joy/>`_

Steps
-----

1. Build the MoveIt 2 workspace

  First, ``cd`` to the root directory of the moveit2 workspace. (if you followed the :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` tutorial, this will be ``~/ws_moveit/``).

  Then, run ``colcon build``.

2. Plug in your gamepad.
3. Source the install script and run the ``moveit_servo`` example file.

  Run ``source install/setup.bash``, then ``ros2 launch moveit_servo servo_example.launch.py``

4. Move the arm around, using the below image as a guide.

  .. image:: xboxcontroller.png
    :width: 600px

The drawio document can be seen `here <https://drive.google.com/file/d/1Hr3ZLvkYo0y0fA3Qb1Nk_y7wag4UO8Al/view?usp=sharing>`__.

Explanation
-----------

This section explains the launch file and the node that translates gamepad inputs to motion commands.

Launch File
^^^^^^^^^^^

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

JoyToServoPub
^^^^^^^^^^^^^

The node that translates gamepad inputs to motion commands is
``ws_moveit2/src/moveit2/moveit_ros/moveit_servo/src/teleop_demo/joystick_servo_example.cpp``

This node subscribes to the joy node (which publishes messages giving the state of the gamepad). It publishes ``TwistStamped`` messages, ``JointJog`` messages, and ``PlanningScene`` messages.

The ``PlanningScene`` message is only published once, when the JoyToServoPub is first constructed. It simply adds some obstacles into the planning scene.

The difference between the ``JointJog`` and ``TwistStamped`` messages is
that the inverse kinematic solver moves the joints to achieve the end
effector motions defined by the ``TwistStamped`` messages, while the
``JointJog`` messages directly move individual joints.

The ``joyCB`` function is called when a message is published to the ``joy``
topic, and translates the button presses from the gamepad into commands
for the arm. If both ``JointJog`` and ``TwistStamped`` messages would be
published by the inputs, only ``JointJog`` messages are published.
