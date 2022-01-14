Move Group C++ Interface
==================================
.. image:: move_group_interface_tutorial_start_screen.png
   :width: 700px

In MoveIt, the simplest user interface is through the :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>` class. It provides easy to use functionality for most operations that a user may want to carry out, specifically setting joint or pose goals, creating motion plans, moving the robot, adding objects into the environment and attaching/detaching objects from the robot. This interface communicates over ROS topics, services, and actions to the :moveit_codedir:`MoveGroup<moveit_ros/move_group/src/move_group.cpp>` node.


Watch this quick `YouTube video demo <https://youtu.be/_5siHkFQPBQ>`_ to see the power of the move group interface!

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in :doc:`Getting Started </doc/tutorials/getting_started/getting_started>`.

Running the Code
----------------
Open two shells. In the first shell, start RViz and wait for everything to finish loading: ::

  ros2 launch moveit2_tutorials move_group.launch.py

In the second shell, run the launch file: ::

  ros2 launch moveit2_tutorials move_group_interface_tutorial.launch.py

After a short moment, the RViz window should appear and look similar to the one at the top of this page. To progress through each demo step either press the **Next** button in the **RvizVisualToolsGui** panel at the bottom of the screen or select **Key Tool** in the **Tools** panel at the top of the screen and then press **0** on your keyboard while RViz is focused.

Expected Output
---------------
See the `YouTube video <https://youtu.be/_5siHkFQPBQ>`_ at the top of this tutorial for expected output. In RViz, we should be able to see the following:
 1. The robot moves its arm to the pose goal to its front.
 2. The robot moves its arm to the joint goal at its side.
 3. The robot moves its arm back to a new pose goal while maintaining the end-effector level.
 4. The robot moves its arm along the desired Cartesian path (a triangle down, right, up+left).
 5. The robot moves its arm to a simple goal with no objects in the way.
 6. A box object is added into the environment to the right of the arm.
    |B|

 7. The robot moves its arm to the pose goal, avoiding collision with the box.
 8. The object is attached to the wrist (its color will change to purple/orange/green).
 9. The robot moves its arm with the attached object to the pose goal, avoiding collision with the box.
 10. The object is detached from the wrist (its color will change back to green).
 11. The object is removed from the environment.

.. |B| image:: ./move_group_interface_tutorial_robot_with_box.png

The Entire Code
---------------
The entire code can be seen :codedir:`here in the MoveIt GitHub project<examples/move_group_interface/src/move_group_interface_tutorial.cpp>`. Next, we step through the code piece by piece to explain its functionality.

.. tutorial-formatter:: ./src/move_group_interface_tutorial.cpp

The Launch File
---------------
The entire launch file is :codedir:`here<examples/move_group_interface/launch/move_group_interface_tutorial.launch.py>` on GitHub. All the code in this tutorial can be run from the **moveit2_tutorials** package that you have as part of your MoveIt setup.


A Note on Setting Tolerances
----------------------------
Note that the `MoveGroupInterface's <https://github.com/ros-planning/moveit2/blob/ed844d4b46f70ed6e97d0c1f971ab2b9a45f156d/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h#L293>`_ *setGoalTolerance()* and related methods sets the tolerance for **planning**, not execution.

If you want to configure the execution tolerances, you will have to edit the *controller.yaml* file if using a FollowJointTrajectory controller, or manually add it into the generated trajectory message from the planner.
