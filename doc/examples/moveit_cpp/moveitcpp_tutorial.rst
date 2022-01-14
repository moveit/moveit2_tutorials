MoveItCpp Tutorial
==================================

Introduction
------------
MoveItCpp is a new high level interface, a unified C++ API that does not require the use of ROS Actions, Services, and Messages to access the core MoveIt functionality, and an alternative (not a full replacement) for the existing :doc:`MoveGroup API </doc/examples/move_group_interface/move_group_interface_tutorial>`, we recommend this interface for advanced users needing more realtime control or for industry applications. This interface has been developed at PickNik Robotics by necessity for our many commercial applications.

.. image:: images/moveitcpp_start.png
   :width: 300pt
   :align: center

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in :doc:`Getting Started </doc/tutorials/getting_started/getting_started>`.

Running the Code
----------------
Open a shell, run the launch file: ::

  ros2 launch moveit2_tutorials moveit_cpp_tutorial.launch.py

After a short moment, the RViz window should appear and look similar to the one at the top of this page. To progress through each demo step either press the **Next** button in the **RvizVisualToolsGui** panel at the bottom of the screen or select **Key Tool** in the **Tools** panel at the top of the screen and then press **0** on your keyboard while RViz is focused.

The Entire Code
---------------
The entire code can be seen :codedir:`here in the MoveIt GitHub project<examples/moveit_cpp/src/moveit_cpp_tutorial.cpp >`. Next we step through the code piece by piece to explain its functionality.

.. tutorial-formatter:: ./src/moveit_cpp_tutorial.cpp

The Launch File
---------------
The entire launch file is :codedir:`here<examples/moveit_cpp/launch/moveit_cpp_tutorial.launch.py>` on GitHub. All the code in this tutorial can be run from the **moveit2_tutorials** package that you have as part of your MoveIt setup.
