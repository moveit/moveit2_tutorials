Your First C++ MoveIt Project
=============================

This tutorial will quickly get you writing your first ROS project with MoveIt.

Prerequisites
-------------

If you haven't already done so, make sure you've completed the steps in :doc:`Getting Started </doc/tutorials/getting_started/getting_started>`.

This tutorial assumes you understand the basics of ROS 2.
To prepare yourself for this please complete the `Official ROS 2 Tutorials <https://docs.ros.org/en/{DISTRO}/Tutorials.html>`_ up until "Writing a simple publisher and Subscriber (C++)".

Steps
-----

1 Create a package
^^^^^^^^^^^^^^^^^^

Open a terminal and `source your ROS 2 installation <https://docs.ros.org/en/{DISTRO}/Tutorials/Configuring-ROS2-Environment.html>`_ so that ``ros2`` commands will work.

Navigate to your ``ws_moveit2`` directory you created in the :doc:`Getting Started Tutorial </doc/tutorials/getting_started/getting_started>`.

Change directory into the ``src`` directory, as that is where we put our source code.

Create a new package with the ROS 2 command line tools:

.. code-block:: bash

  ros2 pkg create \
   --build-type ament_cmake \
   --dependencies moveit_ros_planning_interface rclcpp \
   --node-name hello_moveit hello_moveit

The output of this will show that it created some files in a new directory.

Note that we added ``moveit_ros_planning_interface`` and ``rclcpp`` as dependencies.
This will create the necessary changes in the ``package.xml`` and ``CMakeLists.txt`` files so that we can depend on these two packages.

Open the new source file created for you at ``ws_moveit2/src/hello_moveit/src/hello_moveit.cpp`` in your favorite editor.

2 Create a ROS Node and Executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This first bit is a bit of boilerplate but you should be used to seeing this from the ROS 2 tutorials.

.. code-block:: C++

  #include <memory>
  #include <thread>

  #include <rclcpp/rclcpp.hpp>
  #include <moveit/move_group_interface/move_group_interface.h>

  int main(int argc, char * argv[])
  {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create a thread to run to spin a Executor
    auto thread = std::thread([node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      executor.spin();
    });

    // Next step goes here

    // Shutdown ROS and our thread
    rclcpp::shutdown();
    thread.join();
    return 0;
  }

2.1 Build and Run
~~~~~~~~~~~~~~~~~

We will build and run the program to see that everything is right before we move on.

Change directory back to the workspace directory ``ws_moveit2`` and run this command:

.. code-block:: bash

  colcon build --mixin debug

After this succeeds, **open a new terminal**, then source the workspace environment script in that new terminal so that we can run our program.

.. code-block:: bash

  source install/setup.bash

Run your program and see the output.

.. code-block:: bash

  ros2 run hello_moveit hello_moveit

You will probably see this error:

.. code-block::

  terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
    what():  the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called., at /tmp/binarydeb/ros-rolling-rcl-4.0.0/src/rcl/guard_condition.c:67
  [ros2run]: Aborted

This is because ``rclcpp::shutdown()`` was called before the Executor finished initializing.
We'll fix that in the next section by adding some code that does something after creating the thread.

2.2 Examine the code
~~~~~~~~~~~~~~~~~~~~

The headers included at the top are just some standard C++ headers and the header for ROS and MoveIt that we will use later.

After that we have the normal call to initialize rclcpp and then we create our Node.

.. code-block:: C++

  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

The first argument is the string that ROS will use to make a unique node.
The second is needed for MoveIt because of how we use ROS Parameters.

Before we can start use our ROS Node we have to give it to a spinning executor.
This is what will enable ROS to call callbacks to update our Robot State among other things.

.. code-block:: C++

  auto thread = std::thread([node]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

Lastly we have the code to shutdown ROS and cleanup our thread.

3 Plan and Execute using MoveGroupInterface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After the creation of the thread add this block of code:

.. code-block:: C++

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

3.1 Build and Run
~~~~~~~~~~~~~~~~~

Just like before we need to build the code before we can run it.

In the workspace directory ``ws_moveit2`` run this command:

.. code-block:: bash

  colcon build --mixin debug

After this succeeds, we need to re-use the demo launch file from the previous tutorial to start RViz and the MoveGroup node.
In a separate terminal, source the workspace and then execute this:

.. code-block:: bash

  ros2 launch moveit2_tutorials demo.launch.py

Then in the ``Displays`` window under ``MotionPlanning/Planning Request`` uncheck the box ``Query Goal State``.

.. image:: rviz_1.png
   :width: 300px

In a third terminal source the workspace and run your program.

.. code-block:: bash

  ros2 run hello_moveit hello_moveit

This should cause the robot in RViz to move and end up in this pose:

.. image:: rviz_2.png
   :width: 300px

3.2 Examine the code
~~~~~~~~~~~~~~~~~~~~

The first thing we do is create the MoveGroupInterface.
Note that this is the only mutable object (other than the thread) that we create in this program.

.. code-block:: C++

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

Then we set our target pose and plan.

.. code-block:: C++

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

Finally we execute our plan if planning was successful, otherwise we log an error:

.. code-block:: C++

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

Summary
-------

You created a ROS 2 package and wrote your first program using MoveIt.


Further Reading
---------------

- We used lambdas to be able to initialize objects as constants. This is known as a technique called IIFE.  `Read more about this pattern from C++ Stories <https://www.cppstories.com/2016/11/iife-for-complex-initialization/>`_.
- We also declared everything we could as const.  `Read more about the usefulness of const here <https://www.cppstories.com/2016/12/please-declare-your-variables-as-const/>`_.

Next Step
---------

TODO
