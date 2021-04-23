Getting Started
===============

This tutorial will install MoveIt2 and create a workspace sandbox to run the tutorials and example robot.

Install ROS2 and Colcon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Install ROS2 Foxy <https://docs.ros.org/en/foxy/Installation.html>`_.
It is easy to miss steps when going through the ROS2 installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS2 correctly.

Install `rosdep2 <http://wiki.ros.org/rosdep>`_ to install system dependencies : ::

  sudo apt install python3-rosdep2

Once you have ROS2 installed, make sure you have the most up to date packages: ::

  rosdep update
  sudo apt update
  sudo apt dist-upgrade

Install `Colcon <https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon>`_ the ROS2 build system: ::

  sudo apt install python3-colcon-common-extensions

Install `vcstool <https://index.ros.org/d/python3-vcstool/>`_ : ::

  sudo apt install python3-vcstool

Create A Colcon Workspace and Download MoveIt2 Source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
These tutorials rely on the master branch of MoveIt2, which requires a build from source.
You will need to have a `colcon <https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon>`_ workspace setup: ::

  mkdir -p ~/ws_moveit2/src
  cd ~/ws_moveit2/src

  wget https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos
  vcs import < moveit2.repos
  wget https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/moveit2_tutorials.repos
  vcs import < moveit2_tutorials.repos

Build your Colcon Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from Debian any package dependencies not already in your workspace: ::

  cd ~/ws_moveit2/src
  rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y

The next command will configure your colcon workspace: ::

  cd ~/ws_moveit2
  colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

Source the colcon workspace: ::

  source ~/ws_moveit2/install/setup.bash

Optional: add the previous command to your ``.bashrc``: ::

   echo 'source ~/ws_moveit2/install/setup.bash' >> ~/.bashrc

.. note:: Sourcing the ``setup.bash`` automatically in your ``~/.bashrc`` is
   not required and often skipped by advanced users who use more than one
   colcon workspace at a time, but we recommend it for simplicity.

Next Step
^^^^^^^^^^
`Visualize a robot with the interactive motion planning plugin for RViz <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_
