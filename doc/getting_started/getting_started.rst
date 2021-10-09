Getting Started
===============

This tutorial will install MoveIt 2 and create a workspace sandbox to run the tutorials and example robot.

Install ROS 2 and Colcon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`Install ROS 2 Foxy <https://docs.ros.org/en/foxy/Installation.html>`_.
It is easy to miss steps when going through the ROS 2 installation tutorial. If you run into errors in the next few steps, a good place to start is to go back and make sure you have installed ROS 2 correctly.

Install `rosdep <http://wiki.ros.org/rosdep>`_ to install system dependencies : ::

  sudo apt install python3-rosdep

Once you have ROS 2 installed, make sure you have the most up to date packages: ::

  rosdep update
  sudo apt update
  sudo apt dist-upgrade

Install `Colcon <https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon>`_ the ROS 2 build system: ::

  sudo apt install python3-colcon-common-extensions

Install `vcstool <https://index.ros.org/d/python3-vcstool/>`_ : ::

  sudo apt install python3-vcstool

Create A Colcon Workspace and Download MoveIt 2 Source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
These tutorials rely on the ROS 2 project MoveIt 2, which can be built from source or installed from binaries. Please make sure you install or build MoveIt 2 for your distribution of choice by following the instructions from `the official MoveIt website <https://moveit.ros.org/install-moveit2/source/>`_.
For tutorials you will need to have a `colcon <https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#install-colcon>`_ workspace setup. If you already have a colcon workspace by building MoveIt 2 from source skip the following, else create a workspace: ::

  mkdir -p ~/ws_moveit2/src

Download MoveIt 2 Tutorials Source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Move into your colcon workspace and pull the MoveIt 2 tutorials source: ::

  cd ~/ws_moveit2/src
  git clone https://github.com/ros-planning/moveit2_tutorials.git
  vcs import < moveit2_tutorials/moveit2_tutorials.repos

Build your Colcon Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following will install from Debian any package dependencies not already in your workspace: ::

  rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

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
