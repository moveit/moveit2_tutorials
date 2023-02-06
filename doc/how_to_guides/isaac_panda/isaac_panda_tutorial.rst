How To Command Simulated Isaac Robot
====================================

This tutorial requires a machine with ``Isaac 2022.2.0`` installed. For requirements and installation please see the `Omniverse documentation <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html>`_

This tutorial has the following assumptions on system configuration:

1. Isacc 2022.2.0 is installed on a Ubuntu 20.04 host in the "$HOME/.local/share/ov/pkg/isaac_sim-2022.2.0" directory
2. Docker is installed
3. You clone this repo so that you can build a Ubuntu 22.04 Humble based Docker image that can communicate with Isaac and run this tutorial

Introduction to ros2_control
----------------------------

One of the recommeded ways to execute trajectories calculated by MoveIt is to use the `ros2_control <https://control.ros.org/master/index.html>`_
framework to manage and communnicate with your robot, real or simulated. It comes highly recommended because it offses a single API that
allows your software to switch between many different robot types and the features they have built in by simply changing some launch arguments.
For example if we inspect the Panda Robot's ``ros2_control.xacro`` we can see it uses a flag ``use_fake_hardware`` to switch between being simulated or connecting to a physical robot.

.. code-block:: XML

    <hardware>
      <xacro:if value="${use_fake_hardware}">
        <plugin>mock_components/GenericSystem</plugin>
      </xacro:if>
      <xacro:unless value="${use_fake_hardware}">
        <plugin>franka_hardware/FrankaHardwareInterface</plugin>
        <param name="robot_ip">${robot_ip}</param>
      </xacro:unless>
    </hardware>


`Hardware Components <https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-components>`_ can be of different types, but
the plugin ``<plugin>fake_components/GenericSystem</plugin>`` is just a simple system that forwards the incomming commands to the tracked state of the joints (I.E. perfect control of the hardware).

For us to expand our Panda robot to Isaac Sim we first have to introduce `isaac_ros2_control <https://github.com/PickNikRobotics/isaac_ros2_control>`_.
This Hardware Interface is a very simple ros2_control System that subsrcibes and publishes on a configured topic.
For this tutorial the topic ``/isaac_joint_states`` will be the Systems current state and the topic ``/isaac_joint_commands`` will be used to actuate it.
The ``moveit_resources`` ``panda_moveit_config <https://github.com/ros-planning/moveit_resources/blob/humble/panda_moveit_config/config/panda.ros2_control.xacro#L7>`` we are using in this tutorial does not support
connecting to hardware so our ``ros2_control.xacro`` is now updated to load the ``isaac_ros2_control`` hardware when the flag ``ros2_control_hardware_type`` is set to ``isaac``.

.. code-block:: XML

    <xacro:if value="${ros2_control_hardware_type == 'mock_components'}">
        <plugin>mock_components/GenericSystem</plugin>
    </xacro:if>
    <xacro:if value="${ros2_control_hardware_type == 'isaac'}">
        <plugin>isaac_ros2_control/IsaacSystem</plugin>
        <param name="joint_commands_topic">/isaac_joint_commands</param>
        <param name="joint_states_topic">/isaac_joint_states</param>
    </xacro:if>


To learn about configuring your robot to communicate with ROS2 please see the `Joint Control tutorial <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_manipulation.html>`_ on Omniverse.

Computer Setup
--------------

1. Install `Isaac Sim <https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html>`_

2. For Rviz to run inside Docker we have to allow it to display applications

.. note:: this step is only required once every host machine login

.. code-block:: bash

  xhost +local:docker

3. Clone the MoveIt2 Tutorials repo

.. code-block:: bash

  git clone https://github.com/MarqRazz/moveit2_tutorials.git -b pr-isaac_tutorial
  # git clone https://github.com/ros-planning/moveit2_tutorials.git -b humble

4. Cd into the path you cloned the tutorials and then switch to the following directory

.. code-block:: bash

  cd moveit2_tutorials/doc/how_to_guides/isaac_panda

5. Build the Docker image

.. code-block:: bash

  docker compose build

6. Start a conatiner based on the new image

.. code-block:: bash

  docker compose up

7. Open up a second terminal and connect a bash instance to the docker container so we can run the Rviz MoveIt portion of the tutorial

.. code-block:: bash

  docker exec -it isaac_panda-base-1 bash

Once Steps 1-7 are complete you are ready to simulate the Panda robot with a fake system or connect to a simulated robot in Isaac.

To start the simulated robot in Isaac:

8. On the host computer cd into the tutorials directory

.. code-block:: bash

  cd moveit2_tutorials/doc/how_to_guides/isaac_panda/launch

9. Then run the following command to load the Panda Robot pre-configured to work with this tutorial

.. note:: This step assumes Isaac is installed on the host in the "$HOME/.local/share/ov/pkg/isaac_sim-2022.2.0" directory

.. code-block:: bash

  ./python.sh isaac_moveit.py

Running the MoveIt Interactive Marker Demo
------------------------------------------

After the Isaac Simulator has started and the Panda Robot appears in the Viewport move over to the termainal we have loaded inside the docker container and verify that we can receive ROS messages from Isaac

.. code-block:: bash

  ros2 topic list

Should return the following topics. If you do not see the topics from Isaac you can not continue and you will need to diagnose your DDS configuration.

.. code-block:: bash

  /clock
  /isaac_joint_commands
  /isaac_joint_states
  /parameter_events
  /rosout

Lastly start MoveIt with the ``isaac_ros2_control`` hardware interface

.. code-block:: bash

  ros2 launch moveit_resources_panda_moveit_config demo.launch.py ros2_control_hardware_type:=isaac


Show running the arm with both ros2_controllers

[INFO] [resource_manager]: Loading hardware 'PandaFakeSystem'
[INFO] [resource_manager]: Successful initialization of hardware 'PandaFakeSystem'

Debug section for DDS
