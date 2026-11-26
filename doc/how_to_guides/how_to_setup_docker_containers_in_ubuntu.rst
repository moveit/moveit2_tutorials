How to Set Up MoveIt 2 Docker Containers in Ubuntu
===================================================
This guide will provide a walkthrough on how to get a Docker container with MoveIt 2 dependencies set up quickly.
It includes a script that will get you up and running in MoveIt quickly!
This guide is intended for people who would like to have a separate environment for working with MoveIt up and running quickly \
without having to do much configuring. In this guide, we will be setting up a ROS2 Humble environment.

Learning Objectives
-------------------

- How to use ``docker compose`` to run MoveIt 2 Docker containers and tutorials

Requirements
------------

- Ubuntu 20.04 or 22.04
- `Docker Installation for Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_
- `Nvidia drivers for Docker <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit>`_

Steps
-----
1. Install Docker and docker compose (links are available in the Requirements section) and be sure to follow the `Linux Post Install <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_ instructions. If you do not complete these additional steps you will need to preface all ``docker`` commands with ``sudo``.

2. Open a terminal session and download docker-compose.yml

  .. code-block:: bash

    wget https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/.docker/docker-compose.yml

3. Launch the container (you may need to hyphenate ``docker-compose`` if using compose V1)

   .. code-block:: bash

    DOCKER_IMAGE={DISTRO}-tutorial docker compose run --rm --name moveit2_container gpu

   You can replace ``{DISTRO}-tutorial`` with other tagged images, e.g. ``rolling-tutorial``. Similarly, you can replace ``gpu`` with ``cpu`` if you do not wish to run using Nvidia GPU drivers and you can change the name of the container by replacing ``moveit2_container``. The ``--rm`` argument will remove the container when you stop (or exit) it, otherwise you can keep your modified container on disk and start it using ``docker start moveit2_container``

4. You should now be inside of your Docker container, in the workspace directory, with the completed :doc:`Planning Around Objects </doc/tutorials/planning_around_objects/planning_around_objects>` and :doc:`Pick and Place with MoveIt Task Constructor </doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor>` tutorials. Go ahead and try one of the launch commands like ``ros2 launch moveit2_tutorials demo.launch.py``

  If you wish to enter the container through another terminal, use:

   .. code-block:: bash

    docker exec -it moveit2_container /bin/bash

Further Reading
---------------
- For more information about Docker best practices with respect to MoveIt,
  refer to `this blog post <https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html>`_
  from PickNik's Vatan Aksoy Tezer and Brennard Pierce.

- You can find a list of tagged tutorial images `here <https://github.com/ros-planning/moveit2_tutorials/pkgs/container/moveit2_tutorials>`__. There are tagged images for both ``rolling`` and ``{DISTRO}`` which are built on top of the ``rolling-source`` and ``{DISTRO}-source`` MoveIt 2 Docker images `here <https://hub.docker.com/r/moveit/moveit2/tags>`__.

- You can find more tagged images for MoveIt 2 Docker containers `here <https://hub.docker.com/r/moveit/moveit2/tags>`__.
  The tagged images coincide with ROS2 version releases. The ``release`` version of the container provides an environment in which MoveIt 2 is installed via the binaries.
  The ``source`` version of the Docker image will build MoveIt 2 from source.
  You can use any of these images by substituting the DOCKER_IMAGE environment variable with a tag name from the above link (like ``rolling-source``), but you must use `this docker-compose.yml <https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/_scripts/docker-compose.yml>`_ instead (simply copy it to a different location and run your ``docker compose`` command there).
