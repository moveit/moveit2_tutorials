How to Set Up MoveIt 2 Docker Containers in Ubuntu
===================================================
This guide will provide a walkthrough on how to get a Docker container with MoveIt 2 dependencies set up quickly.
It includes a docker-compose config file that will get you up and running in MoveIt quickly!
This guide is intended for people who would like to have a separate environment for working with MoveIt up and running quickly \
without having to do much configuring. In this guide, we will be setting up a ROS2 Rolling environment.

Learning Objectives
-------------------

- How to use ``docker compose`` to run MoveIt 2 Docker containers and tutorials

Requirements
------------

- Ubuntu 22.04 or 24.04
- `Docker Installation for Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_
- `NVIDIA drivers for Docker <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit>`_
- `docker compose (if not already installed via Docker Desktop) <https://docs.docker.com/compose/install/>`_

Steps
-----
1. Install Docker and docker compose (links are available in the Requirements section) and be sure to follow the `Linux Post Install instructions <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_.
If you do not complete these additional steps you will need to preface all ``docker`` commands with ``sudo``.

2. Open a terminal session and download ``docker-compose.yml``

  .. code-block:: bash

    wget https://raw.githubusercontent.com/moveit/moveit2_tutorials/main/.docker/docker-compose.yml

3. Launch the container (you may need to hyphenate ``docker-compose`` if using compose V1)

   .. code-block:: bash

    DOCKER_IMAGE=main-rolling-tutorial-source docker compose run --rm --name moveit2_container gpu

   You can replace ``DOCKER_IMAGE`` with other Docker image names, such as ``main-jazzy-tutorial-source`` or ``humble-humble-tutorial source``.
   This is based on the convention ``<MoveIt branch name>-<ROS distro name>-tutorial-source``.

   Similarly, you can replace the ``gpu`` service name with ``cpu`` if you do not wish to run using NVIDIA GPU drivers and you can change the name of the container by replacing ``moveit2_container``.
   The ``--rm`` argument will remove the container when you stop (or exit) it, otherwise you can keep your modified container on disk and start it using ``docker start moveit2_container``.

4. You should now be inside of your Docker container, in the workspace directory, with the completed :doc:`Planning Around Objects </doc/tutorials/planning_around_objects/planning_around_objects>` and :doc:`Pick and Place with MoveIt Task Constructor </doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor>` tutorials.
Go ahead and try one of the launch commands like ``ros2 launch moveit2_tutorials demo.launch.py``.

  If you wish to enter the container through another terminal, use:

   .. code-block:: bash

    docker exec -it moveit2_container /bin/bash

Further Reading
---------------
- For more information about Docker best practices with respect to MoveIt,
  refer to `this blog post <https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html>`_
  from Vatan Aksoy Tezer and Brennand Pierce.

- There are tagged images for ``rolling``, ``jazzy``, and ``humble`` which are built on top of MoveIt 2 Docker images `here <https://hub.docker.com/r/moveit/moveit2/tags>`__.

- You can find more tagged images for MoveIt 2 Docker containers `here <https://hub.docker.com/r/moveit/moveit2/tags>`__.
  The tagged images coincide with ROS 2 version releases.
  The ``release`` version of the container provides an environment in which MoveIt 2 is installed via the binaries.
  The ``source`` version of the Docker image will build MoveIt 2 from source.
  You can use any of these images by substituting the ``DOCKER_IMAGE`` environment variable with a tag name from the above link (like ``main-rolling-tutorial-source``), but you must use `this docker-compose.yml <https://raw.githubusercontent.com/moveit/moveit2_tutorials/main/_scripts/docker-compose.yml>`_ instead (simply copy it to a different location and run your ``docker compose`` command there).
