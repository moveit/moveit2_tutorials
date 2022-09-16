How to Set Up MoveIt 2 Docker Containers in Ubuntu
===================================================
This guide will provide a walkthrough on how to get a Docker container with MoveIt 2 dependencies set up quickly.
It includes a docker-compose config file that will get you up and running in MoveIt quickly!
This guide is intended for people who would like to have a separate environment for working with MoveIt up and running quickly \
without having to do much configuring. In this guide, we will be setting up a ROS2 Rolling environment.

Learning Objectives
-------------------

- How to setup a Docker environment using the provided docker compose config

Requirements
------------

- Ubuntu 20.04 or 22.04
- `Docker Installation for Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_
- `Nvidia drivers for Docker <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit>`_
- `docker compose (if not already installed via Docker Desktop) <https://docs.docker.com/compose/install/>`_

Steps
-----
1. Install Docker and docker compose (links are available in the Requirements section) and be sure to follow the `Linux Post Install <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_ instructions. If you do not complete these additional steps you will need to preface all ``docker`` commands with ``sudo``.

2. Open a terminal session, download the config, and make it executable.

  .. code-block:: bash

    wget https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/_scripts/docker-compose.yml

3. Launch the container (you may need to hyphenate ``docker-compose`` if using compose V1)

   .. code-block:: bash

    DOCKER_IMAGE=rolling-source docker compose run gpu

   You can replace ``rolling-source`` with other tagged images, e.g. ``humble-source``. Similarly, you can replace ``gpu`` with ``cpu`` if you do not wish to run using Nvidia GPU drivers.

4. You should now be inside of your Docker container, in the workspace directory. You should now be able to start working with MoveIt!

  Whenever you wish to reenter your container, run the same command as in #3.

Further Reading
---------------
- For more information about Docker best practices with respect to MoveIt,
  refer to `this blog post <https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html>`_
  from PickNik's Vatan Aksoy Tezer and Brennard Pierce.

- You can find a list of tagged images for the MoveIt 2 Docker container `here <https://hub.docker.com/r/moveit/moveit2/tags>`_.
  The tagged images coincide with ROS2 version releases. The ``release`` version of the container provides an environment in which MoveIt 2 is installed via the binaries.
  The ``source`` version of the Docker image will build MoveIt 2 from source.
  You can use any of the images in that link by substituting the DOCKER_IMAGE environment variable with ``moveit/moveit2:<tag_name>``, where ``<tag_name>`` is from the above link.
  For example, this guide instructs you to use the image with the tag ``rolling-source``.
