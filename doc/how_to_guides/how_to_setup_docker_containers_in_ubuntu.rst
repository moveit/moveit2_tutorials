How to Set Up MoveIt 2 Docker Containers in Ubuntu
===================================================
This guide will provide a walkthrough on how to get a Docker container with MoveIt 2 dependencies set up quickly.
It includes a script that will get you up and running in MoveIt quickly!
This guide is intended for people who would like to have a separate environment for working with MoveIt up and running quickly \
without having to do much configuring. In this guide, we will be setting up a ROS2 Humble environment.

Learning Objectives
-------------------

<<<<<<< HEAD
- How to setup a Docker environment using the provided script
=======
- How to use ``docker compose`` to run MoveIt 2 Docker containers and tutorials
>>>>>>> e2f3b9a (Build and host docker images of the tutorials (#533))

Requirements
------------

- Ubuntu 20.04 or 22.04
- `Docker Installation for Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_
- `Nvidia drivers for Docker <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit>`_

Steps
-----
1. Install Docker (a link is available in the Requirements section) and be sure to follow the `Linux Post Install <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_ instructions. If you do not complete these additional steps you will need to preface all ``docker`` commands with ``sudo``.

<<<<<<< HEAD
2. Open a terminal session, download the Docker script, and make it executable.

  .. code-block:: bash

    wget https://raw.githubusercontent.com/abake48/moveit2_tutorials/how-to-docker-ubuntu/_scripts/start-docker.sh -O ~/.local/bin/start-docker.sh
    chmod +x ~/.local/bin/start-docker.sh
=======
2. Open a terminal session and download docker-compose.yml

  .. code-block:: bash

    wget https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/.docker/docker-compose.yml
>>>>>>> e2f3b9a (Build and host docker images of the tutorials (#533))

3. Run the script.

  There are 3 parameters for the script:
      - ``name_of_the_container`` : this is the name you wish to give the created container. For this guide, we will be naming the container ``moveit2-humble``.
      - ``name_of_the_image`` : if you are creating a fresh Docker container, provide the name of the Docker image here. For this guide, we will be using the image ``moveit/moveit2:humble-source``. Further explanation of this parameter is provided in the ``Further Reading`` section.
      - ``using_gpu`` : if ``true``, the Docker will be run using Nvidia GPU drivers. By default, this value is true.

<<<<<<< HEAD
  To run the script and use Nvidia GPU drivers

  .. code-block:: bash

    start-docker.sh moveit2-humble moveit/moveit2:humble-source

  If the above command fails, it is likely that Nvidia drivers cannot be used or are installed correctly. In which case, you can still proceed without using Nvidia drivers!
  First, you'll need to remove the container you just created by running the following command:

  .. code-block:: bash

    docker rm moveit2-humble

  Then, to run the Docker container without the Nvidia drivers, run the following command:

  .. code-block:: bash

    start-docker.sh moveit2-humble moveit/moveit2:humble-source false

  Running the script for the first time creates, starts, and executes the container ``moveit2-humble``.
=======
    DOCKER_IMAGE=rolling-tutorial docker compose run --rm --name moveit2_container gpu

   You can replace ``rolling-tutorial`` with other tagged images, e.g. ``humble-tutorial``. Similarly, you can replace ``gpu`` with ``cpu`` if you do not wish to run using Nvidia GPU drivers and you can change the name of the container by replacing ``moveit2_container``. The ``--rm`` argument will remove the container when you stop (or exit) it, otherwise you can keep your modified container on disk and start it using ``docker start moveit2_container``
>>>>>>> e2f3b9a (Build and host docker images of the tutorials (#533))

4. You should now be inside of your Docker container, in the workspace directory, with the completed :doc:`Planning Around Objects </doc/tutorials/planning_around_objects/planning_around_objects>` and :doc:`Pick and Place with MoveIt Task Constructor </doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor>` tutorials. Go ahead and try one of the launch commands like ``ros2 launch moveit2_tutorials demo.launch.py``

<<<<<<< HEAD
  Whenever you wish to reenter your container, you can run the following command:

  .. code-block:: bash

    start-docker.sh moveit2-humble
=======
  If you wish to enter the container through another terminal, use:

   .. code-block:: bash

    docker exec -it moveit2_container /bin/bash
>>>>>>> e2f3b9a (Build and host docker images of the tutorials (#533))

Further Reading
---------------
- For more information about Docker best practices with respect to MoveIt,
  refer to `this blog post <https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html>`_
  from PickNik's Vatan Aksoy Tezer and Brennard Pierce.

- You can find a list of tagged tutorial images `here <https://github.com/ros-planning/moveit2_tutorials/pkgs/container/moveit2_tutorials>`__. There are tagged images for both ``rolling`` and ``humble`` which are built on top of the ``rolling-source`` and ``humble-source`` MoveIt 2 Docker images `here <https://hub.docker.com/r/moveit/moveit2/tags>`__.

- You can find more tagged images for MoveIt 2 Docker containers `here <https://hub.docker.com/r/moveit/moveit2/tags>`__.
  The tagged images coincide with ROS2 version releases. The ``release`` version of the container provides an environment in which MoveIt 2 is installed via the binaries.
  The ``source`` version of the Docker image will build MoveIt 2 from source.
<<<<<<< HEAD
  You can use any of the images in that link by substituting the second parameter in the script, ``name_of_the_image``, with moveit/moveit2:<tag_name>, where ``<tag_name>`` is from the above link.
  For example, this guide instructs you to use the image with the tag ``humble-source``.
=======
  You can use any of these images by substituting the DOCKER_IMAGE environment variable with a tag name from the above link (like ``rolling-source``), but you must use `this docker-compose.yml <https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/_scripts/docker-compose.yml>`_ instead (simply copy it to a different location and run your ``docker compose`` command there).
>>>>>>> e2f3b9a (Build and host docker images of the tutorials (#533))
