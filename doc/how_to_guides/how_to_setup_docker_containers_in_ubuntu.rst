How to Set Up MoveIt 2 Docker Containers in Ubuntu
===================================================
This guide will provide a walkthrough on how to get a Docker container with MoveIt 2 dependencies set up quickly.
It includes a script that will get you up and running in MoveIt quickly!
This guide is intended for people who would like to have a separate environment for working with MoveIt up and running quickly \
without having to do much configuring. In this guide, we will be setting up a ROS2 Rolling environment.

Learning Objectives
-------------------

- How to setup a Docker environment using the provided script

Requirements
------------

- Ubuntu 20.04 or 22.04
- `Docker Installation for Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_
- `Nvidia drivers for Docker <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit>`_

Steps
-----
1. Install Docker (a link is available in the Requirements section) and be sure to follow the `Linux Post Install <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_ instructions. If you do not complete these additional steps you will need to preface all ``docker`` commands with ``sudo``.

2. Open a terminal session, download the Docker script, and make it executable.

  .. code-block:: bash

    wget https://raw.githubusercontent.com/abake48/moveit2_tutorials/how-to-docker-ubuntu/_scripts/start-docker.sh -O ~/.local/bin/start-docker.sh
    chmod +x ~/.local/bin/start-docker.sh

3. Run the script.

  There are 3 parameters for the script:
      - ``name_of_the_container`` : this is the name you wish to give the created container. For this guide, we will be naming the container ``moveit2-rolling``.
      - ``name_of_the_image`` : if you are creating a fresh Docker container, provide the name of the Docker image here. For this guide, we will be using the image ``moveit/moveit2:rolling-source``. Further explanation of this parameter is provided in the ``Further Reading`` section.
      - ``using_gpu`` : if ``true``, the Docker will be run using Nvidia GPU drivers. By default, this value is true.

  To run the script and use Nvidia GPU drivers

  .. code-block:: bash

    start-docker.sh moveit2-rolling moveit/moveit2:rolling-source

  If the above command fails, it is likely that Nvidia drivers cannot be used or are installed correctly. In which case, you can still proceed without using Nvidia drivers!
  First, you'll need to remove the container you just created by running the following command:

  .. code-block:: bash

    docker rm moveit2-rolling

  Then, to run the Docker container without the Nvidia drivers, run the following command:

  .. code-block:: bash

    start-docker.sh moveit2-rolling moveit/moveit2:rolling-source false

  Running the script for the first time creates, starts, and executes the container ``moveit2-rolling``.

4. You should now be inside of your Docker container, in the workspace directory. You should now be able to start working with MoveIt!

  Whenever you wish to reenter your container, you can run the following command:

  .. code-block:: bash

    start-docker.sh moveit2-rolling

Further Reading
---------------
- For more information about Docker best practices with respect to MoveIt,
  refer to `this blog post <https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html>`_
  from PickNik's Vatan Aksoy Tezer and Brennard Pierce.

- You can find a list of tagged images for the MoveIt 2 Docker container `here <https://hub.docker.com/r/moveit/moveit2/tags>`_.
  The tagged images coincide with ROS2 version releases. The ``release`` version of the container provides an environment in which MoveIt 2 is installed via the binaries.
  The ``source`` version of the Docker image will build MoveIt 2 from source.
  You can use any of the images in that link by substituting the second parameter in the script, ``name_of_the_image``, with moveit/moveit2:<tag_name>, where ``<tag_name>`` is from the above link.
  For example, this guide instructs you to use the image with the tag ``rolling-source``.
