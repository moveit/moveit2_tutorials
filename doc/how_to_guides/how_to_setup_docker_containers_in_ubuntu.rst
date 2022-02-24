How to Setup Moveit 2 Docker Containers in Ubuntu
=================================================
This guide will provide a walkthrough on how to get docker container with Moveit 2 dependencies setup quickly.
It includes a script that will get you up and running in Moveit quickly!
This guide is intended for people who would like to have a separate environment for working with Moveit up and running quickly \
without having to do much configuring. In this guide, we will be setting up a ROS2 Galactic environment.

Learning Objectives
-------------------

- How to setup a docker environment using the provided script

Requirements
------------

- Ubuntu 20.04
- `Docker Installation for Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_

Steps
-----

1. Install Docker (a link is available in the Requirements section) and be sure to follow the `Linux Post Install <https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_ instructions. If you do not complete these additional steps you will need to preface all `docker` commands with `sudo`.

2. Open a terminal session and create an empty text file.

  .. code-block:: bash

    mkdir -p ~/Docker/scripts
    cd ~/Docker/scripts/

3. Fetch docker script and make it executable

  .. code-block:: bash

    wget https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/_scripts/start-docker.sh
    chmod +x ~/Docker/scripts/start-docker.sh

4. Run the script

  There are 3 parameters for the script.
     - name_of_the_container : this is the name you wish to give the created container
     - name_of_the_image : if you are creating a fresh docker container, provide the name of the docker image here
     - using_gpu : if `true`, the docker will be run using nvidia gpu drivers. By default, this value is true.

  To run the script and use nvidia gpu drivers

  .. code-block:: bash

    ~/Docker/scripts/start-docker.sh moveit2-galactic moveit/moveit2:galactic-source

  If either of the above command fails, you are likely not using nvidia drivers. You'll need to remove the container you just created `docker rm moveit2-galactic`

  To run the docker without nvidia drivers

  .. code-block:: bash

    ~/Docker/scripts/start-docker.sh moveit2-galactic moveit/moveit2:galactic-source false

  After running the script for the first time, you only would need to

  .. code-block:: bash

    ~/Docker/scripts/start-docker.sh moveit2-galactic

5. You should now be inside of your docker container, in the workspace directory. You should now be able to start working with Moveit!

Further Reading
---------------
- For more information about Docker best practices with respects to Moveit,
  refer to `this blog post <https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html>`_
  from Picknik's Vatan Aksoy Tezer and Brennard Pierce.

- `Here <https://hub.docker.com/r/moveit/moveit2/tags>`_ is a list of the available moveit2 docker images available.
