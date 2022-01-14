How to Setup Moveit2 Docker Containers in Ubuntu
=================================================
This guide will provide a walkthrough on how to get docker container with Moveit2 dependencies setup quickly.
Provided in this guide is a script that will get you up and running in Moveit quickly!
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

1. Open a terminal session and create an empty text file.
::
  mkdir -p ~/Docker/scripts
  cd ~/Docker/scripts/
  gedit start-docker.sh

2. Copy and paste the following script inside of the opened text file
::
  #!/bin/bash

  display_usage() {
      printf "Usage:\n start_docker <name_of_the_container> <name_of_the_image (optional)> <using_gpu (true) (optional)>\n"
  }

  if [ -z "$1" ]
  then
      display_usage
      exit 1
  else
      CONTAINER_NAME=$1
      IMAGE_NAME=$2
      NO_GPU=$3
      if (docker ps --all | grep -q "$CONTAINER_NAME")
      then
          xhost +local:root &> /dev/null
          echo "Found a docker container with the given name, starting $1"
          printf "\n"
          # If Docker is already running, no need to start it
          if (docker ps | grep -q "$CONTAINER_NAME")
          then
              docker exec -it "$CONTAINER_NAME" /bin/bash && \
              xhost -local:root 1>/dev/null 2>&1
          else
              docker start "$CONTAINER_NAME" 1>/dev/null 2>&1
              docker exec -it "$CONTAINER_NAME" /bin/bash && \
              xhost -local:root 1>/dev/null 2>&1
          fi

      else
          if [ -z "$2" ]
          then
              printf "Can't find docker with the given name, need an image name to start the container from\n"
              display_usage
              exit 1
          else
              echo "Creating docker container $1 from image $2"
              printf "\n"
              if [ -z "$3" ]
              then
                  xhost +local:root &> /dev/null
                  docker run -it --privileged \
                      --net=host \
                      --gpus all \
                      --env=NVIDIA_VISIBLE_DEVICES=all \
                      --env=NVIDIA_DRIVER_CAPABILITIES=all \
                      --env=DISPLAY \
                      --env=QT_X11_NO_MITSHM=1 \
                      -v /tmp/.X11-unix:/tmp/.X11-unix \
                      --name "$CONTAINER_NAME" \
                      "$IMAGE_NAME" \
                      /bin/bash
                  xhost -local:root 1>/dev/null 2>&1
              else
                  # Start without GPU
                  echo "Opening up the docker container without GPU support"
                  printf "\n"
                  xhost +local:root &> /dev/null
                  docker run -it --privileged \
                      --net=host \
                      --env=DISPLAY \
                      --env=QT_X11_NO_MITSHM=1 \
                      -v "/tmp/.X11-unix:/tmp/.X11-unix" \
                      --name "$CONTAINER_NAME" \
                      "$IMAGE_NAME" \
                      /bin/bash
                  xhost -local:root 1>/dev/null 2>&1
              fi
          fi
      fi
  fi

 There are 3 parameters for the script.
  - name_of_the_container : this is the name you wish to give the created container
  - name_of_the_image : if you are creating a fresh docker container, provide the name of the docker image here
  - using_gpu : if `true`, the docker will be run using nvidia gpu drivers. By default, this value is true.

3. Running the script

To run the script and use nvidia gpu drivers
::
    ~/Docker/scripts/start-docker.sh moveit2-galactic moveit/moveit2:galactic-source

To run the docker without nvidia drivers
::
    ~/Docker/scripts/start-docker.sh moveit2-galactic moveit/moveit2:galactic-source false

4. You should now be inside of your docker container, in the workspace directory. You should now be able to start working with Moveit!

Further Reading
---------------
- For more information about Docker best practices with respects to Moveit,
  refer to `this blog post <https://picknik.ai/ros/robotics/docker/2021/07/20/Vatan-Aksoy-Tezer-Docker.html>`_
  from Picknik's Vatan Aksoy Tezer and Brennard Pierce.

- `Here <https://hub.docker.com/r/moveit/moveit2/tags>`_ is a list of the available moveit2 docker images available.