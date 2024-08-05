Perception Pipeline Tutorial
==================================

Introduction
------------
MoveIt allows for seamless integration of 3D sensors using `Octomap <http://octomap.github.io/>`_.
Once properly configured, you should see something like this in rviz:

.. raw:: html

        <video width="700px" controls="true" autoplay="true" loop="true">
            <source src="../../../_static/videos/perception_pipeline_demo.webm" type="video/webm">
            MoveIt Perception Pipeline Demo
        </video>

Getting Started
---------------
Even though you haven't completed :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` ever, you can still run this tutorial's demo. But, it is suggested to start with steps in :doc:`Getting Started </doc/tutorials/getting_started/getting_started>` for better understanding what's going on in this tutorial.

Get The Necessary 3D Pointcloud Data For Octomap Creation
---------------------------------------------------------
In this tutorial, you can use `previously recorded 3D pointcloud data <https://drive.google.com/file/d/1fPtDAtJKIiw2gpFOOwA2TrPZOfFU053W/view?usp=sharing>`_ or you can record your own bag file. For recording bag, firstly, it can be both run ``depth_camera_envrionment.launch.py`` and recorded bag using following commands.

In shell 1, run this command: ::

    ros2 launch moveit2_tutorials depth_camera_environment.launch.py

In shell 2, run this command: ::

    ros2 bag record /camera_1/points /camera_2/points /tf /tf_static

When the command in shell 1 is executed, it is opened a gazebo environment which contains two depth camera and a table in the middle of those cameras. This gazebo environment is used for getting 3d sensor data. Gazebo environment should be looked like following image.

.. image:: images/perception_pipeline_demo_gazebo_screen.png

.. image:: images/perception_pipeline_depth_camera_environment.png

For more explanation of ``depth_camera_environment.launch.py``, you can look at comments in :codedir:`depth_camera_environment.launch.py <examples/perception_pipeline/launch/depth_camera_environment.launch.py>` on GitHub.

.. tutorial-formatter:: ./launch/depth_camera_environment.launch.py

For the commands in Shell 2, we must save both camera topics and tf topics due to the fact that MoveIt perception pipeline needs to listen TF's in order to convert the coordinates of pointcloud points according to ``world`` frame. Moreover, the reason of publishing static tf from world to camera frames in ``depth_camera_environment.launch.py`` is that it's necessary to determine transformation between robot and poincloud and that MoveIt's sensor plugins uses this transformation later.

By the way, you can also use :codedir:`this rviz file <examples/perception_pipeline/rviz2/depth_camera_environment.rviz>` on Github to visualize poincloud topics in rviz.

In next step, it will be used this recorded bag file for octomap creation.


Configuration For 3D Sensors
----------------------------
Now whatever you get bag file, you will see ``/camera_1/points``, ``/camera_2/points``, ``/tf`` and ``/tf_static`` in both way when played bag file. It should be created following config file for MoveIt to process these pointcloud topics in planning pipeline. You can also go to :codedir:`here <examples/perception_pipeline/config/sensors_3d.yaml>` for seeing all ``sensors_3d.yaml`` config file on Github.

.. tutorial-formatter:: config/sensors_3d.yaml

sensors_3d.yaml: ::

    sensors:
      - camera_1_pointcloud
      - camera_2_pointcloud
    camera_1_pointcloud:
        sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
        point_cloud_topic: /camera_1/points
        max_range: 5.0
        point_subsample: 1
        padding_offset: 0.1
        padding_scale: 1.0
        max_update_rate: 1.0
        filtered_cloud_topic: /camera_1/filtered_points
    camera_2_pointcloud:
        sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
        point_cloud_topic: /camera_2/points
        max_range: 5.0
        point_subsample: 1
        padding_offset: 0.1
        padding_scale: 1.0
        max_update_rate: 1.0
        filtered_cloud_topic: /camera_2/filtered_points

Running Demo
------------
The last step is to run ``perception_pipeline_demo.launch.py`` and play the bag file we recorded previously. You can apply these substeps using following commands.

In Shell 3: ::

    ros2 launch moveit2_tutorials perception_pipeline_demo.launch.py

In Shell 4: ::

    ros2 bag play -r 5 <your_bag_file> --loop

:codedir:`perception_pipeline_demo.launch.py <examples/perception_pipeline/launch/perception_pipeline_demo.launch.py>` is similar to :codedir:`demo.launch.py </doc/tutorials/quickstart_in_rviz/launch/demo.launch.py>` inside :doc:`MoveIt Quickstart in RViz </doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial>` except a couple of details. For ``perception_pipeline_demo.launch.py``, following lines is added to ``moveit_config``.

You can find these additional lines in line 51, 52 and 53 inside ``perception_pipeline_demo.launch.py``: ::

    .sensors_3d(file_path = os.path.join(
                get_package_share_directory("moveit2_tutorials"),
                "config/sensors_3d.yaml"))


Finally, all demo codes can be found in :codedir:`perception_pipeline's directory <examples/perception_pipeline>` on Github.
