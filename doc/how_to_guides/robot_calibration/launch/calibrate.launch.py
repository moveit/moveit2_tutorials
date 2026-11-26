#!/usr/bin/env python3

# Copyright (c) 2020-2022, Michael Ferguson
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # Load the capture config
    capture_config = os.path.join(
        get_package_share_directory('ur5_calibration'),
        'config',
        'capture.yaml'
    )

    # Load the calibration config
    calibration_config = os.path.join(
        get_package_share_directory('ur5_calibration'),
        'config',
        'calibrate.yaml'
    )

    # Load the calibration poses YAML
    calibration_poses = os.path.join(
        get_package_share_directory('ur5_calibration'),
        'config',
        'calibration_poses.yaml'
    )

    # Make a directory for bagfiles to be located
    # try:
    #     os.mkdir("/tmp/ubr1_calibration")
    # except FileExistsError:
    #     pass

    return LaunchDescription([
        # Calibration
        Node(
            name='robot_calibration',
            package='robot_calibration',
            executable='calibrate',
            arguments=[calibration_poses],
            parameters=[capture_config,
                        calibration_config],
            output='screen',
        ),
        # Record bagfile for debugging
        # ExecuteProcess(
        #     cmd=["ros2", "bag", "record", "/calibration_data", "/robot_description"],
        #     cwd="/tmp/ur5_calibration"
        # )
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()