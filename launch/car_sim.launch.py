#!/usr/bin/env python3
#
# Copyright 2023. Prof. Jong Min Lee @ Dong-eui University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import json
from launch.substitutions import LaunchConfiguration


print(os.path.realpath(__file__))

ld = LaunchDescription()


def generate_launch_description():
    # configuration
    world = LaunchConfiguration('world')
    print('world =', world)
    world_file_name = 'car_track.world'
    world = os.path.join(get_package_share_directory('ros2_term_project'),
                         'worlds', world_file_name)
    print('world file name = %s' % world)
    # ld = LaunchDescription()
    car = 'PR001'
    declare_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    gazebo_run = ExecuteProcess(
        cmd=['gazebo', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    spawn_prius = Node(
        package='ros2_term_project',
        executable='spawn_prius',
        output='screen'
    )
    # 지정된 차량 정보를 전달하는 노드
    starter = Node(
        package='ros2_term_project',
        executable='starter',
        name='start_car',
        arguments=[car],
        output='screen'
    )

    controller = Node(
        package='ros2_term_project',
        executable='controller',
        name='controller',
        output='screen'
    )

    ld.add_action(declare_argument)
    ld.add_action(gazebo_run)
    ld.add_action(spawn_prius)
    ld.add_action(controller)
    # ld.add_action(starter)

    return ld
