# Copyright (c) 2022，Horizon Robotics.
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

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 启动智能语音识别pkg
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_audio'),
                    'launch/hobot_audio.launch.py')),
            launch_arguments={
                'audio_pub_topic_name': '/audio_smart'

            }.items()
        ),
        DeclareLaunchArgument(
            'car_front_audio_angle',
            default_value='180',
            description='Voice angle in front of the car'),
        # 启动语音控制pkg
        Node(
            package='audio_tracking',
            executable='audio_tracking',
            output='screen',
            parameters=[
                {"ai_msg_sub_topic_name": "/audio_smart"},
                {"twist_pub_topic_name": "/cmd_vel"},
                {"front_audio_angle": LaunchConfiguration('car_front_audio_angle')}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
