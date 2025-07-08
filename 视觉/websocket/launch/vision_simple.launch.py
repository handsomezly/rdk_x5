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
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    
    print("==========================================")
    print("    启动视觉核心系统")
    print("    Vision Core System Starting...")
    print("==========================================")
    
    return LaunchDescription([
        # 1. 跌倒检测功能（核心视觉功能）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_falldown_detection'),
                    'launch/hobot_falldown_detection.launch.py'))
        ),
        
        # 2. WebSocket显示服务
        Node(
            package='websocket',
            executable='websocket',
            output='screen',
            parameters=[
                {"image_topic": "/image_jpeg"},
                {"image_type": "mjpeg"},
                {"only_show_image": False},
                {"output_fps": 0},
                {"smart_topic": "/hobot_falldown_detection"}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        
        # 3. 图像订阅者（图像理解功能）
        Node(
            package='image_subscriber',
            executable='image_subscriber',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ]) 