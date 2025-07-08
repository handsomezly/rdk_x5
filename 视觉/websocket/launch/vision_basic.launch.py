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
from ament_index_python import get_package_share_directory


def generate_launch_description():
    
    print("==========================================")
    print("    启动基础系统（无AI功能）")
    print("    Basic System Starting (No AI)...")
    print("==========================================")
    
    return LaunchDescription([
        # 1. 图像订阅者（基础图像处理）
        Node(
            package='image_subscriber',
            executable='image_subscriber',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        
        # 2. 串口通信节点
        Node(
            package='uart',
            executable='serial_node',
            output='screen',
            parameters=[
                {"device": "/dev/ttyS1"},
                {"baud_rate": 115200}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ]) 