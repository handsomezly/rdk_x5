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


def generate_launch_description():
    print(111111111111111111)
    hand_lmk_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hand_lmk_detection'),
                'launch/hand_lmk_detection.launch.py')),
        launch_arguments={
            'smart_topic': '/hobot_hand_gesture_detection',
            'hand_lmk_pub_topic': '/hobot_hand_lmk_detection'
        }.items()
    )

    # 手势识别算法
    hand_gesture_det_node = Node(
        package='hand_gesture_detection',
        executable='hand_gesture_detection',
        output='screen',
        parameters=[
            {"ai_msg_pub_topic_name": "/hobot_hand_gesture_detection"},
            {"ai_msg_sub_topic_name": "/hobot_hand_lmk_detection"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 跌倒检测
    hobot_falldown_det_node = Node(
        package='hobot_falldown_detection',
        executable='hobot_falldown_detection',
        output='screen',
        parameters=[
            {"paramSensivity": 3},
            {"body_kps_topic_name": "hobot_mono2d_body_detection"},
            {"pub_smart_topic_name": "/hobot_falldown_detection"}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # mono2d_body_det_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('mono2d_body_detection'),
    #             'launch/mono2d_body_detection.launch.py')),
    #     launch_arguments={
    #         'smart_topic': '/hobot_falldown_detection',
    #         'mono2d_body_pub_topic': '/hobot_mono2d_body_detection'
    #     }.items()
    # )

    return LaunchDescription([
        # mono2d_body_det_node,
        hand_lmk_det_node,
        hand_gesture_det_node,
        hobot_falldown_det_node
    ])
