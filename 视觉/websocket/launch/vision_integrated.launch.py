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
import subprocess

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    
    print("==========================================")
    print("    启动视觉集成系统")
    print("    Vision Integrated System Starting...")
    print("==========================================")
    
    # 自动启动webserver服务
    name = 'nginx'
    nginx = "./sbin/" + name
    webserver = nginx + " -p ."
    launch_webserver = True
    
    try:
        # 查询进程列表，获取所有包含 webserver 字符串的进程
        processes = subprocess.check_output(['ps', 'ax'], universal_newlines=True)
        processes = [p.strip() for p in processes.split('\n') if webserver in p]
        
        # 如果有进程，说明目标程序已经在运行
        if len(processes) > 0:
            launch_webserver = False
    except:
        print("无法检查webserver状态，将尝试启动")
    
    if launch_webserver:
        print("正在启动webserver...")
        try:
            pwd_path = os.getcwd()
            webserver_path = os.path.join(get_package_prefix('websocket'),
                                          "lib/websocket/webservice")
            os.chdir(webserver_path)
            os.system(webserver + " &")  # 后台运行
            os.chdir(pwd_path)
            print("webserver启动成功")
        except Exception as e:
            print(f"webserver启动失败: {e}")
    else:
        print("webserver已经在运行")

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
        
        # 3. TTS语音合成服务（修复音频设备）
        Node(
            package='hobot_tts',
            executable='hobot_tts',
            output='screen',
            parameters=[
                {"topic_sub": "/tts_text"},
                {"playback_device": "hw:0,0"}  # 修复音频设备配置
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        
        # 4. 串口通信
        Node(
            package='uart',
            executable='serial_node',
            output='screen',
            parameters=[
                {"uart_dev": "/dev/ttyS1"},  # 使用实际存在的设备
                {"baudrate": 115200}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        
        # 5. 机械臂控制
        Node(
            package='hobot_arm',
            executable='hobot_arm',
            output='screen',
            parameters=[
                {"targetX": 0.15},
                {"targetY": 0.0},
                {"targetZ": -0.08}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ]) 