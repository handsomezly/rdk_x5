# RDK X5 Intelligent Robot Vision-Audio System

[简体中文](./README_cn.md) | English

## 📋 Project Overview

This project is a comprehensive intelligent robot system based on the **RDK X5** development board, integrating **computer vision**, **audio processing**, and **robotic control** capabilities. The system is built on **ROS2 Humble** and leverages Horizon's **TogetheROS.Bot** ecosystem to provide a complete solution for intelligent robotics applications.

### 🎯 Key Features

- **🎤 Intelligent Voice Processing**: Wake-up word detection, voice command recognition, sound source localization (DOA), and ASR
- **👁️ Computer Vision**: Fall detection, gesture recognition, object detection, and real-time image processing
- **🤖 Robotic Control**: Robotic arm control, mobile robot navigation, and multi-sensor fusion
- **🌐 Web Interface**: Real-time video streaming and algorithm visualization through WebSocket
- **🔊 Audio Synthesis**: Text-to-speech (TTS) and audio playback capabilities
- **📡 Communication**: Serial communication, IoT integration, and sensor data processing

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    RDK X5 Intelligent Robot System              │
├─────────────────────────────────────────────────────────────────┤
│  Vision Module (视觉)          │  Audio Module (音频)            │
│  ├── Fall Detection            │  ├── Voice Recognition          │
│  ├── Gesture Recognition       │  ├── Sound Source Localization  │
│  ├── Object Detection          │  ├── Audio Tracking             │
│  ├── Robotic Arm Control       │  ├── TTS Synthesis              │
│  ├── Web Display Interface     │  ├── ChatBot Integration        │
│  └── Serial Communication      │  └── LiDAR Integration          │
├─────────────────────────────────────────────────────────────────┤
│                        ROS2 Humble + TogetheROS.Bot             │
├─────────────────────────────────────────────────────────────────┤
│                           RDK X5 Hardware                       │
└─────────────────────────────────────────────────────────────────┘
```

## 🛠️ Hardware Requirements

### Core Components

| Component | Specification | Purchase Link |
|-----------|---------------|---------------|
| **RDK X5** | Horizon RDK X5 Development Board | [Official Store](https://developer.horizon.cc/rdkx5) |
| **Microphone Array** | 4-mic or 2-mic circular array | [Waveshare Audio HAT](https://www.waveshare.net/shop/Audio-Driver-HAT.htm) |
| **Camera** | MIPI CSI camera | Compatible with RDK X5 |
| **Robotic Arm** | 6-DOF robotic arm | Compatible models |
| **LiDAR** | Oradar MS200 | [Purchase Link](https://detail.tmall.com/item.htm?id=706184556245) |

### Compatible Robot Platforms

| Robot Name | Manufacturer | Link |
|------------|--------------|------|
| OriginBot Smart Robot | GUYUEJU | [Official Site](https://www.originbot.org/) |
| X3 Pi Robot | Wheel Fun Technology | [Taobao](https://item.taobao.com/item.htm?id=676436236906) |
| Tracked Smart Car | Waveshare | [TMall](https://detail.tmall.com/item.htm?id=696078152772) |
| RDK X3 Robot | Yabo Intelligence | [TMall](https://detail.tmall.com/item.htm?id=726857243156) |

## 🚀 Quick Start

### 1. System Requirements

- **OS**: Ubuntu 20.04/22.04 LTS
- **ROS**: ROS2 Humble
- **Hardware**: RDK X5 + Microphone Array + Camera

### 2. Installation

```bash
# Clone the repository
git clone https://github.com/your-username/RDK-X5.git
cd RDK-X5

# Install dependencies
sudo apt update
sudo apt install -y tros-humble-*

# Build the workspace
colcon build
source install/setup.bash
```

### 3. Launch Options

#### Option 1: Basic System (Recommended)
```bash
# Basic image processing + serial communication
ros2 launch websocket vision_basic.launch.py
```

#### Option 2: Vision Core System
```bash
# Fall detection + WebSocket display + image processing
ros2 launch websocket vision_simple.launch.py
```

#### Option 3: Complete System
```bash
# All features enabled
ros2 launch websocket vision_integrated.launch.py
```

### 4. Verification

```bash
# Check running nodes
ros2 node list

# Check topics
ros2 topic list

# Test robotic arm
ros2 topic pub /cmd_ns std_msgs/msg/String "data: 'test_command'" --once
```

## 📦 Module Details

### 🎤 Audio Module (音频/)

#### Core Features
- **Voice Recognition**: Wake-up word detection and command recognition
- **Sound Source Localization**: DOA angle calculation (0-360°)
- **Audio Tracking**: Robot movement based on sound source direction
- **TTS Synthesis**: Text-to-speech conversion
- **ChatBot Integration**: AI-powered voice conversation

#### Key Packages
- `hobot_audio/`: Core audio processing engine
- `audio_control/`: Voice-controlled robot movement
- `audio_tracking/`: Sound source tracking and following
- `pygame_node/`: Audio playback and music control
- `zhipu/`: AI chatbot integration

### 👁️ Vision Module (视觉/)

#### Core Features
- **Fall Detection**: Real-time human fall detection
- **Gesture Recognition**: Hand gesture recognition and control
- **Object Detection**: Multi-class object detection
- **Robotic Arm Control**: Precision manipulation based on vision
- **Web Interface**: Real-time video streaming and visualization

#### Key Packages
- `hobot_falldown_detection/`: Fall detection algorithm
- `hobot_arm/`: Robotic arm control system
- `websocket/`: Web interface and real-time streaming
- `image_subscriber/`: Image processing and analysis
- `uart/`: Serial communication for hardware control

## 🔧 Configuration

### Audio Configuration
```yaml
# config/api_config.yaml
audio_device: "hw:0,0"
wake_word: "地平线你好"
command_words:
  - "向前走"
  - "向后退"
  - "向左转"
  - "向右转"
  - "停止运动"
```

### Vision Configuration
```json
// hobot_arm/thresholds.json
{
  "targetX": 0.15,
  "targetY": 0.0,
  "targetZ": -0.08
}
```

## 🌐 Web Interface

Access the real-time visualization interface at:
```
http://[RDK_IP]:8000
```

Features:
- Live video streaming
- Algorithm result overlay
- Detection box visualization
- System status monitoring

## 🔌 API Reference

### ROS2 Topics

#### Published Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot movement commands |
| `/tts_text` | `std_msgs/msg/String` | Text for speech synthesis |
| `/audio_smart` | `audio_msg/msg/SmartAudioData` | Audio intelligence results |

#### Subscribed Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/image_raw` | `sensor_msgs/msg/Image` | Camera image input |
| `/cmd_ns` | `std_msgs/msg/String` | Voice command input |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | IMU sensor data |

## 🔍 Troubleshooting

### Common Issues

1. **Audio Device Not Found**
   ```bash
   # Check audio devices
   aplay -l
   # Fix permissions
   sudo chmod 666 /dev/snd/*
   ```

2. **Camera Access Failed**
   ```bash
   # Check camera devices
   ls -la /dev/video*
   # Fix permissions
   sudo chmod 666 /dev/video*
   ```

3. **Serial Communication Error**
   ```bash
   # Check serial ports
   ls -la /dev/ttyUSB* /dev/ttyS*
   # Fix permissions
   sudo chmod 666 /dev/ttyUSB0
   ```

### Performance Optimization

- **CPU Usage**: Monitor with `htop`
- **Memory Usage**: Use `free -h` to check available memory
- **GPU Utilization**: Check BPU usage with system tools

## 📚 Documentation

- [Quick Start Guide](./视觉/quick_start.md)
- [Dependency Fix Guide](./视觉/fix_dependencies.md)
- [API Configuration](./音频/config/api_config.yaml)
- [Hardware Setup Guide](./docs/hardware_setup.md)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Horizon Robotics** for the RDK X5 platform and TogetheROS.Bot ecosystem
- **ROS2 Community** for the excellent robotics framework
- **Open Source Contributors** who made this project possible

## 📞 Support

- **GitHub Issues**: [Report bugs and feature requests](https://github.com/your-username/RDK-X5/issues)
- **Documentation**: [Project Wiki](https://github.com/your-username/RDK-X5/wiki)
- **Community**: [Discussion Forum](https://github.com/your-username/RDK-X5/discussions)

---

**🌟 Star this repository if you find it useful!**

Built with ❤️ by the RDK X5 Community
