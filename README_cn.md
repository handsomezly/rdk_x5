# RDK X5 智能机器人视觉音频系统

简体中文 | [English](./README.md)

## 📋 项目概述

本项目是基于**RDK X5**开发板的综合智能机器人系统，集成了**计算机视觉**、**音频处理**和**机器人控制**功能。系统基于**ROS2 Humble**构建，利用地平线的**TogetheROS.Bot**生态系统，为智能机器人应用提供完整的解决方案。

### 🎯 核心特性

- **🎤 智能语音处理**：唤醒词检测、语音指令识别、声源定位（DOA）和语音识别
- **👁️ 计算机视觉**：跌倒检测、手势识别、物体检测和实时图像处理
- **🤖 机器人控制**：机械臂控制、移动机器人导航和多传感器融合
- **🌐 Web界面**：通过WebSocket实现实时视频流和算法可视化
- **🔊 音频合成**：文本转语音（TTS）和音频播放功能
- **📡 通信接口**：串口通信、物联网集成和传感器数据处理

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    RDK X5 智能机器人系统                         │
├─────────────────────────────────────────────────────────────────┤
│  视觉模块 (视觉/)              │  音频模块 (音频/)                │
│  ├── 跌倒检测                 │  ├── 语音识别                   │
│  ├── 手势识别                 │  ├── 声源定位                   │
│  ├── 物体检测                 │  ├── 音频跟踪                   │
│  ├── 机械臂控制               │  ├── TTS语音合成                │
│  ├── Web显示界面              │  ├── 聊天机器人集成             │
│  └── 串口通信                 │  └── 激光雷达集成               │
├─────────────────────────────────────────────────────────────────┤
│                        ROS2 Humble + TogetheROS.Bot             │
├─────────────────────────────────────────────────────────────────┤
│                           RDK X5 硬件平台                       │
└─────────────────────────────────────────────────────────────────┘
```

## 🛠️ 硬件需求

### 核心组件

| 组件 | 规格 | 购买链接 |
|------|------|----------|
| **RDK X5** | 地平线RDK X5开发板 | [官方商店](https://developer.horizon.cc/rdkx5) |
| **麦克风阵列** | 4麦克风或2麦克风环形阵列 | [微雪音频HAT](https://www.waveshare.net/shop/Audio-Driver-HAT.htm) |
| **摄像头** | MIPI CSI摄像头 | 兼容RDK X5 |
| **机械臂** | 6自由度机械臂 | 兼容型号 |
| **激光雷达** | 奥比中光MS200 | [购买链接](https://detail.tmall.com/item.htm?id=706184556245) |

### 兼容机器人平台

| 机器人名称 | 制造商 | 链接 |
|------------|--------|------|
| OriginBot智能机器人 | 古月居 | [官方网站](https://www.originbot.org/) |
| X3派机器人 | 轮趣科技 | [淘宝](https://item.taobao.com/item.htm?id=676436236906) |
| 履带智能车 | 微雪电子 | [天猫](https://detail.tmall.com/item.htm?id=696078152772) |
| RDK X3机器人 | 亚博智能 | [天猫](https://detail.tmall.com/item.htm?id=726857243156) |

## 🚀 快速开始

### 1. 系统要求

- **操作系统**：Ubuntu 20.04/22.04 LTS
- **ROS版本**：ROS2 Humble
- **硬件**：RDK X5 + 麦克风阵列 + 摄像头

### 2. 安装步骤

```bash
# 克隆仓库
git clone https://github.com/your-username/RDK-X5.git
cd RDK-X5

# 安装依赖
sudo apt update
sudo apt install -y tros-humble-*

# 构建工作空间
colcon build
source install/setup.bash
```

### 3. 启动选项

#### 方案1：基础系统（推荐）
```bash
# 基础图像处理 + 串口通信
ros2 launch websocket vision_basic.launch.py
```

#### 方案2：视觉核心系统
```bash
# 跌倒检测 + WebSocket显示 + 图像处理
ros2 launch websocket vision_simple.launch.py
```

#### 方案3：完整系统
```bash
# 启用所有功能
ros2 launch websocket vision_integrated.launch.py
```

### 4. 系统验证

```bash
# 检查运行节点
ros2 node list

# 检查话题
ros2 topic list

# 测试机械臂
ros2 topic pub /cmd_ns std_msgs/msg/String "data: 'test_command'" --once
```

## 📦 模块详情

### 🎤 音频模块 (音频/)

#### 核心功能
- **语音识别**：唤醒词检测和指令识别
- **声源定位**：DOA角度计算（0-360°）
- **音频跟踪**：基于声源方向的机器人移动
- **TTS合成**：文本转语音转换
- **聊天机器人集成**：AI驱动的语音对话

#### 关键包
- `hobot_audio/`：核心音频处理引擎
- `audio_control/`：语音控制机器人移动
- `audio_tracking/`：声源跟踪和跟随
- `pygame_node/`：音频播放和音乐控制
- `zhipu/`：AI聊天机器人集成

### 👁️ 视觉模块 (视觉/)

#### 核心功能
- **跌倒检测**：实时人体跌倒检测
- **手势识别**：手势识别和控制
- **物体检测**：多类别物体检测
- **机械臂控制**：基于视觉的精确操作
- **Web界面**：实时视频流和可视化

#### 关键包
- `hobot_falldown_detection/`：跌倒检测算法
- `hobot_arm/`：机械臂控制系统
- `websocket/`：Web界面和实时流媒体
- `image_subscriber/`：图像处理和分析
- `uart/`：硬件控制的串口通信

## 🔧 配置说明

### 音频配置
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

### 视觉配置
```json
// hobot_arm/thresholds.json
{
  "targetX": 0.15,
  "targetY": 0.0,
  "targetZ": -0.08
}
```

## 🌐 Web界面

访问实时可视化界面：
```
http://[RDK_IP]:8000
```

功能特性：
- 实时视频流
- 算法结果叠加
- 检测框可视化
- 系统状态监控

## 🔌 API参考

### ROS2话题

#### 发布话题
| 话题 | 消息类型 | 描述 |
|------|----------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 机器人移动指令 |
| `/tts_text` | `std_msgs/msg/String` | 语音合成文本 |
| `/audio_smart` | `audio_msg/msg/SmartAudioData` | 音频智能结果 |

#### 订阅话题
| 话题 | 消息类型 | 描述 |
|------|----------|------|
| `/image_raw` | `sensor_msgs/msg/Image` | 摄像头图像输入 |
| `/cmd_ns` | `std_msgs/msg/String` | 语音指令输入 |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | IMU传感器数据 |

## 🔍 故障排除

### 常见问题

1. **音频设备未找到**
   ```bash
   # 检查音频设备
   aplay -l
   # 修复权限
   sudo chmod 666 /dev/snd/*
   ```

2. **摄像头访问失败**
   ```bash
   # 检查摄像头设备
   ls -la /dev/video*
   # 修复权限
   sudo chmod 666 /dev/video*
   ```

3. **串口通信错误**
   ```bash
   # 检查串口端口
   ls -la /dev/ttyUSB* /dev/ttyS*
   # 修复权限
   sudo chmod 666 /dev/ttyUSB0
   ```

### 性能优化

- **CPU使用率**：使用`htop`监控
- **内存使用**：使用`free -h`检查可用内存
- **GPU利用率**：使用系统工具检查BPU使用情况

## 📚 文档资源

- [快速启动指南](./视觉/quick_start.md)
- [依赖修复指南](./视觉/fix_dependencies.md)
- [API配置文件](./音频/config/api_config.yaml)
- [硬件设置指南](./docs/hardware_setup.md)

## 🎯 使用场景

### 智能家居
- **语音控制**：通过语音指令控制家电设备
- **安全监控**：跌倒检测和异常行为识别
- **智能交互**：AI聊天机器人和语音助手

### 教育机器人
- **STEM教育**：机器人编程和AI算法学习
- **互动教学**：语音交互和视觉反馈
- **实验平台**：多传感器融合和算法验证

### 工业应用
- **自动化控制**：机械臂精确操作
- **质量检测**：视觉检测和缺陷识别
- **人机协作**：安全监控和智能交互

## 🤝 贡献指南

1. Fork本仓库
2. 创建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 打开Pull Request

## 📄 许可证

本项目采用Apache License 2.0许可证 - 查看[LICENSE](LICENSE)文件了解详情。

## 🙏 致谢

- **地平线机器人**提供的RDK X5平台和TogetheROS.Bot生态系统
- **ROS2社区**提供的优秀机器人框架
- **开源贡献者**使这个项目成为可能

## 📞 支持

- **GitHub Issues**：[报告问题和功能请求](https://github.com/your-username/RDK-X5/issues)
- **项目文档**：[项目Wiki](https://github.com/your-username/RDK-X5/wiki)
- **社区讨论**：[讨论论坛](https://github.com/your-username/RDK-X5/discussions)

## 🔮 发展路线图

### 短期目标（3个月）
- [ ] 完善Web界面功能
- [ ] 增加更多语音指令
- [ ] 优化跌倒检测算法
- [ ] 添加更多机器人平台支持

### 中期目标（6个月）
- [ ] 集成更多AI模型
- [ ] 支持多机器人协作
- [ ] 增加云端服务集成
- [ ] 完善文档和教程

### 长期目标（1年）
- [ ] 构建完整的开发者生态
- [ ] 支持更多硬件平台
- [ ] 开发商业化应用
- [ ] 建立认证体系

---

**🌟 如果您觉得这个项目有用，请为我们点个星！**

由RDK X5社区用❤️构建 