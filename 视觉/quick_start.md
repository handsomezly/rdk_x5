# 视觉系统快速启动指南 🚀

## 🛠️ 准备工作

### 1. 复制机械臂配置文件
```bash
# 复制配置文件到系统目录
sudo cp /home/root/cc_ws/src/视觉/hobot_arm/thresholds.json /root/

# 验证文件是否存在
ls -la /root/thresholds.json
```

### 2. 检查并配置串口设备
```bash
# 查看所有串口设备
ls -la /dev/tty*

# 查看USB串口设备
ls -la /dev/ttyUSB*

# 查看系统串口设备  
ls -la /dev/ttyS*

# 查看设备连接信息
dmesg | grep tty

# 如果找到串口设备，修复权限（替换为实际设备名）
# sudo chmod 666 /dev/ttyUSB0
# sudo chmod 666 /dev/ttyS3
```

### 3. 验证音频设备
```bash
# 确认音频设备
aplay -l

# 你应该看到：
# card 0: duplexaudio [duplex-audio], device 0
```

## 🚀 启动系统

### 方案1：启动基础系统（推荐，无AI功能）
```bash
# 进入工作空间
cd /home/root/cc_ws

# 编译
colcon build

# 设置环境
source install/setup.bash

# 启动基础系统（图像处理+串口通信）
ros2 launch websocket vision_basic.launch.py
```

### 方案2：启动最小化系统（包含机械臂）
```bash
# 进入工作空间
cd /home/root/cc_ws

# 编译
colcon build

# 设置环境
source install/setup.bash

# 启动最小化系统（只包含机械臂+串口）
ros2 launch websocket vision_minimal.launch.py
```

### 方案3：启动视觉核心系统（需要AI模型）
```bash
# ⚠️ 需要先安装AI模型文件
sudo apt install tros-hobot-dnn-models

# 进入工作空间
cd /home/root/cc_ws

# 编译
colcon build

# 设置环境
source install/setup.bash

# 启动视觉核心系统（跌倒检测+WebSocket显示+图像处理）
ros2 launch websocket vision_simple.launch.py
```

### 方案4：启动完整系统（需要所有依赖）
```bash
# ⚠️ 需要先修复所有依赖问题
# 参考 fix_dependencies.md 文件

# 进入工作空间
cd /home/root/cc_ws

# 编译
colcon build

# 设置环境
source install/setup.bash

# 启动完整系统
ros2 launch websocket vision_integrated.launch.py
```

## 🔍 验证系统运行

### 1. 检查节点状态
```bash
# 新开一个终端
ros2 node list

# 你应该看到：
# /arm_node
# /serial_node
```

### 2. 检查话题
```bash
# 查看活动话题
ros2 topic list

# 查看机械臂参数
ros2 param get /arm_node targetX
```

### 3. 测试机械臂
```bash
# 发布测试消息到机械臂
ros2 topic pub /cmd_ns std_msgs/msg/String "data: 'test_command'" --once
```

## ⚠️ 常见问题解决

### 1. 如果机械臂节点启动失败
```bash
# 检查配置文件是否存在
ls -la /root/thresholds.json

# 如果不存在，重新复制
sudo cp /home/root/cc_ws/src/视觉/hobot_arm/thresholds.json /root/
```

### 2. 如果串口通信失败
```bash
# 检查串口设备
ls -la /dev/ttyUSB*

# 修复权限
sudo chmod 666 /dev/ttyUSB0
```

### 3. 如果TTS服务启动失败
```bash
# 检查音频设备
aplay -l

# 确认音频配置为 hw:0,0
```

## 📊 系统架构

```
机械臂控制节点 (hobot_arm)
├── 订阅话题
│   ├── /cmd_ns (语音指令)
│   ├── /cmd_mo (物联网指令)
│   ├── /cs_dis (超声波距离)
│   ├── /imu/data_raw (IMU数据)
│   ├── /hobot_falldown_detection (跌倒检测)
│   └── /hobot_hand_gesture_detection (手势识别)
├── 发布话题
│   └── /tts_text (TTS文本)
└── 串口通信
    ├── /dev/ttyS3 (机械臂控制)
    └── /dev/ttyUSB0 (通用串口)
```

## 🎯 成功标志

如果看到以下日志，说明系统启动成功：
```
[arm_node]: Parameter:
 targetX: 0.15
 targetY: 0
 targetZ: -0.08
[arm_node]: ArmNode Init Succeed!
```

## 📞 下一步

成功启动后，您可以：
1. 测试机械臂控制功能
2. 添加更多传感器节点
3. 集成语音控制系统
4. 连接Web显示界面 