# 依赖问题修复指南 🔧

## 🚨 SSL库缺失问题

### 问题现象
```
./sbin/nginx: error while loading shared libraries: libssl.so.1.1: cannot open shared object file: No such file or directory
```

### 解决方案

#### 方案1：Ubuntu 20.04 (推荐)
```bash
# 更新包列表
sudo apt update

# 安装OpenSSL 1.1
sudo apt install libssl1.1

# 验证安装
ls -la /usr/lib/x86_64-linux-gnu/libssl.so.1.1
```

#### 方案2：Ubuntu 22.04+
```bash
# 下载并安装OpenSSL 1.1
wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_amd64.deb
sudo dpkg -i libssl1.1_1.1.1f-1ubuntu2_amd64.deb

# 或者创建软链接
sudo ln -s /usr/lib/x86_64-linux-gnu/libssl.so.3 /usr/lib/x86_64-linux-gnu/libssl.so.1.1
sudo ln -s /usr/lib/x86_64-linux-gnu/libcrypto.so.3 /usr/lib/x86_64-linux-gnu/libcrypto.so.1.1
```

#### 方案3：ARM64设备
```bash
# 对于ARM64设备（如RDK X3）
sudo apt install libssl1.1

# 如果不可用，尝试：
wget http://ports.ubuntu.com/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_arm64.deb
sudo dpkg -i libssl1.1_1.1.1f-1ubuntu2_arm64.deb
```

## 🤖 AI模型文件缺失问题

### 问题现象
```
Model file config/multitask_body_head_face_hand_kps_960x544.hbm is not exist
Model file config/handLMKs.hbm is not exist
```

### 解决方案

#### 方案1：安装官方模型包
```bash
# 更新包列表
sudo apt update

# 安装TogetheROS模型包
sudo apt install tros-hobot-dnn-models

# 验证模型文件
ls -la /opt/tros/humble/lib/*/config/*.hbm
```

#### 方案2：手动下载模型
```bash
# 创建模型目录
sudo mkdir -p /opt/tros/humble/lib/mono2d_body_detection/config/
sudo mkdir -p /opt/tros/humble/lib/hand_lmk_detection/config/
sudo mkdir -p /opt/tros/humble/lib/hand_gesture_detection/config/

# 下载模型文件（需要从官方渠道获取）
# 这些是示例路径，实际需要从Horizon官方获取
```

#### 方案3：禁用AI功能（临时方案）
如果暂时不需要AI功能，可以创建简化的launch文件：

```bash
# 创建无AI版本的launch文件
cp /home/root/cc_ws/src/视觉/websocket/launch/vision_simple.launch.py /home/root/cc_ws/src/视觉/websocket/launch/vision_basic.launch.py
```

## 📷 摄像头问题

### 问题现象
```
[ERROR] [mipi_cam]: [init]->cap capture init failture.
```

### 解决方案

#### 检查摄像头连接
```bash
# 检查摄像头设备
ls -la /dev/video*

# 检查MIPI摄像头
dmesg | grep -i camera
dmesg | grep -i mipi
```

#### 检查摄像头权限
```bash
# 修复摄像头权限
sudo chmod 666 /dev/video*
```

## 🐍 Python模块缺失

### 安装缺失的Python模块
```bash
# 安装pygame
pip3 install pygame

# 安装requests
pip3 install requests

# 安装其他可能需要的模块
pip3 install pyserial
pip3 install zhipuai
```

## 🔊 音频设备配置

### 检查音频设备
```bash
# 查看音频设备
aplay -l

# 应该看到：
# card 0: duplexaudio [duplex-audio], device 0: duplex-audio duplex-audio-0 []
```

### 修复音频配置
```bash
# 如果音频设备不是hw:0,0，需要修改配置文件
# 编辑音频配置文件，将设备改为实际的设备名
```

## 📋 完整修复流程

### 1. 修复SSL库
```bash
sudo apt update
sudo apt install libssl1.1
```

### 2. 安装AI模型
```bash
sudo apt install tros-hobot-dnn-models
```

### 3. 检查硬件
```bash
# 检查摄像头
ls -la /dev/video*

# 检查音频
aplay -l

# 检查串口
ls -la /dev/ttyS*
```

### 4. 重新编译
```bash
cd /home/root/cc_ws
colcon build
source install/setup.bash
```

### 5. 重新启动
```bash
ros2 launch websocket vision_simple.launch.py
```

## 🆘 如果问题仍然存在

1. **检查系统版本**：确认是否使用了正确的Ubuntu版本
2. **检查硬件连接**：确认摄像头、音频设备正确连接
3. **查看详细日志**：使用 `--log-level debug` 获取更详细的错误信息
4. **分步测试**：单独测试每个功能模块

## 📞 联系支持

如果以上方案都无法解决问题，请提供：
- 系统版本信息：`lsb_release -a`
- 硬件信息：`lscpu`, `lsusb`, `dmesg`
- 详细错误日志 