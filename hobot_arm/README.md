English| [简体中文](./README_cn.md)

# Function Introduction

The hobot_arm package is an application example based on the mono2d_trash_detection package for 2D garbage object detection and mechanical arm grabbing. Utilizing BPU for model inference to obtain perception results on the Horizon Sunrise X3 and using the HEBI mechanical arm as the lower machine, it demonstrates garbage grabbing.

# Compilation

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

## Compilation

Supports compilation on the X3 Ubuntu system and cross-compilation using Docker on a PC.

### Compilation on X3 for Ubuntu Board

1. Compilation Environment Confirmation
   - The X3 Ubuntu system is installed on the board.
   - The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`, where PATH is the installation path of TogetherROS.
   - ROS2 compilation tools colcon are installed, installation command: `pip install -U colcon-common-extensions`

2. Compilation

Compilation command: `colcon build --packages-select hobot_arm`

### Cross-compilation on X3 using Docker

1. Compilation Environment Confirmation
   - Compilation is done in Docker, and TogetherROS is already installed in Docker. For Docker installation, cross-compilation instructions, TogetherROS compilation, and deployment instructions, please refer to the README.md in the robot_dev_config repo of the robot development platform.
   
2. Compilation

   - Compilation command:

   ```
   export TARGET_ARCH=aarch64
   export TARGET_TRIPLE=aarch64-linux-gnu
   export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

   colcon build --packages-select hobot_arm \
      --merge-install \
      --cmake-force-configure \
      --cmake-args \
      --no-warn-unused-cli \
      -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
   ```### Compiling X86 Version on X86 Ubuntu System

1. Confirming Compilation Environment

   - X86 Ubuntu Version: Ubuntu 20.04

2. Compilation

   - Compilation Command:

   ```
   colcon build --packages-select hobot_arm  \
      --merge-install \
      --cmake-args \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## Precautions

# Usage Introduction

## Dependencies

- mono2d_trash_detection package: Publishes garbage detection perception msg

## Parameters

None

## Execution

After successful compilation, copy the generated install path to the Horizon X3 development board (if compiling on X3, ignore the copy step), and run the following command:

### **X3 Ubuntu Startup**

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# The config is an example model, copy according to the actual installation path
# If compiling on the board end (without the --merge-install compilation option), the copy command is cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., where PKG_NAME is the specific package name.
cp -r install/lib/mono2d_trash_detection/config/ .

# Start the dnn_node_example package
# Detect input from the mipi camera with rendering results visualized on the web page
export CAM_TYPE=mipi
ros2 launch dnn_node_example dnn_node_example.launch.py config_file:=config/ppyoloworkconfig.json msg_pub_topic_name:=ai_msg_mono2d_trash_detection image_width:=1920 image_height:=1080

# Start the dnn_node_example package
# USB camera input detection, rendering results visualized on Web page
export CAM_TYPE=usb
ros2 launch dnn_node_example dnn_node_example.launch.py config_file:=config/ppyoloworkconfig.json msg_pub_topic_name:=ai_msg_mono2d_trash_detection image_width:=1920 image_height:=1080

ros2 run hobot_arm hobot_arm
```

### **X3 Linux**
If you need to render and display images published by the sensor and the corresponding AI results on a PC browser, make sure that the X3 Pi has started the webserver service for web display (the service only needs to be started once after the device is started, and only needs to be restarted if the device is restarted). Check if there is an nginx process by executing the "ps -aux" command on the X3 Pi. If there is, it means the service has been started; if not, start the service using the following method:
```shell
cd /opt/tros/lib/websocket/webservice/
chmod +x ./sbin/nginx && ./sbin/nginx -p .
cd -

export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# Copy the model configuration based on the actual installation path in the config folder
cp -r ./install/lib/mono2d_trash_detection/config/ .

# Launch the image publishing package
./install/lib/mipi_cam/mipi_cam --ros-args -p out_format:=nv12 -p image_width:=416 -p image_height:=416 -p io_method:=shared_mem -p video_device:=GC4663 --log-level error &

# Launch the JPEG image encoding & publishing package
./install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg --ros-args --log-level error &

# Launch the web display package
./install/lib/websocket/websocket --ros-args -p image_topic:=/image_jpeg -p image_type:=mjpeg -p smart_topic:=/ai_msg_mono2d_trash_detection --log-level error &

# Launch the dnn_node_example node
./install/lib/dnn_node_example/example --ros-args -p feed_type:=1 -p is_shared_mem_sub:=1 -p dump_render_img:=0 -p msg_pub_topic_name:=/ai_msg_mono2d_trash_detection --log-level warn &

# Launch the hobot node
./install/lib/hobot_arm/hobot_arm --log-level warn
```

### **X86 Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# Copy the model configuration based on the actual installation path in the config folder
cp -r ./install/lib/mono2d_trash_detection/config/ .

# Launch the dnn_node_example node
ros2 run dnn_node_example example --ros-args -p feed_type:=0 -p image:=config/trashDet0028.jpg -p image_type:=0 -p dump_render_img:=1 -p config_file:=config/ppyoloworkconfig.json

```
# Result Analysis
## X3 Result Display

Output log:
```
[WARN] [1700107618.608870258] [arm_node]: Parameter:
 targetX: 0.15
 targetY: 0
 targetZ: -0.08
[WARN] [1700107618.609489260] [arm_node]: ArmNode Init Succeed!
[WARN] [1700107621.610287833] [arm_node]: Operate Succeed!
^C[INFO] [1700107623.664628627] [rclcpp]: signal_handler(signal_value=2)
root@ubuntu:~# ros2 run hobot_arm hobot_arm

        This is hobot arm package.

============================================
        the robotic arm device
============================================

[WARN] [1700107629.060302603] [arm_node]: Parameter:
 targetX: 0.15
 targetY: 0
 targetZ: -0.08
[WARN] [1700107629.060854068] [arm_node]: ArmNode Init Succeed!
[WARN] [1700107632.061743344] [arm_node]: Operate Succeed!

```


## Result Description
The example demonstrates real-time inference for garbage detection. The local display effect shows rendering images saved locally.

When setting "dump_render_img" to 1, the rendered images are saved in real-time in the current path.
