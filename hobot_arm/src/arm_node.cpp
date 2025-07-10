// Copyright (c) 2023，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/arm_node.h"
#include<iostream>

ArmNode::ArmNode(const std::string &node_name, const NodeOptions &options)   //初始化节点
    : rclcpp::Node(node_name, options) {

  this->declare_parameter<float>("targetX", targetX_);
  this->declare_parameter<float>("targetY", targetY_);
  this->declare_parameter<float>("targetZ", targetZ_);  //声明参数

  this->get_parameter<float>("targetX", targetX_);
  this->get_parameter<float>("targetY", targetY_);
  this->get_parameter<float>("targetZ", targetZ_);  //获取参数

  // 调用读取阈值的函数
  loadThresholds("/root/thresholds.json");  //读取阈值
  tts_text_publisher_ = this->create_publisher<std_msgs::msg::String>("tts_text", 10);  //创建发布者，发布tts_text话题 tts（文本转语音）

  {
  //在程序运行时，将 targetX_, targetY_, targetZ_ 这三个变量的值，以一个清晰、多行的格式，作为一条黄色的警告信息打印在屏幕上。
    std::stringstream ss;    //创建字符串流
    ss << "Parameter:"  
       << "\n targetX: " << targetX_  
       << "\n targetY: " << targetY_   
       << "\n targetZ: " << targetZ_;      
    RCLCPP_WARN(rclcpp::get_logger("arm_node"), "%s", ss.str().c_str());  
  }  

  RCLCPP_WARN(rclcpp::get_logger("arm_node"), "ArmNode Init Succeed!");

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw", 10, std::bind(&ArmNode::imu_callback, this, std::placeholders::_1));  //创建订阅器，订阅imu/data_raw话题

  // 初始化订阅器
  cs_dis_subscription_ = this->create_subscription<std_msgs::msg::Int32>(  
    "/cs_dis", 10, std::bind(&ArmNode::csDisCallback, this, std::placeholders::_1));  //创建订阅器，订阅cs_dis话题，超声波距离用于简单避障

  //订阅语音消息
  update_object_subscription_ =
    this->create_subscription<std_msgs::msg::String>(  //接收来自语音识别系统的指令，并执行相应的操作
        "cmd_ns", // 替换为更具体的 object 话题名称  
        10,
        std::bind(&ArmNode::UpdateObjectCallback, this, std::placeholders::_1));  //绑定回调函数

  //创建了三个不同的订阅器,这可以理解为给机器人安装了三个不同的“耳朵”，每个“耳朵”只听特定频道的信息，并根据听到的内容执行不同的任务。
  smart_subscription_ =  //通用AI感知订阅
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        ai_msg_sub_topic_name_,
        10,
        std::bind(&ArmNode::SmartTopicCallback,
                  this,
                  std::placeholders::_1));  

  body_detection_subscription_ =  //人体摔倒检测订阅
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        "/hobot_falldown_detection",  // Topic name
        10,  // Buffer size
        std::bind(&ArmNode::BodyDetectionCallback, this, std::placeholders::_1));  
  
  gesture_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>( //手势识别订阅
      "/hobot_hand_gesture_detection", 10,
      std::bind(&ArmNode::GestureCallback, this, std::placeholders::_1));


  iot_subscription_ = this->create_subscription<std_msgs::msg::String>(   //通过云端或手机App对机器人进行远程控制
      "cmd_mo", 
      10, 
      std::bind(&ArmNode::iot_callback, this, std::placeholders::_1));   //创建订阅器，订阅cmd_mo话题，接收来自物联网系统的指令，并执行相应的操作
}

ArmNode::~ArmNode() {}  //ROS2节点关闭时,释放资源

int ArmNode::position_to_angle(int position) {   //将位置转角度
    std::cout << "Position: " << position << std::endl;  // 打印 position 的值
    int angle;
    
    if (0 <= position && position <= 100) {
        angle = position * 1.8;
    } else {
        // -99 到 -1 对应 180 度到 360 度
        angle = 360 + position * 1.8;
    }

    // 确保角度在 0 到 360 度之间
    //angle = std::fmod(angle, 360);
    return angle;
}

int ArmNode::Operate() {

  const char* device = "/dev/ttyS1";  // 修改为实际存在的串口设备
  int baud = 115200;  // 设置波特率为115200
  int serial_fd;

  if ((serial_fd = serialOpen(device, baud)) < 0) {
            std::cerr << "无法打开串口" << std::endl;
            RCLCPP_ERROR(this->get_logger(), "ERROR!");
            return -1;
        }

  //将数据逐字节写入串口
  for (const char c : serial_data_) {
      serialPutchar(serial_fd, c);
  }

  serialClose(serial_fd);  // 关闭串口

  RCLCPP_WARN(rclcpp::get_logger("arm_node"), "Operate Succeed!");
  return 0;
}

int ArmNode::Operate1() {

  const char* device = "/dev/ttyS5";  // 修改为第二个串口设备
  int baud = 115200;  // 设置波特率为115200
  int serial_fd;

  if ((serial_fd = serialOpen(device, baud)) < 0) {
            std::cerr << "无法打开串口" << std::endl;
            RCLCPP_ERROR(this->get_logger(), "ERROR!");
            return -1;
        }

  //将数据逐字节写入串口
  for (const char c : serial_data_1) {
      serialPutchar(serial_fd, c);
  }

  serialClose(serial_fd);  // 关闭串口

  RCLCPP_WARN(rclcpp::get_logger("arm_node"), "Operate1 Succeed!");
  return 0;
}

//机械臂将垃圾放入垃圾桶
void ArmNode::on_timer5() {
    //RCLCPP_INFO(this->get_logger(), "The robotic arm has been raised");
    //is_hbcap = false;
    //serial_data_ = "\xAA\x5A\x2D\x00\x2D\x5A\x32\x0F\x78\x96\x3C\x5A\x5A\x5A\x5A\x5A\x5A\x22\x55";
    //serial_data_ = serial_data_list[count];
        
    //     扔入垃圾桶   AA5A2D002D5A96148C5A780B5A5A5A5A5A55  AA5A2D002D5A96148C5A785A5A5A5A5A5A55
    //  AA5A2D002D5A96878CA05A0B5A5A5A5A5A55  AA5A2D002D5A5A87B4A05A0B5A5A5A5A5A55
    serial_data_1 = "7";
    Operate1();
    if(count == 1)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x96, 0x14, 0x8C, 0x5A, 0x78, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 2)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x96, 0x14, 0x8C, 0x5A, 0x78, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 3)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x96, 0x87, 0x8C, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    //0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55
    else if(count == 4)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 5)
    { 
      serial_data_ = "br";
      Operate();
    }
    else if(count == 6)
    {
      serial_data_ = "st";
      Operate();
      serial_data_1 = "1";
      Operate1();
    }
    count++;
    if(count == 7){
      count = 1;
      timer_5.reset();
      is_lay_down = false;
    }
}

//机械臂放下物体
void ArmNode::on_timer3() {
    //RCLCPP_INFO(this->get_logger(), "The robotic arm has been raised");
    //is_hbcap = false;
    //serial_data_ = "\xAA\x5A\x2D\x00\x2D\x5A\x32\x0F\x78\x96\x3C\x5A\x5A\x5A\x5A\x5A\x5A\x22\x55";
    //serial_data_ = serial_data_list[count];
    
    // 放下物体收手 AA5A2D002D5A5A3796965A005A5A5A5A5A55  AA5A2D002D5A410F6E963C005A5A5A5A5A55  AA5A2D002D5A410F6E963C5A5A5A5A5A5A55
    // AA5A2D002D5A5A87B4A05A2D5A5A5A5A5A55   AA5A2D002D5A5A87B4A05A0B5A5A5A5A5A55
    serial_data_1 = "1";
    Operate1();
    if(count == 1)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x37, 0x96, 0x96, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 2)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x41, 0x0F, 0x6E, 0x96, 0x3C, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 3)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x41, 0x0F, 0x6E, 0x96, 0x3C, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    //0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55
    else if(count == 4)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x2D, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 5)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    count++;
    if(count == 6){
      count = 1;
      timer_3.reset();
      is_lay_down = false;
    }
}

//机械臂预放苹果
void ArmNode::on_timer() {
    //RCLCPP_INFO(this->get_logger(), "The robotic arm has been raised");
    //is_hbcap = false;
    //serial_data_ = "\xAA\x5A\x2D\x00\x2D\x5A\x32\x0F\x78\x96\x3C\x5A\x5A\x5A\x5A\x5A\x5A\x22\x55";
    //serial_data_ = serial_data_list[count];
    serial_data_1 = "7";
    Operate1();
    if(count == 1)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x3A, 0x0A, 0x6E, 0x82, 0x41, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 2)
    {  //AA5A2D002D5A410A6E82410B5A5A5A5A5A55
       //AA5A2D002D5A3A0A6E82415A5A5A5A5A5A55
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x3A, 0x0A, 0x6E, 0x82, 0x41, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    // else if(count == 3)
    // {
    //   serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x2D, 0x96, 0x96, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
    //   Operate();
    // }
    //0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55
    else if(count == 3)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    count++;
    if(count == 4){

      count = 1;
      timer_.reset();
      is_lay_down = true;
    }
}

//机械臂预放瓶子
void ArmNode::on_timer2() {
    //RCLCPP_INFO(this->get_logger(), "The robotic arm has been raised");
    //is_hbcap = false;
    //serial_data_ = "\xAA\x5A\x2D\x00\x2D\x5A\x32\x0F\x78\x96\x3C\x5A\x5A\x5A\x5A\x5A\x5A\x22\x55";
    //serial_data_ = serial_data_list[count];
    serial_data_1 = "7";
    Operate1();
    if(count == 1)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x41, 0x0F, 0x6E, 0x96, 0x3C, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 2)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x41, 0x0F, 0x6E, 0x96, 0x3C, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    // else if(count == 3)
    // {
    //   serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x2D, 0x96, 0x96, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
    //   Operate();
    // }
    //0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55
    else if(count == 3)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    count++;
    if(count == 4){
      count = 1;
      timer_2.reset();
      is_lay_down = true;
    }
}

//机械臂归位
void ArmNode::on_timer4() {
    //RCLCPP_INFO(this->get_logger(), "The robotic arm has been raised");
    //is_hbcap = false;
    //serial_data_ = "\xAA\x5A\x2D\x00\x2D\x5A\x32\x0F\x78\x96\x3C\x5A\x5A\x5A\x5A\x5A\x5A\x22\x55";
    //serial_data_ = serial_data_list[count];
        
    serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
    Operate();

    timer_4.reset();

}

//手势控制小车
void ArmNode::on_timer8() {
    //RCLCPP_INFO(this->get_logger(), "The robotic arm has been raised");
    //is_hbcap = false;
    //serial_data_ = "\xAA\x5A\x2D\x00\x2D\x5A\x32\x0F\x78\x96\x3C\x5A\x5A\x5A\x5A\x5A\x5A\x22\x55";
    //serial_data_ = serial_data_list[count];
    serial_data_1 = "3";
    Operate1();
    if(serial_data_ == "go"){
      Operate();
      sleep(2);
    }
    else if(serial_data_ == "br")
    {
      Operate();
      sleep(2);
    }
    else if(serial_data_ == "qw")
    {
      Operate();
      sleep(2);
    }
    else if(serial_data_ == "er")
    {
      Operate();
      sleep(2);
    }
    serial_data_ = "st";
    Operate();
    serial_data_1 = "1";
    Operate1();
    run_or_not = true;
    timer_8.reset();
}

//检测核辐射
void ArmNode::on_timer6() {
    //检测核辐射 AA5A2D002D5AB45A9678B40B5A5A5A5A5A55   AA5A2D002D5AB45AB494B40B5A5A5A5A5A55
    // AA5A2D002D5AB45A7878B40B5A5A5A5A5A55    AA5A2D002D5A5A87B4A05A0B5A5A5A5A5A55
    // AA5A2D002D5AB45AB494B40B5A5A5A5A5A55
    //AA5A2D002D5AB450B482B40B5A5A5A5A5A55
    serial_data_1 = "7";
    Operate1();
    if(count == 1)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0xB4, 0x5A, 0x96, 0x78, 0xB4, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 2)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0xB4, 0x50, 0xB4, 0x82, 0xB4, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
      sleep(6);
    }
    else if(count == 3)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0xB4, 0x5A, 0x78, 0x78, 0xB4, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    else if(count == 4)
    {
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    count++;
    if(count == 5){
      count = 1;
      timer_6.reset();
    }
}

//打招呼
void ArmNode::on_timer7() {
    //打招呼 AA056E2850B45A87B4A05A0B367E7E7E7E55   AA0578145AB45A87B4A05A0B367E7E7E7E55
    serial_data_1 = "3";
    Operate1();
    if(count == 1)
    {
      serial_data_ = {0xAA, 0x05, 0x78, 0x14, 0x5A, 0xB4, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x36, 0x7E, 0x7E, 0x7E, 0x7E, 0x55};
      Operate();
    }
    else if(count == 2)
    {///
      serial_data_ = {0xAA, 0x05, 0x6E, 0x28, 0x50, 0xB4, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x36, 0x7E, 0x7E, 0x7E, 0x7E, 0x55};
      Operate();
    }
    else if(count == 3)
    {
      serial_data_ = {0xAA, 0x05, 0x78, 0x14, 0x5A, 0xB4, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x36, 0x7E, 0x7E, 0x7E, 0x7E, 0x55};
      Operate();
    }
    else if(count == 4)
    {///
      serial_data_ = {0xAA, 0x05, 0x6E, 0x28, 0x50, 0xB4, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x36, 0x7E, 0x7E, 0x7E, 0x7E, 0x55};
      Operate();
    }
    else if(count == 5)
    {
      serial_data_ = {0xAA, 0x05, 0x78, 0x14, 0x5A, 0xB4, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x36, 0x7E, 0x7E, 0x7E, 0x7E, 0x55};
      Operate();
    }
    else if(count == 6)
    {///
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
    }
    count++;
    if(count == 7){
      count = 1;
      timer_7.reset();
    }
}

//根据超声波传感器的数据，实现一个简单的前向移动和遇障停止的功能
void ArmNode::csDisCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  
  if (is_go && ((msg->data) >= 40))
  {
    //RCLCPP_INFO(this->get_logger(), "接收到的超声波数据: %d cm", msg->data);
    serial_data_ = "go";
    Operate();
  }
  else if (is_go && ((msg->data) < 40))
  {
    //RCLCPP_INFO(this->get_logger(), "接收到的超声波数据: %d cm", msg->data);
    serial_data_ = "st";
    Operate();
    serial_data_1 = "1";
    Operate1();
    is_go = false;
//     if(is_lay_down = true){
//       //使用定时器在3秒后调用on_timer函数
//       timer_3 = create_wall_timer(
//       //std::chrono::seconds(2) + std::chrono::milliseconds(500),
//       std::chrono::seconds(3),
//       std::bind(&ArmNode::on_timer3, this));
//     }
  }
}

void ArmNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  orientation = msg->orientation.z;
  //std::cout << "imu:tuto_doa:" <<tuto_doa<< std::endl;
  int microphone_angle = 0;
  if (isNumber && is_doa){
    //std::cout << "imu:tuto_doa:" <<tuto_doa<< std::endl;
    microphone_angle = std::stoi(object_);
    //std::cout << "object_:" <<object_<< std::endl;
    if (microphone_angle>=0 && microphone_angle<=180){
        if(is_yichu && (orientation > 180)){orientation -= 360;}  //如果溢出，则将orientation减去360
        if(orientation >= tuto_doa)
        {
          std::cout << "tuto_doa:" <<tuto_doa<< std::endl;
          std::cout << "orientation:" <<orientation<< std::endl;
          serial_data_ = "st";  // 假设 'l' 代表左转的串口数据
          Operate();
          serial_data_1 = "1";
          Operate1();
          std::cout << "停车" << std::endl;
          isNumber = false;
          is_doa = false;
          is_yichu = false;
          is_go = true;
          object_ = "kezihu\r";
        }
        if(tuto_doa > orientation){
          serial_data_ = "li";  // 假设 'l' 代表左转的串口数据
          Operate();
          serial_data_1 = "3";
          Operate1();
          std::cout << "1左转 " << std::endl;
        }
    }
    else if (microphone_angle>=180 && microphone_angle <= 360){
        if(is_buzu && (orientation < 180)){orientation +=360;}  //如果不足，则将orientation加上360
        if(orientation <= tuto_doa)
        {
          std::cout << "tuto_doa:" <<tuto_doa<< std::endl; 
          std::cout << "orientation:" <<orientation<< std::endl;
          serial_data_ = "st";  // 假设 'st' 代表停止的串口数据
          Operate();
          serial_data_1 = "1";
          Operate1();
          std::cout << "停车" << std::endl; 
          isNumber = false;
          is_doa = false;
          is_buzu = false;
          is_go = true;
          object_ = "kezihu\r";
        }
        if(tuto_doa < orientation){
          serial_data_ = "ri";  // 假设 'ri' 代表右转的串口数据
          Operate();
          serial_data_1 = "3";
          Operate1();
          std::cout << "2右转 " << std::endl;
        }
        
    }
  }
  //std::cout << "Position111111: " << orientation << std::endl;  // 打印 position 的值
  //orientation = position_to_angle(100*msg->orientation.z);
  //RCLCPP_INFO(this->get_logger(), "Orientation z-axis: %d", orientation);
  RCLCPP_INFO(this->get_logger(), "Orientation z-axis: %d", orientation);
}

//订阅语音消息改变抓取目标
void ArmNode::UpdateObjectCallback(const std_msgs::msg::String::SharedPtr msg) {
    object_ = msg->data;
    is_hbcap = true;
    RCLCPP_INFO(this->get_logger(), "Updated object type to: %s", object_.c_str());
    // 检查字符串是否为数字
    isNumber = false;
    for (char c : object_) {
        if (isdigit(c)) {
            isNumber = true;
            break;
        }
    }
    int number;
    is_doa = false;  //是否到达目标角度
    is_yichu = false;  //是否溢出
    is_buzu = false;  //是否不足
    if (isNumber) {
      number = std::stoi(object_);
      std::cout << "number:" <<number<< std::endl;
      if (number>=0 && number<=180){
        tuto_doa = orientation + number;  //目标角度 = 当前角度 + 目标角度
        if(tuto_doa >= 360){
          tuto_doa = tuto_doa - 360;
          is_yichu = true;
        }
      }
      else if (number>180 && number<=360){
        tuto_doa = orientation + number - 360;
        if(tuto_doa < 0){
          tuto_doa += 360;
          is_buzu = true;
        }
      }
      is_doa = true;
    }
    if(object_ == "go"){
      serial_data_ = "go";
      Operate();
      serial_data_1 = "3";
      Operate1();
    } else if (object_ == "ht"){
      serial_data_ = "br";
      Operate();
      serial_data_1 = "3";
      Operate1();
    } else if(object_ == "none"){
      serial_data_ = "st";
      Operate();
      serial_data_1 = "1";
      Operate1();
    } else if(object_ == "left"){
      serial_data_ = "li";
      Operate();
      serial_data_1 = "3";
      Operate1();
    } else if(object_ == "right"){
      serial_data_ = "ri";
      Operate();
      serial_data_1 = "3";
      Operate1();
    } else if(object_ == "xy"){
      serial_data_1 = "6";
      Operate1();
    } else if(object_ == "yfs"){
      serial_data_1 = "4";
      Operate1();
    } else if(object_ == "ssb"){
      ss_sb = true;
    } else if(object_ == "gsb"){
      ss_sb = false;
    } else if(object_ == "jy"){
      serial_data_ = {0xAA, 0x5A, 0x6E, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x36, 0x36, 0x36, 0x36, 0x36, 0x55};
      Operate();
      serial_data_1 = "3";
      Operate1();
      //使用定时器在3秒后调用on_timer函数
      timer_4 = create_wall_timer(
      std::chrono::seconds(6),
      std::bind(&ArmNode::on_timer4, this));
    } else if(object_ == "ws"){
      serial_data_ = {0xAA, 0x1E, 0xB4, 0x28, 0x5A, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x32, 0x96, 0x96, 0x96, 0x96, 0x55};
      Operate();
      serial_data_1 = "3";
      Operate1();
      timer_4 = create_wall_timer(
      std::chrono::seconds(6),
      std::bind(&ArmNode::on_timer4, this));
    } else if(object_ == "wq"){
      serial_data_ = {0xAA, 0x5A, 0x6E, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x7E, 0x36, 0x36, 0x36, 0x36, 0x55};
      Operate();
      serial_data_1 = "7";
      Operate1();
      timer_4 = create_wall_timer(
      std::chrono::seconds(6),
      std::bind(&ArmNode::on_timer4, this));
    } else if(object_ == "jd"){
      serial_data_ = {0xAA, 0x5A, 0x6E, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x7E, 0x7E, 0x7E, 0x36, 0x36, 0x55};
      Operate();
      serial_data_1 = "7";
      Operate1();
      timer_4 = create_wall_timer(
      std::chrono::seconds(6),
      std::bind(&ArmNode::on_timer4, this));
    } else if(object_ == "szzk"){
      serial_data_ = {0xAA, 0x5A, 0x6E, 0x00, 0x2D, 0x00, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x36, 0x7E, 0x7E, 0x7E, 0x7E, 0x55};
      Operate();
      serial_data_1 = "7";
      Operate1();
      timer_4 = create_wall_timer(
      std::chrono::seconds(6),
      std::bind(&ArmNode::on_timer4, this));
    } else if(object_ == "gw"){
      serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
      Operate();
      serial_data_1 = "5";
      Operate1();
    } else if(object_ == "dzh"){
      //打招呼 AA056E2850B45A87B4A05A0B367E7E7E7E55   AA0578145AB45A87B4A05A0B367E7E7E7E55
      serial_data_ = {0xAA, 0x05, 0x6E, 0x28, 0x50, 0xB4, 0x5A, 0x87, 0xB4, 0xA0, 0x5A, 0x00, 0x36, 0x7E, 0x7E, 0x7E, 0x7E, 0x55};
      Operate();
      serial_data_1 = "3";
      Operate1();
      sleep(1.7);
      timer_7 = create_wall_timer(
      //std::chrono::seconds(1) + std::chrono::milliseconds(700),
      //std::chrono::milliseconds(500),
      std::chrono::seconds(1),
      std::bind(&ArmNode::on_timer7, this));
    } else if(object_ == "fx"){
      timer_3 = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ArmNode::on_timer3, this));
    } else if(object_ == "fsjc"){
      timer_6 = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ArmNode::on_timer6, this));
    } else if(object_ == "sbody"){
      //std::cout << "开启跟人" << std::endl;
      object_ = "body";
      std::thread([](){
         std::system("/root/start_body.sh");
     }).detach();
    } else if(object_ == "unbody"){
      std::system("pkill -f 'ros2 launch hobot_falldown_detection hobot_falldown_detection.launch.py'");
    }
}


//用于处理来自话题的消息  寻找 -> 对准 -> 前进 -> 抓取 -> 返回
void ArmNode::SmartTopicCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {

  bool object_found = false;   //画面是否有需要抓取的物体
  bool in_center_position = false;  //该物体是否在画面中间
  bool is_target_object = false;  // 表示检测到的物体是否是目标物体

  //1
  float max_area = 0.0f;  // 最大的面积
  float max_width = 0.0f;  // 最大的边框宽度
  ai_msgs::msg::Roi largest_roi;  // 最大的边框

  // 定义固定的串口数据  预抓1 
  //std::string fixed_serial_data = "\xAA\x5A\x2D\x00\x2D\x5A\x5A\x2D\x96\x96\x5A\x5A\x5A\x5A\x5A\x5A\x5A\x22\x55";
  
  std::stringstream ss;
  ss << "Recved ai msg"  
     << ", frame_id: " << msg->header.frame_id   
     << ", stamp: " << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec     
     << ", targets size: " << msg->targets.size() << "\n";    

    for (const auto& tar : msg->targets) {   
      ss << " has roi num: " << tar.rois.size();   

      for (const auto& roi : tar.rois) { 
        if(object_ == "none"){  
          ss << "Stopping car";
          // Add your car stopping code here
          serial_data_ = "st";  
          Operate();  
          serial_data_1 = "1";  
          Operate1();
          object_ = "none1";
        }
        else if (roi.type == object_) {
          object_found = true;
          ss << ", roi type: " << roi.type;
          float area = roi.rect.width*roi.rect.height;
          
          if (area > max_area) {
            max_area = area;
            max_width = roi.rect.width;
            largest_roi = roi;
          }
        }
      }
    }
    if (object_found && max_width > 0.0f){      //如果检测到物体且检测框宽度大于0
      int roi_center_x = largest_roi.rect.x_offset + largest_roi.rect.width / 2;    //计算检测框的中心点

      if (roi_center_x >= left_thresholds_[object_] && roi_center_x <= right_thresholds_[object_]) {  //如果检测框中心点在左右阈值之间
        in_center_position = true;
      } else if (roi_center_x < left_thresholds_[object_]) {  //如果检测框中心点小于左阈值，左转
        ss << ", turning left";
        serial_data_ = "li";
        Operate();
        serial_data_1 = "3";
        Operate1();
      } else {  //如果检测框中心点大于右阈值，右转
        ss << ", turning right";
        serial_data_ = "ri";
        Operate();
        serial_data_1 = "3";
        Operate1();
      }

    // 检查检测框大小
    if (in_center_position && (largest_roi.rect.width >= stop_thresholds_[object_]) && is_hbcap) {
      in_center_position = false;
      is_hbcap = false;
      ss << ", stopping car due to large object size";
      //std::cout << "Stopping car due to large object size\n";
      serial_data_ = "st";
      Operate();
      serial_data_1 = "1";
      Operate1();
      sleep(0.01);

      if(object_ == "apple\r")
      {
        //驱动机械臂抓取   AA5A2D002D5A5A3796965A5A5A5A5A5A5A55
        serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x37, 0x96, 0x96, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
        Operate();
        //使用定时器在3秒后调用on_timer函数
        timer_ = create_wall_timer(
        //std::chrono::seconds(2) + std::chrono::milliseconds(500),
        std::chrono::seconds(2),
        std::bind(&ArmNode::on_timer, this));
      } else if(object_ == "kezihu\r") {
        if(is_lay_down){
          //使用定时器在3秒后调用on_timer函数
          timer_3 = create_wall_timer(
          //std::chrono::seconds(2) + std::chrono::milliseconds(500),
          std::chrono::seconds(2),
          std::bind(&ArmNode::on_timer3, this));
        }
      } else if(object_ == "trash_can\r") {
        //使用定时器在3秒后调用on_timer函数
        timer_5 = create_wall_timer(
        //std::chrono::seconds(2) + std::chrono::milliseconds(500),
        std::chrono::seconds(2),
        std::bind(&ArmNode::on_timer5, this));
      } else if(object_ == "bandeng\r") {
        //使用定时器在3秒后调用on_timer函数
        timer_3 = create_wall_timer(
        //std::chrono::seconds(2) + std::chrono::milliseconds(500),
        std::chrono::seconds(2),
        std::bind(&ArmNode::on_timer3, this));
      } 
      else {
        //驱动机械臂抓取
        serial_data_ = {0xAA, 0x5A, 0x2D, 0x00, 0x2D, 0x5A, 0x5A, 0x2D, 0x96, 0x96, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x55};
        Operate();
        //使用定时器在3秒后调用on_timer函数
        timer_2 = create_wall_timer(
        //std::chrono::seconds(2) + std::chrono::milliseconds(500),
        std::chrono::seconds(2),
        std::bind(&ArmNode::on_timer2, this));
      }
      object_ = "none1";
    }
  }
//}
  for (const auto& target : target_objects) {
    if (object_ == target) {
      is_target_object = true;
      std::cout << "object_: " << object_ << std::endl;
      break;
    }
  }

  if (!object_found && is_target_object) {
    ss << "Object not found, rotating to find object";
    serial_data_ = "li";
    Operate();
    serial_data_1 = "3";
    Operate1();
    // Add your car rotating code here
  } else if (in_center_position) {
    ss << "Object in center position, qianjin";
    // Add your car stopping code here
    serial_data_ = "go";
    Operate();
    serial_data_1 = "3";
    Operate1();
  }

  RCLCPP_INFO(rclcpp::get_logger("TrashGradNode"), "%s", ss.str().c_str());
}

// arm_node.cpp
// arm_node.cpp

// 假设 BodyDetectionMsg 是消息类型，您需要根据实际使用的消息类型来替换
void ArmNode::BodyDetectionCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
//void ArmNode::BodyDetectionCallback(const std_msgs::msg::String::SharedPtr msg) {
  // 如果object_的值为"none"，则停车
  bool man_cent = false;
  bool man_dis = false;
  bool body_found = false;
  int max_area = 0;
  ai_msgs::msg::Roi largest_roi;  // 最大的边框
  auto message = std::make_unique<std_msgs::msg::String>();
  for (const auto& target : msg->targets) {
    // 遍历每个目标的属性
    for (const auto& attribute : target.attributes) {
      if (attribute.type == "falldown") {
        // 读取 falldown的值
        float falldown_value = attribute.value;
        //RCLCPP_INFO(this->get_logger(), "Falldown value: %f", falldown_value);
        if(falldown_value == 1.0f){
          countsd++;
          if(countsd > 5){
            serial_data_ = "st";  // 假设 'st' 代表停车的串口数据
            Operate();
            serial_data_1 = "4";
            Operate1();
            message->data = "警报，有人摔倒了";
            tts_text_publisher_->publish(std::move(message));
            sleep(3);
            countsd = 0;
            //return;  // 停车后退出回调函数
          }
        }
        if(falldown_value == 0.0f){
          countsd = 0;
        }
      }
    }
  }

  if (object_ == "none") {
    RCLCPP_INFO(this->get_logger(), "Object is set to none, stopping the operation.");
    object_ = "none1";
    serial_data_ = "st";  // 假设 'st' 代表停车的串口数据
    Operate();
    serial_data_1 = "1";
    Operate1();
    return;  // 停车后退出回调函数
  }
  if ((object_ != "none") && (object_ !="body")) {
    return;  //退出回调函数
  }
  if (object_ == "body") {
    // 检查消息中是否包含 body 类型的目标
    for (const auto& target : msg->targets) {
      for (const auto& roi : target.rois) {
        if (roi.type == "body") {  // 检查 roi.type 是否为 "body" 
          body_found = true;
          int area = roi.rect.width*roi.rect.height;
          if(area > max_area){
            max_area = area;
            largest_roi = roi;
          }
        }
      }
    }
    if (body_found && largest_roi.rect.width > 0){
          int x_offset = largest_roi.rect.x_offset;
          int width = largest_roi.rect.width;

          // 计算 body 的中心点
          int center = x_offset + width / 2;

          // 判断中心点是否在图像中间160-480之间
          if (center < 260) {
            // 中心点小160，左转
            serial_data_ = "li";  // 假设 'li' 代表左转的串口数据
            Operate();
            //std::cout << "1左转 " << std::endl; 
            serial_data_1 = "7";
            Operate1();
          } else if (center > 380) {
            // 中心点大于480，右转
            serial_data_ = "ri";  // 假设 'ri' 代表右转的串口数据
            Operate();
            //std::cout << "2右转 " << std::endl;
            serial_data_1 = "7";
            Operate1();
          }
          else {
            man_cent = true;
            // 根据 body 的宽度判断前进或后退
            if (width > 220) {
              // 宽度大于 500，后退
              serial_data_ = "br";  // 假设 'br' 代表后退的串口数据
              //std::cout << "后退 " << std::endl;
              serial_data_1 = "7";
              Operate1();
            } else if (width < 160) {
              // 宽度小于 400，前进
              serial_data_ = "go";  // 假设 'go' 代表前进的串口数据
              //std::cout << "前进 " << std::endl;
              serial_data_1 = "7";
              Operate1();
            }
            else{
              man_dis = true;
            }
            Operate();
          }
          if(man_dis){
            serial_data_ = "st";
            Operate();
            serial_data_1 = "1";
            Operate1();
          }
          // 执行操作
          //Operate();

          // 打印调试信息
          RCLCPP_INFO(this->get_logger(), "Body center: %d, width: %d", center, width);
          //break;  // 假设每个 target 只有一个 body，找到后即退出循环
        
      }
    }

    // 如果没有找到 body，执行其他操作，例如旋转寻找
    if (!body_found) {
      RCLCPP_INFO(this->get_logger(), "Body not found, rotating to find");
      serial_data_ = "li";  // 假设 'li' 代表旋转寻找的串口数据
      Operate();
      serial_data_1 = "7";
      Operate1();
    }
}

void ArmNode::GestureCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  if (ss_sb) {
    for (const auto& target : msg->targets) {
      // 遍历每个目标的属性
      for (const auto& attribute : target.attributes) {
        if (attribute.type == "gesture") {
          // 读取 falldown的值
          float gesture_value = attribute.value;
          RCLCPP_INFO(this->get_logger(), "1111111111111111111111Gesture_value: %f", gesture_value);

          if(run_or_not && gesture_value == 2.0f){  //加油  前进
            serial_data_ = "go";  // 假设 'go' 代表前进的串口数据
            //Operate();
            run_or_not = false;
            timer_8 = create_wall_timer(
            //std::chrono::seconds(2) + std::chrono::milliseconds(500),
            //std::chrono::seconds(3),
            std::chrono::milliseconds(100),
            std::bind(&ArmNode::on_timer8, this));
            RCLCPP_INFO(this->get_logger(), "Gesture_value: %f", gesture_value);
          } else if (run_or_not && gesture_value == 3.0f){  //V字形  后退
            serial_data_ = "br";  // 假设 'br' 代表后退的串口数据
            //Operate();
            run_or_not = false;
            timer_8 = create_wall_timer(
            //std::chrono::seconds(2) + std::chrono::milliseconds(500),
            //std::chrono::seconds(3),
            std::chrono::milliseconds(100),
            std::bind(&ArmNode::on_timer8, this));
            RCLCPP_INFO(this->get_logger(), "Gesture_value: %f", gesture_value);
          } else if (run_or_not && gesture_value == 12.0f){
            serial_data_ = "er";  // 假设 'er' 代表停车的串口数据
            //Operate();
            run_or_not = false;
            timer_8 = create_wall_timer(
            //std::chrono::seconds(2) + std::chrono::milliseconds(500),
            //std::chrono::seconds(3),
            std::chrono::milliseconds(100),
            std::bind(&ArmNode::on_timer8, this));
            RCLCPP_INFO(this->get_logger(), "Gesture_value: %f", gesture_value);
          } else if (run_or_not && gesture_value == 13.0f){
            serial_data_ = "qw";  // 假设 'qw' 代表停车的串口数据
            //Operate();
            run_or_not = false;
            timer_8 = create_wall_timer(
            //std::chrono::seconds(2) + std::chrono::milliseconds(500),
            //std::chrono::seconds(3),
            std::chrono::milliseconds(100),
            std::bind(&ArmNode::on_timer8, this));
            RCLCPP_INFO(this->get_logger(), "Gesture_value: %f", gesture_value);
          } 
        }
      }
    }
  }
}

std::vector<uint8_t> parseHexString(const std::string &hexString) { 
    std::vector<uint8_t> result;
    std::string byteString;
    for (size_t i = 0; i < hexString.length(); ++i) {
        // 忽略 '{' 和 '}'
        if (hexString[i] == '{' || hexString[i] == '}') {
            continue;
        }

        // 读取两个字符来表示一个字节
        if (hexString[i] == '0' && hexString[i+1] == 'x') {
            byteString = hexString.substr(i, 4); // 例如 "0xAA"
            uint8_t byteValue = static_cast<uint8_t>(std::stoi(byteString.substr(2), nullptr, 16));
            result.push_back(byteValue);
            i += 3; // 跳过 '0x' 和后面的两个字符
        }
    }

    return result;
}

void ArmNode::iot_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // 处理接收到的消息
    RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());

    // 根据消息内容进行判断
    if (msg->data == "go" || msg->data == "st" || msg->data == "li" ||
        msg->data == "ri" || msg->data == "br" || msg->data == "er" || msg->data == "qw") {
        
        // 处理字符串类型指令
        RCLCPP_INFO(this->get_logger(), "Received command: %s", msg->data.c_str());

        serial_data_ = msg->data;
    } else {
        // 假设收到的数据是十六进制数组格式
        std::vector<uint8_t> byteArray = parseHexString(msg->data);
        
        // 将数据存入 serial_data_（假设 serial_data_ 是一个 std::vector<uint8_t> 类型）
        serial_data_.assign(byteArray.begin(), byteArray.end());

        // 打印出接收到的十六进制数据
        for (const auto &byte : serial_data_) {
            std::cout << "0x" << std::hex << std::uppercase << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;
    }

    // 执行与串口通信相关的操作
    Operate();
}