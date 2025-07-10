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

#ifndef ARM_NODE_H_
#define ARM_NODE_H_

#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ctime>
#include <thread>
#include <cstdio>
#include <stdexcept>
#include <array>
#include "ai_msgs/msg/perception_targets.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include<wiringPi.h>
#include<wiringSerial.h>
#include "sensor_msgs/msg/imu.hpp"
#include <cctype> // 包含字符处理函数
#include "std_msgs/msg/int32.hpp"  // 添加消息类型的头文件
#include <nlohmann/json.hpp>

using rclcpp::NodeOptions;

using json = nlohmann::json;  //

#define PI 3.1415926f
#define l1 0.1035f
#define l2 0.088f
#define l3 0.17f

class ArmNode : public rclcpp::Node {
 public:

  ArmNode(const std::string &node_name,
        const NodeOptions &options = NodeOptions());

  virtual ~ArmNode();

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::TimerBase::SharedPtr timer_2;

  rclcpp::TimerBase::SharedPtr timer_3;

  rclcpp::TimerBase::SharedPtr timer_4;

  rclcpp::TimerBase::SharedPtr timer_5;

  rclcpp::TimerBase::SharedPtr timer_6;

  rclcpp::TimerBase::SharedPtr timer_7;

  rclcpp::TimerBase::SharedPtr timer_8;
  
  // 执行运行流程
  int Run();

  int Encode(float theta0, float theta1, float theta2, float theta3, float theta4, float theta5, float theta6);

  int Model2D(float x, float y, float alpha);

  int Model(float x, float y, float z);
  
  int Operate();
  int Operate1();
  int position_to_angle(int position);

  int count_ = 7;
  int count  = 1;
  int countsd = 40;
  float targetX_ = 0.15;
  float targetY_ = 0.0;
  float targetZ_ = -0.08;

  std::string object_ = "none"; // 添加成员变量
  bool is_hbcap = false;
  bool isNumber = false;
  bool is_doa = false;
  bool is_yichu = false;  //是否溢出
  bool is_buzu = false;  //是否不足
  int tuto_doa = 0;  //测算要旋转到的角度
  int doa_save = 0;  
  bool is_go = false;  //转完向之后前进
  bool is_lay_down = false;  //是否需要放下
  bool run_or_not = true;  //判断手势识别是否运行完毕
  bool ss_sb = false;  //判断手势识别是否运行完毕
  std::vector<float> thetas_ = std::vector<float>(6);

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  private:
   std::string ai_msg_sub_topic_name_ = "/hobot_dnn_detection";
   rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
    smart_subscription_ = nullptr;

   rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr gesture_subscription_ = nullptr;

   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr update_object_subscription_;

   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr iot_subscription_;

   rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr body_detection_subscription_ = nullptr;

   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tts_text_publisher_;
   // 超声波数据订阅器
   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cs_dis_subscription_;
   int orientation;
   std::string serial_data_ = "\xAA\x5A\x2D\x00\x2D\x5A\x5A\x2D\x96\x96\x5A\x5A\x5A\x5A\x5A\x5A\x5A\x22\x55";
   std::string serial_data_1 = "";


   std::vector<int> ruler_ = {1074, 1032, 992, 960, 933, 909, 888, 870, 850, 838};

   // 定义目标物体名称的字符串数组
   std::vector<std::string> target_objects = {"apple\r", "tizhi\r", "yibao\r", "juhuacha\r", "maido\r", "coke\r","trash_can\r","bandeng\r","kezihu\r"};

    // 定义存储阈值的unordered_map
    std::unordered_map<std::string, int> stop_thresholds_;  
    std::unordered_map<std::string, int> left_thresholds_;
    std::unordered_map<std::string, int> right_thresholds_;
    // 加载JSON文件并初始化阈值
     // 读取阈值并存储到 unordered_map 中
    void loadThresholds(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "无法打开文件: " << file_path << std::endl;
            return;
        }

        json thresholds_json;
        file >> thresholds_json;

        // 读取 stop_thresholds
        for (auto& [key, value] : thresholds_json["stop_thresholds"].items()) {
            stop_thresholds_[key + "\r"] = value;
        }

        // 读取 left_thresholds
        for (auto& [key, value] : thresholds_json["left_thresholds"].items()) {
            left_thresholds_[key + "\r"] = value;
        }

        // 读取 right_thresholds
        for (auto& [key, value] : thresholds_json["right_thresholds"].items()) {
            right_thresholds_[key + "\r"] = value;
        }
    }
//    std::unordered_map<std::string, int> stop_thresholds_ = {
//         {"apple\r", 78},
//         {"tizhi\r", 89},
//         {"yibao\r", 76},
//         //{"yibao", 46},
//         {"coke\r", 75},
//         {"juhuacha\r", 78},
//         {"maido\r", 84},
//         {"trash_can\r", 360},   //图像中心为300
//         {"bandeng\r", 125},  //图像中心为320
//         {"kezihu\r", 78}  //图像中心为320
//     };  // 不同物体的停车检测框大小阈值

//    std::unordered_map<std::string, int> left_thresholds_ = {
//         {"apple\r", 198},
//         {"tizhi\r", 213},
//         {"yibao\r", 210},
//         {"coke\r", 215},
//         {"juhuacha\r", 215},
//         {"maido\r", 210},
//         {"trash_can\r", 250},
//         {"bandeng\r", 270},
//         {"kezihu\r", 270}
//     };  // 不同物体的停车检测框左边阈值

//     std::unordered_map<std::string, int> right_thresholds_ = {
//         {"apple\r", 276},
//         {"tizhi\r", 313},
//         {"yibao\r", 316},
//         {"coke\r", 300},
//         {"juhuacha\r", 315},
//         {"maido\r", 310},
//         {"trash_can\r", 350},
//         {"bandeng\r", 370},
//         {"kezihu\r", 370}
//     };  // 不同物体的停车检测框右边阈值


   void on_timer();
   void on_timer2();
   void on_timer3();
   void on_timer4();
   void on_timer5();
   void on_timer6();
   void on_timer7();
   void on_timer8();

   void UpdateObjectCallback(const std_msgs::msg::String::SharedPtr msg);
   // 接受Agentnode话题信息
   void SmartTopicCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

   void BodyDetectionCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
   void GestureCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

   // 添加处理超声波数据的回调函数
   void csDisCallback(const std_msgs::msg::Int32::SharedPtr msg);

   void iot_callback(const std_msgs::msg::String::SharedPtr msg);
};   

#endif  // ARM_NODE_H_
