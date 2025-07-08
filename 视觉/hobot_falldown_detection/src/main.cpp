// Copyright (c) 2022，Horizon Robotics.
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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "include/hobot_falldown_detection/hobot_falldown_detection.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_WARN(rclcpp::get_logger("example"),
    "This is body_kps_subscriber example!!");

    auto node = std::make_shared<hobot_falldown_detection>();
    RCLCPP_WARN(rclcpp::get_logger("example"), "body_kps_subscriber init!");

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    RCLCPP_WARN(rclcpp::get_logger("example"), "body_kps_subscriber add_node!");
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
