#include "image_viewer/image_viewer_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<image_viewer::ImageViewerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}