#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

namespace image_viewer
{
class ImageViewerNode : public rclcpp::Node
{
public:
  ImageViewerNode() : Node("image_viewer_node"), it_(shared_from_this())
  {
    // 使用 image_transport 订阅图像话题
    image_sub_ = it_.subscribe(
      "/image",  // 图像话题名称
      10,  // 队列大小
      &ImageViewerNode::image_callback,  // 回调函数
      this  // 回调函数的上下文
    );
    RCLCPP_INFO(this->get_logger(), "Ready to view images.");
  }

private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
  {
    try
    {
      // 使用cv_bridge转换图像消息为cv::Mat
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
      cv::imshow("Image window", cv_ptr->image);
      cv::waitKey(1); // 1毫秒内如果有按键则处理
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "CvBridge Error: %s", e.what());
    }
  }
};
}  // namespace image_viewer
