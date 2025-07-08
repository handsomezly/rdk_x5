import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import base64
import requests
import json
from std_msgs.msg import String
import time

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription_cmd = self.create_subscription(
            String,
            '/cmd_ns',
            self.listener_callback_cmd,
            10)
        self.subscription_image = None  # 初始化为None
        self.send_image = False  # 定义send_image属性
        self.publisher_tts_text = self.create_publisher(String, 'tts_text', 10)
        self.get_logger().info('ImageSubscriber node initialized')

    def listener_callback_cmd(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        if msg.data == "image_ud":
            self.send_image = True
            if self.subscription_image is None or not self.subscription_image.is_alive():
                self.get_logger().info('Subscribing to image topic')
                self.subscription_image = self.create_subscription(
                    CompressedImage,
                    '/image',
                    self.listener_callback_image,
                    10)
            else:
                self.get_logger().info('Already subscribed to image topic')

    def listener_callback_image(self, msg):
        if self.send_image:
            self.get_logger().info('Sending video frame to API')
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # 将图像数据编码为Base64并发送给API
            image_base64 = base64.b64encode(np_arr).decode('utf-8')
            self.send_image_to_api(image_base64)
            self.send_image = False  # 重置标志位，等待下一个 "图像信息" 命令
        self.get_logger().info('Received image message')
        # 这里省略了发送图像到API的代码，因为它与问题无关

        # 处理完图像后取消订阅
        self.get_logger().info('Unsubscribing from image topic')
        self.subscription_image.destroy()
        self.subscription_image = None

    def send_image_to_api(self, image_base64):
        self.get_logger().info('Sending image to API')
        url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/image-understanding/request?access_token=24.69edf3c5891b746d63021aff80372778.2592000.1720343626.282335-79629271"

        payload = json.dumps({
            "image": image_base64,
            "question": "ls",
            "output_CHN": True
        })
        
        headers = {
            'Content-Type': 'application/json'
        }
        
        response = requests.request("POST", url, headers=headers, data=payload)
        self.get_logger().info('Received response from API')
        
        response_data = json.loads(response.text)
        self.task_id = response_data.get("result", {}).get("task_id")
        
        if self.task_id:
            self.get_logger().info(f"Received task_id: {self.task_id}")
            self.get_result_from_api()
        else:
            self.get_logger().error("Failed to get task_id from API response")

    def get_result_from_api(self):
        self.get_logger().info('Getting result from API')
        if not self.task_id:
            self.get_logger().error("Task_id is not available")
            return

        url = f"https://aip.baidubce.com/rest/2.0/image-classify/v1/image-understanding/get-result?access_token=24.69edf3c5891b746d63021aff80372778.2592000.1720343626.282335-79629271&task_id={self.task_id}"

        payload = json.dumps({"task_id": self.task_id})
        headers = {
            'Content-Type': 'application/json'
        }

        while True:
            response = requests.request("POST", url, headers=headers, data=payload)
            self.get_logger().info('Received result from API')
            response_data = json.loads(response.text)

            if response_data.get("result", {}).get("ret_code") == 0:
                #self.get_logger().info(f"Task completed: {response_data}")
                description = response_data.get("result", {}).get("description", "")
                self.get_logger().info(f"Task completed: {description}")
                self.publish_description(description)
                break
            elif response_data.get("result", {}).get("ret_code") == 1:
                self.get_logger().info("Task is still processing")
                time.sleep(2)  # 等待2秒后再次查询
            else:
                self.get_logger().error(f"Unexpected response: {response_data}")
                break


    def publish_description(self, description):
        self.get_logger().info(f'Publishing description: {description}')
        msg = String()
        msg.data = description
        self.publisher_tts_text.publish(msg)
        # response = requests.request("POST", url, headers=headers, data=payload)
        # self.get_logger().info('Received result from API')
        
        # print(response.text)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
