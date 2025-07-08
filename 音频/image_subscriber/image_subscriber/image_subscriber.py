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
        self.subscription_image = self.create_subscription(
            CompressedImage,
            '/image',
            self.listener_callback_image,
            10)
        self.subscription_cmd = self.create_subscription(
            String,
            '/cmd_ns',
            self.listener_callback_cmd,
            10)
        self.publisher_tts_text = self.create_publisher(String, 'tts_text', 10)
        self.subscription_image  # prevent unused variable warning
        self.subscription_cmd  # prevent unused variable warning
        self.send_image = False
        self.task_id = None  # 初始化 task_id
        self.get_logger().info('ImageSubscriber node initialized')

    def listener_callback_image(self, msg):
        self.get_logger().info('Received image message')
        if self.send_image:
            self.get_logger().info('Sending video frame to API')
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # 将图像数据编码为Base64并发送给API
            image_base64 = base64.b64encode(np_arr).decode('utf-8')
            self.send_image_to_api(image_base64)
            self.send_image = False  # 重置标志位，等待下一个 "图像信息" 命令

    def listener_callback_cmd(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        if msg.data == "image_ud":  
            self.get_logger().info('Received command to send image')
            self.send_image = True

    def send_image_to_api(self, image_base64):
        self.get_logger().info('Sending image to API')
        url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/image-understanding/request?access_token=25.ccd4833ee682b91fc6dd1d99021c5040.315360000.2066887825.282335-119410901"

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

        url = f"https://aip.baidubce.com/rest/2.0/image-classify/v1/image-understanding/get-result?access_token=25.d0ac079fe5526426ed3c4999f55cf06c.315360000.2066887981.282335-119410924&task_id={self.task_id}"

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

