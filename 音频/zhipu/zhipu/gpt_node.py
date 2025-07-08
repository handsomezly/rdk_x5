import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# 假设 zhipuai 模块和 ZhipuAI 类已经正确安装和定义
from zhipuai import ZhipuAI

# 创建 ZhipuAI 类的实例，这里需要提供有效的 API 密钥
client = ZhipuAI(api_key="caa10f7d79d24027a3b675cc90a39c65.ryf30kHFgxEIRyMq")

# 初始化rclpy
rclpy.init(args=[])

class ASRSubscriber(Node):
    def __init__(self):
        super().__init__('asr_subscriber_node')
        self.client = client
        # 创建一个订阅者，订阅 asr_pub_topic_name 话题
        self.subscription = self.create_subscription(
            String,
            'audio_asr',  # 这里替换为实际的话题名称
            self.callback,
            10
        )
         # 创建一个发布者，发布AI的回答结果到 ai_response_topic 话题
        self.publisher = self.create_publisher(
            String,
            'tts_text',  # AI回答结果发布的话题名称
            10
        )

        self.subscription  # 避免未命名的订阅者被创建
        self.logger = self.get_logger()

    def callback(self, msg):
        # 这里的回调函数会处理接收到的消息
        # 将订阅到的消息发送给 AI
        #print(msg.data)
        self.logger.info(f"User: {msg.data}")
        self.send_to_ai(msg.data)
        
    def send_to_ai(self, question):
        # 创建一个新的异步聊天任务
        response = self.client.chat.asyncCompletions.create(
            model="glm-3-turbo",
            messages=[
                {
                    "role": "user",
                    "content": question
                }
            ]
        )
        
        task_id = response.id
        task_status = ''
        get_cnt = 0
        
        # 循环直到获取到任务结果或者重试超过一定次数
        while task_status != 'SUCCESS' and task_status != 'FAILED' and get_cnt <= 40:
            time.sleep(2)  # 等待2秒再次轮询结果
            result_response = self.client.chat.asyncCompletions.retrieve_completion_result(id=task_id)
            task_status = result_response.task_status
            get_cnt += 1

        # 检查任务是否成功并打印结果
        if task_status == 'SUCCESS':
            if hasattr(result_response, 'choices') and result_response.choices:
                ai_response = result_response.choices[0].message.content
                #print("AI的回答是：")
                #print(ai_response)
                self.logger.info("AI's response: %s" % ai_response)
                # 将AI的回答发布到 ai_response_topic 话题
                self.publisher.publish(String(data=ai_response))
            else:
                self.logger.error("Response does not contain valid 'choices' attribute or it is empty.")
                #print("错误：响应中没有找到有效的 'choices' 属性或它为空。")
        else:
            self.logger.error("Failed to get a response, please try again later.")
            #print("获取回答失败，请稍后再试。")

def main():
    node = ASRSubscriber()
    try:
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
