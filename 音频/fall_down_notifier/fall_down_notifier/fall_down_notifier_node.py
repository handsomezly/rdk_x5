# import rclpy
# from rclpy.node import Node
# from ai_msgs.msg import PerceptionTargets
# import smtplib
# from email.mime.multipart import MIMEMultipart
# from email.mime.text import MIMEText

# class FallDownNotifier(Node):

#     def __init__(self):
#         super().__init__('fall_down_notifier')
#         self.subscription = self.create_subscription(
#             PerceptionTargets,
#             '/hobot_falldown_detection',  # 订阅的话题名称
#             self.listener_callback,
#             10
#         )
#         self.subscription  

#     def listener_callback(self, msg):
#         # 遍历接收到的目标
#         for target in msg.targets:
#             # 遍历目标的属性
#             for attribute in target.attributes:
#                 # 检查属性类型是否为 'falldown'
#                 if attribute.type == 'falldown':
#                     if attribute.value == 1.0:
#                         # 检测到摔倒
#                         message = f"Track ID {target.track_id} is detected as fall down!"
#                         self.get_logger().info(message)
#                         self.send_email(message)
#                     # else:
#                     #     # 检测到没有摔倒
#                     #     self.get_logger().info(f"Track ID {target.track_id} is detected as not fall down!")

#     def send_email(self, message):
#         # 邮件设置
#         my_sender = '2150942638@qq.com'   
#         my_pass = 'mvtgqhildopgdjgf'  
#         my_user = '3117098921@qq.com'  

#         msg = MIMEMultipart()
#         msg['From'] = my_sender
#         msg['To'] = my_user
#         msg['Subject'] = 'Fall Down Detection Alert'
#         msg.attach(MIMEText(message, 'plain'))

#     #try:
#         # 连接到SMTP服务器并发送邮件
#         server = smtplib.SMTP_SSL("smtp.qq.com", 465)  # 发件人邮箱中的SMTP服务器
#         server.login(my_sender, my_pass)
#         server.sendmail(my_sender, my_user, msg.as_string())
#         server.quit()
#         #self.get_logger().info("Email sent successfully!")
#         # except Exception as e:
#         #     self.get_logger().error(f"Failed to send email: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     fall_down_notifier = FallDownNotifier()
#     rclpy.spin(fall_down_notifier)
#     fall_down_notifier.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

class FallDownNotifier(Node):

    def __init__(self):
        super().__init__('fall_down_notifier')
        self.subscription = self.create_subscription(
            String,
            'tts_text',  # 订阅的话题名称
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        if msg.data == "警报，有人摔倒了":
            message = "检测到有人摔倒！"
            self.get_logger().info(message)
            self.send_email(message)

    def send_email(self, message):
        # 邮件设置
        my_sender = '3172371842@qq.com'   
        my_pass = 'ykbgbktmhweldgia'  
        my_user = '2720488340@qq.com'  

        msg = MIMEMultipart()
        msg['From'] = my_sender
        msg['To'] = my_user
        msg['Subject'] = '摔倒检测警报'
        msg.attach(MIMEText(message, 'plain'))

        try:
            # 连接到SMTP服务器并发送邮件
            server = smtplib.SMTP_SSL("smtp.qq.com", 465)  # 发件人邮箱中的SMTP服务器
            server.login(my_sender, my_pass)
            server.sendmail(my_sender, my_user, msg.as_string())
            server.quit()
            self.get_logger().info("邮件发送成功！")
        except Exception as e:
            self.get_logger().error(f"发送邮件失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    fall_down_notifier = FallDownNotifier()
    rclpy.spin(fall_down_notifier)
    fall_down_notifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
