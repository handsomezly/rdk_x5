import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import time
import dashscope
from dashscope.audio.tts import SpeechSynthesizer

dashscope.api_key = 'sk-069b539b6498491cac6830aa1b06903d'

class PygameNode(Node):
    def __init__(self):
        super().__init__('pygame_node')
        
        # 初始化Pygame混音器
        pygame.mixer.init()
        
        # 打印节点启动信息
        self.get_logger().info('Pygame Node has been started.')
        
        # 订阅tts_text话题
        self.subscription = self.create_subscription(
            String,
            'tts_text',
            self.tts_text_callback,
            10
        )

    def tts_text_callback(self, msg):
        alert_text = msg.data
        if alert_text.lower() == "none":
            self.stop_voice_alert()
        elif alert_text.lower() == "hckz":
            pygame.mixer.music.load('zjl-hckz.mp3')

            # 设置音量（范围从 0.0 到 1.0）
            pygame.mixer.music.set_volume(1.0)  # 将音量设置为 50%

            # 播放音频
            pygame.mixer.music.play()

        else:
            self.voice_alert(alert_text)

    def voice_alert(self, alert_text):
        # 语音合成并播放警告
        result = SpeechSynthesizer.call(model='sambert-zhishu-v1',
                                        text=alert_text,
                                        sample_rate=48000,
                                        volume=100,
                                        format='wav')
        if result.get_audio_data() is not None:
            with open('out.wav', 'wb') as f:
                f.write(result.get_audio_data())
            print('  get response: %s' % (result.get_response()))
            pygame.mixer.music.load('out.wav')
            pygame.mixer.music.set_volume(1)
            pygame.mixer.music.play()
            self.get_logger().info(f'Playing voice alert: {alert_text}')

    def stop_voice_alert(self):
        pygame.mixer.music.stop()
        self.get_logger().info('Voice alert stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = PygameNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
