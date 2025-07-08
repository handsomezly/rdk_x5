import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import time
import os
import dashscope
from dashscope.audio.tts import SpeechSynthesizer


dashscope.api_key = 'sk-5e8d25a48f8d4a2094c76d9bc660dd24'

class PygameNode(Node):
    def __init__(self):
        super().__init__('pygame_node')
        
        # 获取当前脚本所在目录
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 初始化Pygame混音器
        pygame.mixer.init()
        
        # 打印节点启动信息
        self.get_logger().info('Pygame Node has been started.')
        self.get_logger().info(f'Script directory: {self.script_dir}')
        
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
            try:
                music_path = os.path.join(self.script_dir, 'zjl-hckz.mp3')
                self.get_logger().info(f'Loading music: {music_path}')
                pygame.mixer.music.load(music_path)
                pygame.mixer.music.set_volume(1.0)
                pygame.mixer.music.play()
                self.get_logger().info('Successfully playing hckz music')
            except Exception as e:
                self.get_logger().error(f'Error playing hckz audio: {e}')
        elif alert_text.lower() == "qt":
            try:
                music_path = os.path.join(self.script_dir, 'zhl-qt.mp3')
                self.get_logger().info(f'Loading music: {music_path}')
                pygame.mixer.music.load(music_path)
                pygame.mixer.music.set_volume(1.0)
                pygame.mixer.music.play()
                self.get_logger().info('Successfully playing qt music')
            except Exception as e:
                self.get_logger().error(f'Error playing qt audio: {e}')
        elif alert_text.lower() == "qhc":
            try:
                music_path = os.path.join(self.script_dir, 'zhl-qhc.mp3')
                self.get_logger().info(f'Loading music: {music_path}')
                pygame.mixer.music.load(music_path)
                pygame.mixer.music.set_volume(1.0)
                pygame.mixer.music.play()
                self.get_logger().info('Successfully playing qhc music')
            except Exception as e:
                self.get_logger().error(f'Error playing qhc audio: {e}')
        else:
            self.voice_alert(alert_text)

    def voice_alert(self, alert_text):
        try:
            # 使用正确的阿里云TTS API调用方式
            result = SpeechSynthesizer.call(
                model='sambert-zhichu-v1',  # 使用免费的中文语音模型
                text=alert_text,
                format='wav'
            )
            
            if result.get_audio_data() is not None:
                output_path = os.path.join(self.script_dir, 'out.wav')
                with open(output_path, 'wb') as f:
                    f.write(result.get_audio_data())
                self.get_logger().info(f'TTS synthesis successful for: {alert_text}')
                
                pygame.mixer.music.load(output_path)
                pygame.mixer.music.set_volume(1.0)
                pygame.mixer.music.play()
                self.get_logger().info(f'Playing voice alert: {alert_text}')
            else:
                self.get_logger().error('Failed to generate TTS audio: No audio data')
                
        except Exception as e:
            self.get_logger().error(f'TTS synthesis failed: {e}')
            # 如果TTS失败，至少记录文本
            self.get_logger().info(f'TTS text (audio failed): {alert_text}')

    def stop_voice_alert(self):
        try:
            pygame.mixer.music.stop()
            self.get_logger().info('Voice alert stopped.')
        except Exception as e:
            self.get_logger().error(f'Error stopping voice alert: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PygameNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
