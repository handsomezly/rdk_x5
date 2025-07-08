import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import time
import dashscope
from dashscope.audio.tts_v2 import *

# 配置DashScope API
dashscope.api_key = 'sk-5e8d25a48f8d4a2094c76d9bc660dd24'

class HybridTTSNode(Node):
    def __init__(self):
        super().__init__('hybrid_tts_node')
        
        # 初始化Pygame混音器
        pygame.mixer.init()
        
        # 创建发布器 - 用于发送消息给hobot_tts
        self.hobot_tts_publisher = self.create_publisher(String, '/tts_text', 10)
        
        # 订阅tts_text话题
        self.subscription = self.create_subscription(
            String,
            'tts_request',  # 新的话题名，区分不同的TTS请求
            self.tts_request_callback,
            10
        )
        
        # TTS选择策略配置
        self.tts_strategy = {
            # 紧急/实时场景 - 使用hobot_tts
            'emergency': 'hobot_tts',
            'status': 'hobot_tts',
            'warning': 'hobot_tts',
            'quick': 'hobot_tts',
            
            # 对话/娱乐场景 - 使用API TTS
            'conversation': 'api_tts',
            'story': 'api_tts',
            'music': 'api_tts',
            'default': 'api_tts'
        }
        
        # 预设音频文件映射
        self.preset_audio_map = {
            'hckz': 'zjl-hckz.mp3',
            'qt': 'zhl-qt.mp3',
            'qhc': 'zhl-qhc.mp3'
        }
        
        self.get_logger().info('Hybrid TTS Node has been started.')

    def tts_request_callback(self, msg):
        """
        处理TTS请求
        消息格式: "type:content" 或 "content"
        例如: "emergency:系统故障" 或 "你好，我是机器人"
        """
        request_data = msg.data
        
        # 解析请求类型和内容
        if ':' in request_data:
            request_type, content = request_data.split(':', 1)
        else:
            request_type = 'default'
            content = request_data
        
        # 处理停止命令
        if content.lower() == "none":
            self.stop_voice_alert()
            return
        
        # 处理预设音频
        if self.is_preset_audio(content):
            self.play_preset_audio(content)
            return
        
        # 根据类型选择TTS方式
        tts_method = self.tts_strategy.get(request_type, 'api_tts')
        
        if tts_method == 'hobot_tts':
            self.use_hobot_tts(content)
        else:
            self.use_api_tts(content)

    def use_hobot_tts(self, content):
        """使用hobot_tts进行语音合成"""
        try:
            # 发布消息到hobot_tts节点
            tts_msg = String()
            tts_msg.data = content
            self.hobot_tts_publisher.publish(tts_msg)
            
            self.get_logger().info(f'使用hobot_tts合成: {content}')
            
        except Exception as e:
            self.get_logger().error(f'hobot_tts合成失败: {e}')
            # 失败时回退到API TTS
            self.use_api_tts(content)

    def use_api_tts(self, content):
        """使用API TTS进行语音合成"""
        try:
            # 使用DashScope API进行语音合成
            result = TextToSpeechV2.call(
                model='cosyvoice-v1',
                voice='cosyvoice-v1-lotus',     
                text=content,
                sample_rate=48000,
                volume=100,
                format='wav'
            )
            
            if result.get_audio_data() is not None:
                with open('api_tts_output.wav', 'wb') as f:
                    f.write(result.get_audio_data())
                
                # 播放音频
                pygame.mixer.music.load('api_tts_output.wav')
                pygame.mixer.music.set_volume(1.0)
                pygame.mixer.music.play()
                
                self.get_logger().info(f'使用API TTS合成播放: {content}')
            else:
                self.get_logger().error('API TTS返回数据为空')
                
        except Exception as e:
            self.get_logger().error(f'API TTS合成失败: {e}')

    def is_preset_audio(self, content):
        """检查是否是预设音频"""
        return content.lower() in self.preset_audio_map

    def play_preset_audio(self, content):
        """播放预设音频"""
        try:
            audio_file = self.preset_audio_map[content.lower()]
            pygame.mixer.music.load(audio_file)
            pygame.mixer.music.set_volume(1.0)
            pygame.mixer.music.play()
            
            self.get_logger().info(f'播放预设音频: {audio_file}')
            
        except Exception as e:
            self.get_logger().error(f'预设音频播放失败: {e}')

    def stop_voice_alert(self):
        """停止语音播放"""
        pygame.mixer.music.stop()
        self.get_logger().info('语音播放已停止')

def main(args=None):
    rclpy.init(args=args)
    node = HybridTTSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 