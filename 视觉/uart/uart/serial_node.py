#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serial import Serial
from geometry_msgs.msg import Twist

class SerialCommunicationNode(Node):
    def __init__(self):
        super().__init__('serial_communication_node')
        self.uart_dev = '/dev/ttyUSB0'  # 串口设备
        self.baudrate = 115200  # 波特率
        self.serial_connection = None  # 串口连接对象
        
        # 创建一个订阅者，用于订阅'serial_input'话题上的消息
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        
        # 创建第二个订阅者，用于订阅'serial_input2'话题上的消息
        self.subscription2 = self.create_subscription(
            String,  # 假设第二个话题发送的是String类型的消息
            'cmd_ns',
            self.listener_callback2,  # 使用相同的回调函数
            10
        )
    
    def listener_callback(self, msg):
        # 回调函数，处理从ROS话题订阅到的输入
        try:
            # 尝试建立串口连接
            if not self.serial_connection:
                self.serial_connection = Serial(self.uart_dev, self.baudrate, timeout=1)

            serial_data = f"{msg.linear.x},{msg.linear.y},{msg.angular.z}\n".encode('UTF-8')
            
            print(serial_data)
            # 发送数据到串口
            self.serial_connection.write(serial_data)

            # 读取串口返回的数据
            #received_data = self.serial_connection.read(len(msg.data)).decode('UTF-8')

            # 将接收到的数据发布到ROS话题
            #self.publisher_.publish(String(data=received_data))
        except Exception as e:
            self.get_logger().warn('Serial communication error: %s' % e)
    
    def listener_callback2(self, msg):
        # 回调函数，处理从ROS话题订阅到的输入
        serial_data = bytes.fromhex("AA5AA06400000096969696FFEECCDD55662255")
        try:
            # 尝试建立串口连接
            if not self.serial_connection:
                self.serial_connection = Serial(self.uart_dev, self.baudrate, timeout=1)

            # 直接发送String消息内容，不需要编码
            #serial_data = ("%s\n" % msg.data).encode('UTF-8')
            # 发送数据到串口
            self.serial_connection.write(serial_data)
            
            # 读取串口返回的数据
            #received_data = self.serial_connection.read(len(msg.data)).decode('UTF-8')
            
            # 将接收到的数据发布到ROS话题
            #self.publisher_.publish(String(data=received_data))
        except Exception as e:
            self.get_logger().warn('Serial communication error: %s' % e)
    def close_serial(self):
        # 关闭串口连接
        if self.serial_connection:
            self.serial_connection.close()
            self.serial_connection = None

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialCommunicationNode()
    
    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass
    finally:
        serial_node.close_serial()
        serial_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
