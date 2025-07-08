import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String  # 添加String类型的消息类型
import serial
import sys

class SerialToRos2Publisher(Node):
    def __init__(self):
        super().__init__('serial_to_ros2_publisher')
        self.publisher_nucleus_ = self.create_publisher(Float32, 'nucleus', 10)
        self.publisher_cmd_ns_ = self.create_publisher(String, 'cmd_ns', 10)  # 创建cmd_ns话题的发布者
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.value = 0.0
        self.missing_data_counter = 0
        self.max_missing_data_threshold = 5  # 允许的最大未接收数据次数

        try:
            self.uart_dev = "/dev/ttyUSB0"
            self.baudrate = 115200
            # 设置timeout为0，非阻塞模式
            self.serial_conn = serial.Serial(self.uart_dev, self.baudrate, timeout=0)
        except serial.SerialException as e:
            self.get_logger().error(f"无法打开串口: {e}")
            sys.exit(1)

    def timer_callback(self):
        try:
            received_data = self.serial_conn.read(self.serial_conn.inWaiting())
            if received_data:
                try:
                    # 将数据解码为字符串并转换为浮点数，然后格式化为两位小数
                    self.value = round(float(received_data.decode('UTF-8').strip()), 2)
                    self.missing_data_counter = 0  # 重置未接收数据计数器

                    # 发布nucleus话题
                    nucleus_message = Float32()
                    nucleus_message.data = self.value
                    self.publisher_nucleus_.publish(nucleus_message)
                    self.get_logger().info(f'Published to nucleus: {nucleus_message.data:.2f}')

                    # 如果值大于等于0.3，发布cmd_ns话题
                    if self.value >= 0.3:
                        cmd_ns_message = String()
                        cmd_ns_message.data = 'yfs'
                        self.publisher_cmd_ns_.publish(cmd_ns_message)
                        self.get_logger().info('Published to cmd_ns: yfs')

                except ValueError:
                    self.get_logger().warn("收到无法解析的数据")
            else:
                self.missing_data_counter += 1
                if self.missing_data_counter >= self.max_missing_data_threshold:
                    self.value = 0.0

                    # 发布nucleus话题（值为0.0）
                    nucleus_message = Float32()
                    nucleus_message.data = self.value
                    self.publisher_nucleus_.publish(nucleus_message)
                    self.get_logger().info(f'Published to nucleus: {nucleus_message.data:.2f}')

        except serial.SerialException as e:
            self.get_logger().error(f"串口异常: {e}")
        except Exception as e:
            self.get_logger().error(f"异常: {e}")

    def destroy_node(self):
        self.serial_conn.close()  # 关闭串口连接
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    serial_to_ros2_publisher = SerialToRos2Publisher()
    rclpy.spin(serial_to_ros2_publisher)
    serial_to_ros2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
