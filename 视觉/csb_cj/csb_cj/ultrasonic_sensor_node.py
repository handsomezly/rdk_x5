#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import Hobot.GPIO as GPIO
import time

TRIG = 16  # send-pin
ECHO = 18  # receive-pin

class UltrasonicSensorNode(Node):

    def __init__(self):
        super().__init__('ultrasonic_sensor_node')
        self.publisher_ = self.create_publisher(Int32, 'cs_dis', 10)
        self.setup()
        self.timer = self.create_timer(0.3, self.timer_callback)
        self.get_logger().info('Ultrasonic Sensor Node has been started.')

    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(TRIG, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(ECHO, GPIO.IN)

    def distance(self):
        GPIO.output(TRIG, 1)  # 给Trig一个10US以上的高电平
        time.sleep(0.00001)
        GPIO.output(TRIG, 0)

        # 等待低电平结束，然后记录时间
        while GPIO.input(ECHO) == 0:  # 捕捉 echo 端输出上升沿
            pass
        time1 = time.time()

        # 等待高电平结束，然后记录时间
        while GPIO.input(ECHO) == 1:  # 捕捉 echo 端输出下降沿
            pass
        time2 = time.time()

        during = time2 - time1
        # ECHO高电平时刻时间减去低电平时刻时间，所得时间为超声波传播时间
        distance_cm = int(during * 340 / 2 * 100)  # 将超声波数据转换成整型
        return distance_cm

    def timer_callback(self):
        dis = self.distance()
        msg = Int32()
        msg.data = dis
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published distance: {dis} cm')

    def destroy_node(self):
        super().destroy_node()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

