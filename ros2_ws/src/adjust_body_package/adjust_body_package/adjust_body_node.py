#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_msgs.msg import Point
import serial
import time

class CombinedNode(Node):

    def __init__(self):
        super().__init__('adjust_node')

        # Initialize the serial port
        self.serial_port = serial.Serial('/dev/ttyARDUINOuno', 1000000)
        
        # Create publishers and subscribers
        self.center_subscription = self.create_subscription(
            Point,
            '/center_coordinates',
            self.center_listener_callback,
            1)
        self.adjust_pub = self.create_publisher(Twist, '/cmd_vel_adjust', 1)
        # self.stop_pub = self.create_publisher(Twist, '/cmd_vel_stop', 1)

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 2.0)
        self.declare_parameter("forward_chase_multiplier", 0.4)
        self.declare_parameter("max_size_thresh", 200.0)
        self.declare_parameter("filter_value", 0.9)
        
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_multiplier = self.get_parameter('forward_chase_multiplier').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value

        # Initialize timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize variables used in callbacks
        self.target_x = 0.0
        self.target_y = 0.0
        self.last_received_time = time.time() - 10000


    def timer_callback(self):
        msg = Twist()
        if (time.time()- self.last_received_time < self.rcv_timeout_secs):
            if (self.target_y < self.max_size_thresh):
                msg.linear.x = -self.forward_chase_multiplier * self.target_y
            msg.angular.z = -self.angular_chase_multiplier * self.target_x
        else:
            self.get_logger().info('Target lost')
            msg.linear.x = 0.3
        self.adjust_pub.publish(msg)

    def center_listener_callback(self, msg):
        self.process_center_coordinates(msg.x, msg.y)
        self.check_and_send_serial_command(msg.x, msg.y)
        self.last_received_time = time.time()

    def process_center_coordinates(self, x, y):
        f = self.filter_value

        # Assuming the target pixel variables are equal to the ones used previously
        target_x_pixel = 342
        normalized_x = (x / target_x_pixel) - 1
        self.target_x = self.target_x * f + normalized_x * (1 - f)

        target_y_pixel = 210
        normalized_y = (y / target_y_pixel) - 1
        self.target_y = self.target_y * f + normalized_y * (1 - f)

        self.get_logger().info(f'Message Received: x={x}, y={y}')
        self.get_logger().info(f'Processing: x={self.target_x}, y={self.target_y}')

    def check_and_send_serial_command(self, x, y):
        if 312 <= x <= 362 and 212 <= y <= 252:
            self.get_logger().info('Sending \'r\' to serial port')
            self.serial_port.write('r'.encode())
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.adjust_pub.publish(msg)

            time.sleep(20)  # Wait for 20 seconds

def main(args=None):
    rclpy.init(args=args)
    adjust_node = CombinedNode()
    rclpy.spin(adjust_node)
    adjust_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
