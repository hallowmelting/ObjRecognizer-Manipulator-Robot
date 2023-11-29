#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import Yolov8Inference
from custom_msgs.msg import Point  # Replace with the actual path to your Point message

class InferenceResultSubscriber(Node):

    def __init__(self):
        super().__init__('trash_center_node')
        self.inference_result_subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.listener_callback,
            10)
        self.center_coordinate_publisher = self.create_publisher(Point, '/center_coordinates', 1)

    def listener_callback(self, yolov8_inference):
        self.get_logger().info('Received Yolov8Inference')

        for inference_result in yolov8_inference.yolov8_inference:
            top = inference_result.top
            left = inference_result.left
            bottom = inference_result.bottom
            right = inference_result.right
            
            # Calculate the center x and y coordinates and convert them to integer
            x = int((top + bottom) / 2)
            y = int((left + right) / 2)
            
            # Log the center coordinates as integers
            self.get_logger().info(f'Center coordinates (int): x={x}, y={y}')
            
            # Publish the center coordinates as a Point message with integer values
            center_coordinates = Point(x=float(x), y=float(y))  # 'Point' message might expect float values
            self.center_coordinate_publisher.publish(center_coordinates)

def main(args=None):
    rclpy.init(args=args)
    inference_result_subscriber = InferenceResultSubscriber()
    rclpy.spin(inference_result_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()