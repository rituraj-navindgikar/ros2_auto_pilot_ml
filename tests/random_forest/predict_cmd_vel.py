#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import joblib

import signal

class PredictCmdVelNode(Node):
    def __init__(self):
        super().__init__('predict_cmd_vel')
        self.model = joblib.load("mocel_maze.pkl")
        self.get_logger().info("Model loaded")

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, msg):
        scan_data = np.array(msg.ranges)
        scan_data = np.nan_to_num(scan_data, nan=0.0, posinf=0.0, neginf=0.0)
        scan_data = scan_data.reshape(1, -1)  # Reshape for the model (1 row, 360 columns)

        # Predict the action (label)
        predicted_label = self.model.predict(scan_data)[0]
        self.get_logger().info(f"Predicted label: {predicted_label}")

        linear_x, angular_z = map(float, predicted_label.split('_'))

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(f"Published cmd_vel: linear_x={linear_x}, angular_z={angular_z}")


def main(args=None):
    rclpy.init(args=args)

    node = PredictCmdVelNode()

    def handle_shutdown(signum, frame):
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        exit(0)

    # Register the signal handler
    signal.signal(signal.SIGINT, handle_shutdown)  # Handle Ctrl + C

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Unexpected exception: {e}")



if __name__ == '__main__':
    main()
