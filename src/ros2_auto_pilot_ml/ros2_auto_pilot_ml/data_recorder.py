import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import csv
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')
        
        # Subscribers for /scan and /cmd_vel topics
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Initialize storage for current scan and cmd_vel data
        self.current_scan = []
        self.current_cmd_vel = None
        
        # Create output CSV file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = f'data_{timestamp}.csv'
        
        # CSV Header
        self.header_written = False
        self.csv_header = ['scan_' + str(i) for i in range(360)] + ['linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z']
        
        # Open CSV file for writing
        self.cwd = os.getcwd()
        self.training_data_dir = os.path.join(self.cwd, 'training_data')
        # Ensure the directory exists
        os.makedirs(self.training_data_dir, exist_ok=True)
    
        self.csv_path = os.path.join(self.training_data_dir, self.csv_file)
        self.csv_fp = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_fp)

        self.get_logger().info(f'Data will be saved to {self.csv_path}')
    
    def scan_callback(self, msg: LaserScan):
        # Store the scan ranges (360-degree laser data)
        self.current_scan = list(msg.ranges)

    def cmd_vel_callback(self, msg: Twist):
        # Store the cmd_vel data
        self.current_cmd_vel = [
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ]
        
        # Record data when both scan and cmd_vel are available
        if self.current_scan and self.current_cmd_vel:
            # Write the header once
            if not self.header_written:
                self.csv_writer.writerow(self.csv_header)
                self.header_written = True
            
            # Combine scan data with cmd_vel data
            row = self.current_scan + self.current_cmd_vel
            self.csv_writer.writerow(row)
            self.get_logger().info('Data recorded')
    
    def destroy_node(self):
        # Close the CSV file
        self.csv_fp.close()
        self.get_logger().info(f'CSV file saved at: {self.csv_path}')
        super().destroy_node()


def main():
    rclpy.init()

    try:
        recorder = DataRecorder()
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Data Recording Stopped')
    finally:
        if recorder is not None:
            recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
