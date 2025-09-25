#!/usr/bin/env python3
# record_imu_to_csv.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import argparse
import os

class ImuRecorder(Node):
    def __init__(self, topic, out_file):
        super().__init__('imu_recorder')
        self.sub = self.create_subscription(Imu, topic, self.cb, 10)
        self.out_file = out_file
        self.csvfile = open(self.out_file, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        # header: stamp_sec,stamp_nsec,ax,ay,az,wx,wy,wz,orient_w,orient_x,orient_y,orient_z
        self.writer.writerow(['sec','nsec','ax','ay','az','wx','wy','wz',
                              'oq_w','oq_x','oq_y','oq_z'])
        self.get_logger().info(f'Writing IMU messages on "{topic}" to {out_file}')

    def cb(self, msg: Imu):
        s = msg.header.stamp
        o = msg.orientation
        self.writer.writerow([
            s.sec, s.nanosec,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            o.w, o.x, o.y, o.z
        ])

    def destroy_node(self):
        try:
            self.csvfile.close()
        finally:
            super().destroy_node()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/rslidar_imu_data', help='IMU topic')
    parser.add_argument('--out', default='imu_stationary.csv', help='output CSV file')
    args = parser.parse_args()

    rclpy.init()
    node = ImuRecorder(args.topic, args.out)
    try:
        # Play your bag in another terminal:
        # ros2 bag play mybag.db3 --clock
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
