#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from .Drive_Bot import Car


class VisionNode(Node):

    def __init__(self):
        super().__init__('video_save_node')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_data, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pub_vel)
        self.get_logger().info('Subscribing Video Feed && Saving')
        self.bridge = CvBridge()
        self.cmd_vel_msg = Twist()
        self.Car = Car()

    def pub_vel(self):
        self.publisher_.publish(self.cmd_vel_msg)

    def process_data(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        Angle,Speed = self.Car.drive_car(frame)

        self.cmd_vel_msg.angular.z = Angle
        self.cmd_vel_msg.linear.x = Speed

        frame_resized = cv2.resize(frame, (512, 512))
        # cv2.imshow('frame', frame_resized)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    vid_feed = VisionNode()
    rclpy.spin(vid_feed)

    vid_feed.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()