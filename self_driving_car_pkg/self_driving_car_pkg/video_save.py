#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class VisionNode(Node):

    def __init__(self):
        super().__init__('video_save_node')
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_data, 10)
        self.out = cv2.VideoWriter('/home/piyush/bleh/data/output/vid_save.avi', cv2.VideoWriter_fourcc('M','J','P','G'),30, (1280,720))
        self.get_logger().info('Subscribing Video Feed && Saving')
        self.bridge = CvBridge()

    def process_data(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.out.write(frame)
        resized_frame = cv2.resize(frame, (512, 512))
        cv2.imshow('Frame', resized_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    vid_feed = VisionNode()
    rclpy.spin(vid_feed)

    vid_feed.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()