#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ImageStitchingNode(Node):
    def __init__(self):
        super().__init__('image_stitching_node')
        self.bridge = CvBridge()
        self.images = [None] * 4  # Initialize list to store images from 4 cameras
        self.camera_topics = [
            '/overhead_camera/overhead_camera1/image_raw',
            '/overhead_camera/overhead_camera2/image_raw',
            '/overhead_camera/overhead_camera3/image_raw',
            '/overhead_camera/overhead_camera4/image_raw'
        ]
        self.subscribers = []

        for i, topic in enumerate(self.camera_topics):
            self.subscribers.append(self.create_subscription(
                Image,
                topic,
                lambda msg, index=i: self.image_callback(msg, index),
                10
            ))

        self.stitched_image_pub = self.create_publisher(Image, '/stitched_image', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('ImageStitchingNode has been started.')

    def image_callback(self, msg, index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.images[index] = cv_image
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')

    def timer_callback(self):
        if all(image is not None for image in self.images):
            try:
                self.stitch_images()
            except Exception as e:
                self.get_logger().error(f'Error in timer_callback: {e}')

    def combine_images(self, images):
        resized_images = [cv2.resize(img, (320, 240)) for img in images]
        top_row = np.hstack((resized_images[3], resized_images[2]))
        bottom_row = np.hstack((resized_images[1], resized_images[0]))
        combined_image = np.vstack((top_row, bottom_row))
        return combined_image

    def stitch_images(self):
        combined_image = self.combine_images(self.images)

        try:
            # Save the stitched image to a file
            output_path = os.path.expanduser('~/stitched_image.jpg')
            cv2.imwrite(output_path, combined_image)
            self.get_logger().info(f'Stitched image saved to {output_path}')

            # Publish the stitched image
            stitched_msg = self.bridge.cv2_to_imgmsg(combined_image, 'bgr8')
            self.stitched_image_pub.publish(stitched_msg)
            self.get_logger().info('Image stitching succeeded.')
        except Exception as e:
            self.get_logger().error(f'Error in stitch_images: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_stitching_node = ImageStitchingNode()
    rclpy.spin(image_stitching_node)
    image_stitching_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
