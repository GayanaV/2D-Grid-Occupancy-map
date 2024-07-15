#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageListener(Node):

    def __init__(self):
        super().__init__('image_listener')
        self.bridge = CvBridge()

        # Dictionary to store camera subscriptions and their corresponding IDs
        self.camera_subscriptions = {
            '/overhead_camera/overhead_camera1/image_raw': 1,
            '/overhead_camera/overhead_camera2/image_raw': 2,
            '/overhead_camera/overhead_camera3/image_raw': 3,
            '/overhead_camera/overhead_camera4/image_raw': 4
        }

        # List to store active subscriptions
        self.image_subscriptions = []

        # Create subscriptions for each camera topic
        for topic, camera_id in self.camera_subscriptions.items():
            self.image_subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, cam_id=camera_id: self.listener_callback(msg, cam_id),
                    10
                )
            )

    def listener_callback(self, msg, camera_id):
        self.get_logger().info(f'Receiving image from camera {camera_id}')
        
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Display the image (you can customize this part)
        cv2.imshow(f"Camera {camera_id} Image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

   

