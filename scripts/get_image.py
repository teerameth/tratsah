#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Instantiate CvBridge
bridge = CvBridge()

class GetImage(Node):
    def __init__(self):
        super().__init__('get_image')     # Create Node
        # Create the subscriber (recieve odometry)
        self.subscription_odom = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.cap_callback,
            10)
    def cap_callback(self, msg):
        try:
            img = bridge.imgmsg_to_cv2(msg, "bgr8")
            # print(img.shape)
            cv2.imshow("A", img)
            cv2.waitKey(1)
        except:
            print("Error")
        else:
            pass
def main():
    rclpy.init()
    node = GetImage()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()