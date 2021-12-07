#!/usr/bin/env python3
import rclpy
from rclpy import publisher
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry

import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('omnibase')
        self.publisher_ = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create the subscriber (recieve odometry)
        self.subscription_odom = self.create_subscription(
            Odometry, 
            'odom',
            self.odom_callback, 
            10)
        self.subscription_odom # prevent unused variable warning

        # Create the subscriber (recieve teleop command)
        self.subscription_teleop = self.create_subscription(
            Float32MultiArray, 
            'teleop_omni',
            self.teleop_callback, 
            10)

        # Declare class variable
        self.array = Float64MultiArray()
        self.array.data = [0.0, 0.0, 0.0, 0.0]      # Init as stop wheel
        self.publisher_.publish(self.array)         # Publish first wheel omega
        self.v_x, self.v_y, self.omega_z = 0, 0, 0  # Command values (from teleop, navigation)
        self.yawn = 0                               # Yawn from odom

    def odom_callback(self, data:Odometry): # Callback function when got odom
        q = data.pose.pose.orientation
        self.yawn = euler_from_quaternion(q)[2]*180  # Robot yawn angle
        
    def teleop_callback(self, data:Float32MultiArray): # Callback function when got teleop command
        print(data.data) # v_x = [-1, 1], v_y = [-1, 1], omega_z = [-3.14, 3.14]
        

    def timer_callback(self):
        self.array.data = [1.0, 0.0, 1.0, 0.0]
        self.publisher_.publish(self.array)
def main(args = None):
    rclpy.init(args=args)

    controller_node = Controller()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q
if __name__ == '__main__':
    main()