#!/usr/bin/env python3
from numpy.lib.twodim_base import triu_indices_from
import rclpy
from rclpy import publisher
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry

import math
import numpy as np

sqrt_2 = 1.41421356237
r = 0.25 # wheel_radius
d = 0.45 * sqrt_2 # base_diameter
turn_speed = 10

def trunc(values, decs=2):
    return np.trunc(values*10**decs)/(10**decs)
class Controller(Node):
    def __init__(self):
        super().__init__('omnibase')
        self.publisher_ = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        timer_period = 0.1  # seconds
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
        self.yawn = euler_from_quaternion(q)[2]  # Robot yawn angle (radian)
        
    def teleop_callback(self, data:Float32MultiArray): # Callback function when got teleop command
        [v_x, v_y, zeta] = data.data # v_x = [-1, 1], v_y = [-1, 1], zeta = [-3.14, 3.14]
        
        # v = v_x*50
        # vn = v_y*50
        
        v = (-v_x*math.cos(self.yawn+math.pi/4) - v_y*math.sin(self.yawn+math.pi/4))*50
        vn = (-v_y*math.cos(self.yawn+math.pi/4) + v_x*math.sin(self.yawn+math.pi/4))*50

        # print(trunc(zeta), trunc(self.yawn))
        omega = (zeta - self.yawn)
        if omega > math.pi: omega -= 2*math.pi
        if omega < -math.pi: omega += 2*math.pi
        # print(trunc(omega))
        omega = -omega
        if abs(omega) < 0.1: omega = 0
        if omega > 0 and omega < turn_speed: omega = turn_speed
        if omega < 0 and omega > -turn_speed: omega = -turn_speed

        [v_fr, v_fl, v_bl, v_br] = np.dot(np.array([[0, 1, d], [-1, 0, d], [0, -1, d], [1, 0, d]]), np.array([v, vn, omega]))
        self.array.data = [v*r for v in [v_fl, v_fr, v_bl, v_br]]
        # print([round(v, 2) for v in self.array.data])
    def timer_callback(self):
        # self.array.data = [1.0, 0.0, 1.0, 0.0]
        # print(self.array.data)
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