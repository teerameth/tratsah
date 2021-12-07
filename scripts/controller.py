#!/usr/bin/env python3
import rclpy
from rclpy import publisher
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import Odometry

class Controller(Node):
    def __init__(self):
        super().__init__('omnibase')
        self.publisher_ = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Create the subscriber (recieve odometry)
        self.subscription = self.create_subscription(
            Odometry, 
            'odom', 
            self.listener_callback, 
            10)
        self.subscription # prevent unused variable warning

        self.array = Float64MultiArray()
        print(type(self.array))
        self.array.data = [0.0, 0.0, 0.0, 0.0]  # Init as stop wheel
        self.publisher_.publish(self.array)


    def listener_callback(self, data:Odometry): # Callback function.
        # Display the message on the console
        print("Inside listener callback")
        print(data)
        # self.get_logger().info('Receiving drone driving instructions')
        # speed = data.data[0]
        # fb = data.data[1]
        # print(speed, fb)

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


if __name__ == '__main__':
    main()