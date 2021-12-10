#!/usr/bin/env python3
import pygame, math
import sys

from rclpy import publisher
from rclpy.node import Node

import geometry_msgs.msg
import rclpy

class JoystickPub(Node):
    def __init__(self):
        super().__init__('teleop_twist_joystick')     # Create Node
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)  # Setup publisher
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)   # Setup timer callback

        # Init PyGame
        pygame.init()
        size = [500, 700]
        self.screen = pygame.display.set_mode(size)
        pygame.display.set_caption("teleop_twist_joystick")
        done = False                # Loop until the user clicks the close button.
        clock = pygame.time.Clock() # Used to manage how fast the screen updates
        pygame.joystick.init()      # Initialize the joysticks
        self.textPrint = TextPrint()     # Get ready to print
        self.omega_z = 0.0
    def timer_callback(self):   # Joystick timer callback
        ### Data ###
        axes_value = []
        ### Get button from PyGame ###
        # Event processing step
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
    
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN
            # JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")
        # Drawing step
        # First, clear the screen to white. Don't put other drawing commands
        # above this, or they will be erased with this command.
        self.screen.fill(WHITE)
        self.textPrint.reset()
    
        # Get count of joysticks
        joystick_count = pygame.joystick.get_count()
    
        self.textPrint.print(self.screen, "Number of joysticks: {}".format(joystick_count))
        self.textPrint.indent()
        # For each joystick:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.textPrint.print(self.screen, "Joystick {}".format(i))
            self.textPrint.indent()
            # Get the name from the OS for the controller/joystick
            name = joystick.get_name()
            self.textPrint.print(self.screen, "Joystick name: {}".format(name))
            # Usually axis run in pairs, up/down for one, and left/right for
            # the other.
            axes = joystick.get_numaxes()
            self.textPrint.print(self.screen, "Number of axes: {}".format(axes))
            self.textPrint.indent()
            for i in range(axes):
                axis = joystick.get_axis(i)
                axes_value.append(axis)
                self.textPrint.print(self.screen, "Axis {} value: {:>6.3f}".format(i, axis))
            self.textPrint.unindent()
            buttons = joystick.get_numbuttons()
            self.textPrint.print(self.screen, "Number of buttons: {}".format(buttons))
            self.textPrint.indent()
            for i in range(buttons):
                button = joystick.get_button(i)
                self.textPrint.print(self.screen, "Button {:>2} value: {}".format(i, button))
            self.textPrint.unindent()
            # Hat switch. All or nothing for direction, not like joysticks.
            # Value comes back in an array.
            hats = joystick.get_numhats()
            self.textPrint.print(self.screen, "Number of hats: {}".format(hats))
            self.textPrint.indent()
            for i in range(hats):
                hat = joystick.get_hat(i)
                self.textPrint.print(self.screen, "Hat {} value: {}".format(i, str(hat)))
            self.textPrint.unindent()
            self.textPrint.unindent()
        # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()
        ### Prepare data & send ###
        self.omega_z = -axes_value[3]
        # self.array.data = [axes_value[0], -axes_value[1], self.omega_z]  # [v_x, v_y, omega_z]
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = -axes_value[1]
        twist.linear.y = -axes_value[0]
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.omega_z
        self.publisher_.publish(twist)
def main(args = None):
    rclpy.init(args=args)
    joystick_node = JoystickPub()
    rclpy.spin(joystick_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_node.destroy_node()
    rclpy.shutdown()
    # Close the window and quit.
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()
# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
class TextPrint(object):
    """
    This is a simple class that will help us print to the screen
    It has nothing to do with the joysticks, just outputting the
    information.
    """
    def __init__(self):
        """ Constructor """
        self.reset()
        self.x_pos = 10
        self.y_pos = 10
        self.font = pygame.font.Font(None, 20)
 
    def print(self, my_screen, text_string):
        """ Draw text onto the screen. """
        text_bitmap = self.font.render(text_string, True, BLACK)
        my_screen.blit(text_bitmap, [self.x_pos, self.y_pos])
        self.y_pos += self.line_height
 
    def reset(self):
        """ Reset text to the top of the screen. """
        self.x_pos = 10
        self.y_pos = 10
        self.line_height = 15
 
    def indent(self):
        """ Indent the next line of text """
        self.x_pos += 10
 
    def unindent(self):
        """ Unindent the next line of text """
        self.x_pos -= 10

if __name__ == '__main__':
    main()
