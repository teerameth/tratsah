#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, NavigationResult

import rclpy
from rclpy.duration import Duration
import time

from rclpy import Node

bed_positions = {
    '1': (-30.20, 6.77, 0.18, 0.98),
    '2': (-19.22, 8.75, 0.97, 0.24),
    '3': (-7.94, 9.62, 0.99, 0.12),
    '4': (0.23, 8.05, 0, 0.99),
    '5': (-30.96, -1.40, 0.99, -0.05),
    '6': (-16.5, -5.14, 0.79, 0.61),
    '7': (-6.1, -2.67, 0.99, 0.14)
}

home_position = (0, 0, 0, 0)
storage_position = (10.5, 6.0, 0.72, 0.69)
bin_position = (15.74, 5.05, 0.57, 0.82)

color_index = {'blue':0, 'green':1, 'red':2}

def get_bed_pose(navigator:BasicNavigator(), n:int):
    global bed_positions
    bed_pose = PoseStamped()
    bed_pose.header.frame_id = 'map'
    bed_pose.header.stamp = navigator.get_clock().now().to_msg()
    bed_pose.pose.position.x = bed_positions[n][0]
    bed_pose.pose.position.y = bed_positions[n][1]
    bed_pose.pose.orientation.z = bed_positions[n][2]
    bed_pose.pose.orientation.w = bed_positions[n][3]
    return bed_pose

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Generate general poses
    home_pose, storage_pose, bin_pose = PoseStamped(), PoseStamped(), PoseStamped()
    home_pose.header.frame_id, storage_pose.header.frame_id, bin_pose.header.frame_id = 'map', 'map', 'map'
    home_pose.header.stamp, storage_pose.header.stamp, bin_pose.header.stamp = navigator.get_clock().now().to_msg(), navigator.get_clock().now().to_msg(), navigator.get_clock().now().to_msg()
    home_pose.pose.position.x = home_position[0]
    home_pose.pose.position.y = home_position[1]
    home_pose.pose.orientation.z = home_position[2]
    home_pose.pose.orientation.w = home_position[3]
    storage_pose.pose.position.x = storage_position[0]
    storage_pose.pose.position.y = storage_position[1]
    storage_pose.pose.orientation.z = storage_position[2]
    storage_pose.pose.orientation.w = storage_position[3]
    bin_pose.pose.position.x = bin_position[0]
    bin_pose.pose.position.y = bin_position[1]
    bin_pose.pose.orientation.z = bin_position[2]
    bin_pose.pose.orientation.w = bin_position[3]

    # Set robot initial pose
    navigator.setInitialPose(home_pose)
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    queue = [] # {'type':0=nothing 1=pick 2=place, 'color':0=blue 1=green 2=red, 'destination': PoseStamped()}
    while True:
        command = str(input("Enter command: "))
        if 'home' in command:
            print('home')
            queue.append({'type':0, 'color':None, 'destination': home_pose})
        elif 'pick' in command:
            print('pick')
            sub_commands = command.split(' ')
            if sub_commands[1] == '-c' or sub_commands[1] == '--color': # pick -c x -b y
                color_code = int(sub_commands[2])
                bed_code = int(sub_commands[4])
            else:                                                       # pick -b x -c y
                color_code = int(sub_commands[4])
                bed_code = int(sub_commands[2])
            queue.append({'type':1, 'color':color_code, 'destination': storage_pose})
            queue.append({'type':2, 'color':None, 'destination': get_bed_pose[bed_code]})
        elif 'dump' in command:
            dump_all = True if '-a' in command or '--all' in command else False
            print('dump')
            sub_commands = command.split(' ')
            if sub_commands[1] == '-b' or sub_commands[1] == '--bed':
                bed_code = int(sub_commands[2])
            elif sub_commands[2] == '-b' or sub_commands[2] == '--bed':
                bed_code = int(sub_commands[3])
            queue.append({'type':1, 'color':None, 'destination': get_bed_pose[bed_code]})
            queue.append({'type':2, 'color':None, 'destination': bin_pose})
        time.sleep(1)
        print("Excuting commands")
        order = None
        while len(queue):   # Loop until queue ran out
            order = queue.pop()
            
            # request_item_location = request_destination
            order['destination'].header.stamp = navigator.get_clock().now().to_msg() # update time stamp before execute
            navigator.goToPose(order['destination'])

            # Do something during our route
            # (e.x. queue up future tasks or detect person for fine-tuned positioning)
            # Simply print information for workers on the robot's ETA for the demonstation
            i = 0
            while not navigator.isNavComplete():
                i += 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival for worker: ' + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
            
            result = navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                if order['type'] == 1: # Need to pick (Use moveit2)
                    ### scan for box with order['color'] ###
                    ### place ###
                    pass
                elif order['type'] == 2: # Needed to place (use moveit2)
                    ### scan for box(es) in area ###
                    ### Place on empty space ###
                    pass
            elif result == NavigationResult.CANCELED:
                print(f'Task was canceled. Returning home')
                queue.clear() # Clear all command
                home_pose.header.stamp = navigator.get_clock().now().to_msg()
                navigator.goToPose(home_pose)

            elif result == NavigationResult.FAILED:
                print(f'Task failed!')
                exit(-1)

            while not navigator.isNavComplete():
                pass
        if order is not None:
            # No more order -> go home
            navigator.goToPose(home_pose)
    
if __name__ == '__main__':
    main()