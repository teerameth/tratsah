#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction # Delay
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():

    pkg_tratsah = get_package_share_directory('tratsah')
    
    # Start Gazebo, robot_state_publisher, joint_state_broadcaster, velocity_controller
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tratsah, 'launch', 'gazebo_mapping.launch.py'),
        )
    )

    rviz_file_name = 'mapping.rviz'
    rviz_file_path = os.path.join(pkg_tratsah, 'rviz', rviz_file_name)
    rviz = ExecuteProcess(
        cmd=['rviz2',
        '-d', rviz_file_path],
        output='screen')

    # Controller
    node_controller = Node(
        package='tratsah',
        executable='controller.py',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        start_world,
        TimerAction(actions = [rviz], period = 8.0),
        # node_controller,

    ])