#! /usr/bin/env python3 
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    navigator_ros2 = ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'use_sim_time:=True', 'map:=../assets/mapa.yaml'],
            name='navigator_ros2',
            output='screen'
        )

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'),
            '/turtlebot3_world.launch.py'])
            )
    
    origin = Node(
            package='chatbot_package',
            executable='set_origin',
            name='set_origin',
            output='screen'
        )
    
    chatbot = Node(
            package='chatbot_package',
            executable='chatbot_controller',
            name='chatbot_controller',
            prefix = 'gnome-terminal --',
            output='screen'
        )

    return LaunchDescription([
        navigator_ros2,
        gazebo_world,
        origin,
        chatbot
    ])

if __name__ == "__main__":
    generate_launch_description()
