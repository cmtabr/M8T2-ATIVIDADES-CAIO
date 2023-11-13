import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'),
            '/turtlebot3_world.launch.py'])
            )
    navigator_ros2 = ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', 'use_sim_time:=True', 'map:=teste.yaml'],
            name='navigator_ros2',
            output='screen'
        )
    map_node = ExecuteProcess(
        cmd=['ros2', 'run', 'your_map_package', 'your_map_node'],     name='map_node',
        output='screen'
    )

    initial_pose = ExecuteProcess(
            cmd=['ros2', 'run', 'navigation_package', 'set_origin'],
            name='robot_origin',
            output='screen'
        )
    go_to_location = ExecuteProcess(
            cmd=['ros2', 'run', 'navigation_package', 'naviation_controller'],
            name='pathing_planner',
            output='screen'
        )
    delay_action = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['echo', 'Waiting for 5 seconds...'],
                name='delay_echo',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_world,
        navigator_ros2,
        map_node,
        initial_pose,
        delay_action,
        go_to_location
    ])

if __name__ == "__main__":
    generate_launch_description()