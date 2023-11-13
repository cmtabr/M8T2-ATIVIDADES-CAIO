from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

launch = LaunchDescription()

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_cartographer_dir = get_package_share_directory(
        'turtlebot3_cartographer'
    )

    turtlebot3_world_launch_file = os.path.join(turtlebot3_gazebo_dir, 
                                                'launch', 
                                                'turtlebot3_world.launch.py'
                                                )
    turtlebot3_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_world_launch_file),
    )
    launch.add_action(turtlebot3_world_launch)

    cartographer_launch_file = os.path.join(turtlebot3_cartographer_dir,
                                            'launch', 
                                            'cartographer.launch.py'
                                        )
    turtlebot3_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_launch_file),
        launch_arguments={'use_sim_time': 'True'}.items()
    )
    launch.add_action(turtlebot3_cartographer_launch)

    turtlebot3_teleop_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='turtlebot3_teleop_keyboard',
        prefix = 'gnome-terminal --',
        output='screen'
    )
    launch.add_action(turtlebot3_teleop_node),

    Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
            ]
    ),

    return launch
