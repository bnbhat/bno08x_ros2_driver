from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bno08x_driver'), 
        'config',
        'bno085_uart.yaml'
    )

    return LaunchDescription([
        Node(
            package='bno08x_driver',  
            executable='bno08x_driver',  
            name='bno08x_driver',
            output='screen',
            parameters=[config]
        ),
    ])