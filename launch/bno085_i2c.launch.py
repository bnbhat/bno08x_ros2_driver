from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

#
#  colcon build; source install/setup.bash; ros2 launch bno08x_driver bno085_i2c.launch.py
#

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bno08x_driver'), 
        'config',
        'bno085_i2c.yaml'
    )

    return LaunchDescription([
        Node(
            package='bno08x_driver',  
            executable='bno08x_driver',  
            name='bno08x_driver',
            output='screen',
            parameters=[config],
            remappings=[("imu", "imu/data"), ("magnetic_field","imu/mag")]
        ),
    ])