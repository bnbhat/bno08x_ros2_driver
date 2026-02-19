from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

#
#  colcon build; source install/setup.bash; ros2 launch bno08x_driver bno085_i2c_test.launch.py
#

def generate_launch_description():

    namespace = ""

    return LaunchDescription([

        Node(
            package="bno08x_driver",
            namespace=namespace,
            executable="bno08x_driver",
            name="bno08x_driver",
            output='screen',
            respawn=True,
            respawn_delay=4,
            emulate_tty=True,
            parameters=[{
                'frame_id': 'imu_link',
                'verbose': True,
                'i2c.enabled': True,
                'i2c.bus': "/dev/i2c-1", # I2C bus device (bus "1" is default on Raspberry Pi, use "ls /dev/i2c*")
                'i2c.address': "0x4B",   # I2C address (0x4A or 0x4B depending on the board configuration, use "i2cdetect -y 1")
                'publish.magnetic_field.enabled': True,
                'publish.magnetic_field.rate': 60,   # max 100 Hz
                'publish.imu.enabled': True,
                'publish.imu.rate': 60,  # max 400 Hz (anything above 100Hz depends on your hardware)
                'publish.imu.orientation_yaw_variance': 0.0075  # default 0.0075 (≈5°) means pretty trustworthy
            }],
            remappings=[("imu", "imu/data"), ("magnetic_field","imu/mag")]
        ),

        # for experiments: RViz starts with "map" as Global Fixed Frame, provide a TF to see axes etc.
        Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments=[
                '--x', '0.0',     # X translation in meters
                '--y', '0.0',     # Y translation in meters
                '--z', '0.1',     # Z translation in meters
                '--roll', '0.0',  # Roll in radians
                '--pitch', '0.0', # Pitch in radians
                '--yaw', '0.0',   # Yaw in radians (e.g., 90 degrees)
                '--frame-id', 'map', # Parent frame ID
                '--child-frame-id', 'imu_link' # Child frame ID
            ]
        )
    ])
