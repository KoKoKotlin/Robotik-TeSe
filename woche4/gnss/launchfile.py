from launch import LaunchDescription
from launch_ros.actions import Node

# get ublox nodes from: https://github.com/KumarRobotics/ublox

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='test'
        )
    ])