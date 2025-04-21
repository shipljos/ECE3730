from launch import LaunchDescription
from launch_ros.actions import Node
import os.path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
        ),
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
        ),
        Node(
            package='joy',
            executable='joy_node',
        ),
        Node(
            package='robot',
            executable='peripheral_subscriber',
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('robot'), 'config', 'rviz_config1.rviz')]
        ),
    ])
