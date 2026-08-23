from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('picka_hardware'), 'config', 'hardware.yaml')
    return LaunchDescription([
        Node(
            package='picka_hardware',
            executable='serial_bridge',
            name='serial_bridge',
            parameters=[config],
            output='screen',
        )
    ])
