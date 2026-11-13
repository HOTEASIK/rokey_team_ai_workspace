from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    share = Path(get_package_share_directory('hotteok_press_description'))
    urdf = (share / 'urdf' / 'hotteok_press.urdf').read_text(encoding='utf-8')
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'view.rviz')], output='screen'),
    ])
