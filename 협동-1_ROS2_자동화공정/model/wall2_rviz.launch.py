import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '/home/rokey/rokey_team_ai_workspace/협동-1_ROS2_자동화공정/model/wall2.xml'

    with open(urdf_path, 'r') as f:
        urdf_content = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content, 'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/rokey/rokey_team_ai_workspace/협동-1_ROS2_자동화공정/model/wall2.rviz'] if os.path.exists('/home/rokey/rokey_team_ai_workspace/협동-1_ROS2_자동화공정/model/wall2.rviz') else [],
        ),
    ])