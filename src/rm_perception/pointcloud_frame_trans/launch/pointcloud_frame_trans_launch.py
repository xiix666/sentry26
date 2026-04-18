from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_frame_trans',
            executable='pointcloud_frame_trans',
            name='pointcloud_frame_trans',
            output='screen',
            parameters=[
                {'input_topic': '/livox/lidar/pointcloud'},
                {'output_topic': '/livox/pointcloud2/transframe'},
                {'output_frame': 'base_link'}
            ]
        )
    ])