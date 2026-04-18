from launch import LaunchDescription
from launch_ros.actions import Node
import math

M_PI = math.pi

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='angle_pub',  
            executable='angle_pub',  
            name='angle_pub_node',  
            output='screen',  
            parameters=[
                {'yaw_energy_': -M_PI / 6.0},
                {'yaw_tunnel_forward': M_PI / 6.0},
                {'yaw_tunnel_backward': -M_PI / 6.0 * 5},
            ],
        )
    ])