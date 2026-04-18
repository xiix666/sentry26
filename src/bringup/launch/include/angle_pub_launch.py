from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='angle_pub',  
            executable='angle_pub',  
            name='angle_pub_node',  
            output='screen',  
            parameters=[
                {'yaw_energy_': -0.89},
                {'yaw_tunnel_forward': 0.50},
                {'yaw_tunnel_backward': -2.623},
            ],
        )
    ])