from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    depthimage_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        remappings=[
            ('depth','/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan')
        ],
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.30,
            'range_max': 5.0,
            'scan_height': 1,
            'output_frame': 'depth_frame'
        }]
    )

    ld = LaunchDescription()
    ld.add_action(depthimage_to_laserscan_node)

    return ld
