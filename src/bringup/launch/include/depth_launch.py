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
            'scan_time': 0.033, #扫描时间间隔，30HZ
            'range_min': 0.30, #投影点的最小距离单位（米），更近的被丢弃
            'range_max': 5.0, #投影点的最大距离单位（米），更远的被丢弃
            'scan_height': 1, #depthimage中用于转成laserscan的行
            'output_frame': 'depth_frame' #发布的帧 ID
        }]
    )

    ld = LaunchDescription()
    ld.add_action(depthimage_to_laserscan_node)

    return ld