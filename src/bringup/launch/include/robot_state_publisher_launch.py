from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            # "0.33",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        remappings=remappings,
    )
    # static_transform_node_odom2lidar = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_node_odom2lidar",
    #     output="screen",
    #     arguments=[
    #         "--x",
    #         "0.0",
    #         "--y",
    #         "0.0",
    #         "--z",
    #         "0.0",
    #         "--roll",
    #         "0.0",
    #         "--pitch",
    #         "0.0",
    #         "--yaw",
    #         "0.0",
    #         "--frame-id",
    #         "odom",
    #         "--child-frame-id",
    #         "lidar_odom",
    #     ],
    #     remappings=remappings,
    # )
    static_transform_node_odom2init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_node_odom2lidarlink",
        output="screen",
        arguments=[
            "--x",
            "-0.0",
            "--y",
            "0.174",
            "--z",
            "0.358",
            "--roll",
            "-0.78",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "odom",
            "--child-frame-id",
            "odom_init",
        ],
        remappings=remappings,
    )
    start_static_transform_node_lidar2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_lidar2lidar_1",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "-0.245",
            "--z",
            "-0.245",
            "--roll",
            "1.57",
            "--pitch",
            "-0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "lidar_link",
            "--child-frame-id",
            "lidar_link_1",
        ],
        remappings=remappings,
    )
    start_static_transform_lidar2base_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_lidar2base",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.127",
            "--z",
            "-0.377",
            "--roll",
            "0.78",
            "--pitch",
            "-0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "lidar_link",
            "--child-frame-id",
            "base_link",
        ],
        remappings=remappings,
    )

    static_transform_node_base2rs = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_node_base2rs",
        output="screen",
        arguments=[
            "--x",
            "-0.11",
            "--y",
            "0.134",
            "--z",
            "0.0",
            "--roll",
            "3.14",
            "--pitch",
            "0.0",
            "--yaw",
            "1.57",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "depth_frame",
        ],
        remappings=remappings,
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(start_static_transform_node_lidar2)
    ld.add_action(start_static_transform_node) 
    # ld.add_action(static_transform_node_odom2lidar)   
    ld.add_action(static_transform_node_odom2init)
    ld.add_action(start_static_transform_lidar2base_node)
    # ld.add_action(static_transform_node_base2rs)
    return ld