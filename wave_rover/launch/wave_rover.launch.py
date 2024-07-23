#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ldr_port = LaunchConfiguration('lidar_port')
    rover_port = LaunchConfiguration('rover_port')

    rover_node = Node(
        package="wave_rover_control",
        executable="rover_control",
        parameters=[("port", rover_port)]
    )

    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'port_name': ldr_port},
        {'frame_id': 'laser_frame'},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
        ]
    )

    # base_link to base_laser tf node
    base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld19',
    arguments=['0','-0.03','0.056','0','0','-${pi}','base_link','laser_frame']
    )


    # Define LaunchDescription variable
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_port",
                default_value = '/dev/ttyUSB1',
                description = "LiDAR Port"
            ),
            DeclareLaunchArgument(
                "rover_port",
                default_value = '/dev/ttyUSB0',
                description = "Rover Port"
            ),
        ]
    )

    ld.add_action(rover_node)
    ld.add_action(ldlidar_node)
    ld.add_action(base_link_to_laser_tf_node)

    return ld