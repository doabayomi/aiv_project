import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import GroupAction, TimerAction, DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name='wave_rover' #<--- CHANGE ME
    share_dir = get_package_share_directory(package_name)

    lidar_odom_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : False,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 30.0}],
    )

    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(share_dir, 'params', 'imu_ekf.yaml')],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_transform",
                default_value = "true",
                description= "Publish transformation"
            ),
            lidar_odom_node, 
            TimerAction(
            actions=[ekf_node],
            period =2.0)
        ]
    )