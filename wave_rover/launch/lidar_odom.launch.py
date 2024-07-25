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

    pub_tranform = LaunchConfiguration('publish_transform')


    lidar_odom_node = Node(
        package="lidar_odometry",
        executable="lidar_odometry_node"
    )

    lidar_tranform = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(share_dir, 'params', 'lidar_ekf.yaml')],
    )

    if_lidar_cond = GroupAction(
        condition=IfCondition(pub_tranform),
        actions=[TimerAction(
            actions=[lidar_tranform],
            period =2.0)]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_transform",
                default_value = "true",
                description= "Publish transformation"
            ),
            lidar_odom_node, 
            if_lidar_cond
        ]
    )