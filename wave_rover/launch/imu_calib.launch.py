import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    package_name='wave_rover' #<--- CHANGE ME
    share_dir = get_package_share_directory(package_name)

    imu_calib = Node(
        package="imu_calib",
        executable= "apply_calib",
        output = "screen",
        parameters=[
            {"calib_file": os.path.join(share_dir, 'params', 'imu_calib.yaml')}
        ]
    )    

    complementary_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings=[
            ("imu/data_raw", "/imu/corrected")
        ]
    )
    
    return LaunchDescription(
        [
            imu_calib,
            TimerAction(
            actions=[complementary_filter],
            period =2.0)
            ,
        ]
    )
