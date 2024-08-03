import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name='wave_rover' #<--- CHANGE ME
    share_dir = get_package_share_directory(package_name)
    joy_params = os.path.join(share_dir,'config','joystick.yaml')


    launch_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            share_dir,'launch','imu_calib.launch.py'
        )])
    ) 

    launch_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            share_dir,'launch','ekf.launch.py'
        )])
    )

    # launch_joystick = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         share_dir,'launch','joystick.launch.py'
    #     )])
    # )

    twist_mux_params = os.path.join(share_dir,'params','twist_mux.yaml')
    twist_mux = Node(package="twist_mux",
                     executable="twist_mux",
                     parameters=[twist_mux_params, {"use_sim_time":False}],
                     remappings=[("/cmd_vel_out", '/cmd_vel')]
                     )


    # Launch them all!
    return LaunchDescription([
        launch_imu,
        launch_ekf,
        # launch_joystick,
        twist_mux
    ])
