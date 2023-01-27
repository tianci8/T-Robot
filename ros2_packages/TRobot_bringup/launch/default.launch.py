'''
Author: Tianci Zhang
Email: tianci_zhang@tju.edu.cn
Date: 2023-01-27 21:02:45
LastEditors: Tianci Zhang
LastEditTime: 2023-01-27 21:03:08
FilePath: \ros2_packages\TRobot_bringup\launch\default.launch.py
Description: 

Copyright (c) 2023 by tianci_zhang@tju.edu.cn, All Rights Reserved. 
'''


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # launch传感器，若有其他传感器，可在自行添加
    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('ld14'), 'launch', 'ld14.launch.py']
    )
    
    #launch rviz2
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('TRobot_description'), 'launch', 'description.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("TRobot_bringup"), "config", "ekf.yaml"]
    )
    return LaunchDescription([
        # 设置micro_ros串口
        DeclareLaunchArgument(
            name='micro_ros_serial_port', 
            default_value='/dev/ttyUSB0',
            description='micro_ros Serial Port'
        ),

        # 设置串口通讯波特率
        DeclareLaunchArgument(
            name='micro_ros_baudrate', 
            default_value='460800',
            description='baudrate'
        ),

        # 启动micro_ros_agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration("micro_ros_serial_port"), 
                                    '-b',LaunchConfiguration("micro_ros_baudrate")]
        ),
        
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     parameters=[
        #         ekf_config_path
        #     ],
        #     remappings=[("odometry/filtered", "odom")]
        # ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
        )
    ])