#!/usr/bin/env python3
"""底盘 + EKF 启动：只跑底盘桥接、URDF 和 robot_localization。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('seedling_bringup')

    urdf_path = os.path.join(bringup_dir, 'urdf', 'seedling_robot.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='seedling_chassis',
            executable='chassis_node',
            name='seedling_chassis',
            output='screen',
            parameters=[
                os.path.join(bringup_dir, 'config', 'chassis.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(bringup_dir, 'config', 'ekf.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('/odometry/filtered', '/odom_combined')],
        ),
    ])
