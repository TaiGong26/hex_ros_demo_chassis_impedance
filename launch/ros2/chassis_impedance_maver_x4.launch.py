#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    impedance_pkg_path = FindPackageShare('hex_ros_demo_chassis_impedance')
    urdf_pkg_path = FindPackageShare('hex_ros_urdf_maver_x4')
    impedance_param_path = PathJoinSubstitution(
        [impedance_pkg_path, 'config', 'ros2', 'params_maver_x4.yaml'])
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, 'urdf', 'model.urdf'])
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                             default_value='true',
                                             choices=['true', 'false'])

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='hex_ros_demo_chassis_impedance',
            executable='chassis_impedance',
            name='chassis_impedance',
            output='screen',
            emulate_tty=True,
            parameters=[
                impedance_param_path,
                {
                    'model_urdf':
                    ParameterValue(urdf_file_path, value_type=str),
                    'use_sim_time':
                    ParameterValue(LaunchConfiguration('use_sim_time'),
                                   value_type=bool),
                },
            ],
            remappings=[
                ('chs_state', 'chs_state'),
                ('chs_ctrl', 'chs_ctrl'),
                ('teleop_keyboard_state', '/teleop_keyboard_state'),
                ('cmd_vel', 'cmd_vel'),
            ],
        ),
    ])
