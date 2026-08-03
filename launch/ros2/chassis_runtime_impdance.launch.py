#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-08-03
################################################################

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

    # args
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to use simulation time (/clock)')

    # chassis_runtime_impdance node
    impedance_param_path = PathJoinSubstitution(
        [impedance_pkg_path, "config", "ros2",
         "chassis_runtime_impdance_params.yaml"])
    urdf_file_path = PathJoinSubstitution(
        [urdf_pkg_path, "urdf", "model.urdf"])

    chassis_runtime_impdance_node = Node(
        package='hex_ros_demo_chassis_impedance',
        executable='chassis_runtime_impdance',
        name='chassis_runtime_impdance',
        output="screen",
        emulate_tty=True,
        parameters=[
            impedance_param_path,
            {
                "model_urdf": ParameterValue(urdf_file_path, value_type=str),
                "use_sim_time": LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('chs_state', 'chs_state'),
            ('chs_ctrl', 'chs_ctrl'),
            ('teleop_keyboard_state', 'teleop_keyboard_state'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        chassis_runtime_impdance_node,
    ])
