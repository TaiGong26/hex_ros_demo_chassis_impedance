#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-08-03
################################################################

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keyboard_pkg_path = FindPackageShare('hex_ros_teleop_keyboard')
    chassis_pkg_path = FindPackageShare('hex_ros_robot_chassis')
    impedance_pkg_path = FindPackageShare('hex_ros_demo_chassis_impedance')

    # args
    robot_host_arg = DeclareLaunchArgument(
        name='robot_host',
        default_value='192.168.1.100',
        description='Robot controller IP address')
    robot_port_arg = DeclareLaunchArgument(
        name='robot_port',
        default_value='8439',
        description='Robot controller WebSocket port')

    # keyboard teleop
    keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [keyboard_pkg_path, "teleop_keyboard.launch.py"])), )

    # real chassis driver (Maver, robot_type default 30 = X4H1)
    maver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [chassis_pkg_path, "maver.launch.py"])),
        launch_arguments={
            'robot_host': LaunchConfiguration('robot_host'),
            'robot_port': LaunchConfiguration('robot_port'),
        }.items(),
    )

    # runtime impedance control node
    impedance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [impedance_pkg_path,
                 "chassis_runtime_impdance.launch.py"])),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        robot_host_arg,
        robot_port_arg,
        keyboard_launch,
        maver_launch,
        impedance_launch,
    ])
