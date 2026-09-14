#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joystick_pkg_path = FindPackageShare('hex_ros_teleop_joystick')
    keyboard_pkg_path = FindPackageShare('hex_ros_teleop_keyboard')
    chassis_pkg_path = FindPackageShare('hex_ros_robot_chassis')
    impedance_pkg_path = FindPackageShare('hex_ros_demo_chassis_impedance')

    robot_host_arg = DeclareLaunchArgument(name='robot_host',
                                           default_value='192.168.1.100')
    robot_port_arg = DeclareLaunchArgument(name='robot_port',
                                           default_value='8439')

    chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([chassis_pkg_path, 'maver.launch.py'])),
        launch_arguments={
            'robot_host': LaunchConfiguration('robot_host'),
            'robot_port': LaunchConfiguration('robot_port'),
        }.items(),
    )
    keyboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [keyboard_pkg_path, 'teleop_keyboard.launch.py'])))
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [joystick_pkg_path, 'teleop_joystick.launch.py'])),
        launch_arguments={
            'use_cmd': 'true',
            'cmd_topic': 'cmd_vel',
        }.items(),
    )
    impedance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [impedance_pkg_path, 'chassis_impedance_maver_x4.launch.py'])),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([
        robot_host_arg,
        robot_port_arg,
        keyboard_launch,
        joystick_launch,
        chassis_launch,
        impedance_launch,
    ])
