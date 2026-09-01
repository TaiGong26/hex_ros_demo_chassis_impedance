#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg_path = FindPackageShare('hex_ros_sim_trigger_a')
    keyboard_pkg_path = FindPackageShare('hex_ros_teleop_keyboard')
    joystick_pkg_path = FindPackageShare('hex_ros_teleop_joystick')
    impedance_pkg_path = FindPackageShare('hex_ros_demo_chassis_impedance')

    viewer_arg = DeclareLaunchArgument(name='viewer',
                                       default_value='true',
                                       choices=['true', 'false'])
    rviz_arg = DeclareLaunchArgument(name='rviz',
                                     default_value='true',
                                     choices=['true', 'false'])
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg_path, 'sim_trigger_a.launch.py'])),
        launch_arguments={
            'viewer': LaunchConfiguration('viewer'),
            'rviz': LaunchConfiguration('rviz'),
            'test': 'false',
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
                [impedance_pkg_path,
                 'chassis_impedance_trigger_a.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items())

    return LaunchDescription([
        viewer_arg,
        rviz_arg,
        sim_launch,
        keyboard_launch,
        joystick_launch,
        impedance_launch,
    ])
