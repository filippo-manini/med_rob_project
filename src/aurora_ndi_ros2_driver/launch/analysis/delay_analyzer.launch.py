#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # =============================================================================
    # LAUNCH ARGUMENTS
    # =============================================================================

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('aurora_ndi_ros2_driver'),
            'config', 'analysis',
            'delay_analyzer_config.yaml'
        ]),
        description='Path to delay analyzer configuration YAML file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error'],
        description='ROS logging level'
    )

    # =============================================================================
    # DELAY ANALYZER NODE
    # =============================================================================

    delay_analyzer_node = Node(
        package='aurora_ndi_ros2_driver',
        executable='delay_analyzer_node',
        name='delay_analyzer_node',
        parameters=[LaunchConfiguration('config_file')],
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level')
        ],
        output='screen',
        emulate_tty=True
    )

    # =============================================================================
    # LAUNCH DESCRIPTION
    # =============================================================================

    return LaunchDescription([
        # Arguments
        config_file_arg,
        log_level_arg,

        # Log startup info
        LogInfo(msg="=== Delay Analyzer Starting ==="),
        LogInfo(msg=["Config file: ", LaunchConfiguration('config_file')]),
        LogInfo(msg="==================================="),

        # Main node
        delay_analyzer_node,

        # Final log
        LogInfo(msg="Delay Analyzer launched!")
    ])
