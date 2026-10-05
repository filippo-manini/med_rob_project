#!/usr/bin/env python3

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    ExecuteProcess,
    OpaqueFunction
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def read_filter_config(config_file_path):
    """Read filter enable flags from aurora_tracking_config.yaml"""
    # Default values
    enable_kalman = True
    enable_lowpass = False

    try:
        with open(config_file_path, 'r') as f:
            config = yaml.safe_load(f)
            if 'aurora_publisher_node' in config and 'ros__parameters' in config['aurora_publisher_node']:
                params = config['aurora_publisher_node']['ros__parameters']
                enable_kalman = params.get('enable_kalman_filter', True)
                enable_lowpass = params.get('enable_lowpass_filter', False)
    except Exception as e:
        print(f"Warning: Could not read filter config from {config_file_path}: {e}")
        print("Using default values: enable_kalman=True, enable_lowpass=False")

    return enable_kalman, enable_lowpass


def launch_setup(context):
    """Setup function that will be called with the launch context"""

    # Get the config file path from launch configuration
    config_file = LaunchConfiguration('config_file').perform(context)
    kalman_config_file = LaunchConfiguration('kalman_config_file').perform(context)
    lowpass_config_file = LaunchConfiguration('lowpass_config_file').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

    # Read filter enable flags from main config
    enable_kalman, enable_lowpass = read_filter_config(config_file)

    nodes_to_launch = []

    # Kalman filter node (conditionally added)
    if enable_kalman:
        kalman_filter_node = Node(
            package='aurora_ndi_ros2_driver',
            executable='kalman_filter_node',
            name='kalman_filter_node',
            namespace=namespace,
            parameters=[kalman_config_file],
            arguments=[
                '--ros-args',
                '--log-level', log_level
            ],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=3
        )
        nodes_to_launch.append(kalman_filter_node)
        nodes_to_launch.append(LogInfo(msg="Kalman filter: ENABLED"))
    else:
        nodes_to_launch.append(LogInfo(msg="Kalman filter: DISABLED"))

    # Low-pass filter node (conditionally added)
    if enable_lowpass:
        lowpass_filter_node = Node(
            package='aurora_ndi_ros2_driver',
            executable='lowpass_filter_node',
            name='lowpass_filter_node',
            namespace=namespace,
            parameters=[lowpass_config_file],
            arguments=[
                '--ros-args',
                '--log-level', log_level
            ],
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=3
        )
        nodes_to_launch.append(lowpass_filter_node)
        nodes_to_launch.append(LogInfo(msg="Low-pass filter: ENABLED"))
    else:
        nodes_to_launch.append(LogInfo(msg="Low-pass filter: DISABLED"))

    return nodes_to_launch


def generate_launch_description():
    # =============================================================================
    # LAUNCH ARGUMENTS
    # =============================================================================
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('aurora_ndi_ros2_driver'),
            'config', 'driver',
            'aurora_tracking_config.yaml'
        ]),
        description='Path to Aurora configuration YAML file'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        choices=['true', 'false'],
        description='Enable debug mode with verbose logging'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error'],
        description='ROS logging level'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        choices=['true', 'false'],
        description='Launch RViz for Aurora pose visualization'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        choices=['true', 'false'],
        description='Publish static transform from world to aurora_base'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for Aurora node (optional)'
    )

    kalman_config_file_arg = DeclareLaunchArgument(
        'kalman_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('aurora_ndi_ros2_driver'),
            'config', 'driver',
            'kalman_filter_config.yaml'
        ]),
        description='Path to Kalman filter configuration YAML file'
    )

    lowpass_config_file_arg = DeclareLaunchArgument(
        'lowpass_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('aurora_ndi_ros2_driver'),
            'config', 'driver',
            'lowpass_filter_config.yaml'
        ]),
        description='Path to low-pass filter configuration YAML file'
    )

    # =============================================================================
    # MAIN AURORA PUBLISHER NODE
    # =============================================================================
    
    aurora_publisher_node = Node(
        package='aurora_ndi_ros2_driver',
        executable='aurora_publisher_node',
        name='aurora_publisher_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('config_file')],
        arguments=[
            '--ros-args', 
            '--log-level', LaunchConfiguration('log_level')
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=3
    )
    
    # =============================================================================
    # STATIC TRANSFORM PUBLISHER 
    # =============================================================================
    
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aurora_base_tf_publisher',
        arguments=[
            '0', '0', '0',      # translation x, y, z (meters)
             '0', '0.7071', '0', '0.7071', # rotation x, y, z, w (quaternion)
            'world',            # parent frame
            'aurora_base'       # child frame
        ],
        output='log',
        condition=IfCondition(LaunchConfiguration('publish_tf'))
    )

    # =============================================================================
    # RVIZ VISUALIZATION
    # =============================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='aurora_rviz',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('aurora_ndi_ros2_driver'),
                'config',
                'aurora_viz.rviz'
            ])
        ],
        output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # =============================================================================
    # DEBUG MONITORING (only if debug=true)
    # =============================================================================
    
    topic_monitor = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'hz', 'aurora/sensor0',
            '--window', '50', '--wall-time'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug'))
    )
    
    # =============================================================================
    # LAUNCH DESCRIPTION
    # =============================================================================
    
    return LaunchDescription([
        # Arguments
        config_file_arg,
        debug_arg,
        log_level_arg,
        use_rviz_arg,
        publish_tf_arg,
        namespace_arg,
        kalman_config_file_arg,
        lowpass_config_file_arg,

        # Log startup info
        LogInfo(msg="=== Aurora Publisher Starting ==="),
        LogInfo(msg=["Config: ", LaunchConfiguration('config_file')]),
        LogInfo(msg=["Kalman Config: ", LaunchConfiguration('kalman_config_file')]),
        LogInfo(msg=["Lowpass Config: ", LaunchConfiguration('lowpass_config_file')]),
        LogInfo(msg=["Debug: ", LaunchConfiguration('debug')]),
        LogInfo(msg=["RViz: ", LaunchConfiguration('use_rviz')]),
        LogInfo(msg="Filter enable flags are read from config file"),
        LogInfo(msg="==================================="),

        # Main nodes
        aurora_publisher_node,
        static_tf_publisher,
        rviz_node,

        # Debug tools
        topic_monitor,

        # Dynamically add filter nodes based on config
        OpaqueFunction(function=launch_setup),

        # Final log
        LogInfo(msg="Aurora Publisher launch complete!")
    ])
