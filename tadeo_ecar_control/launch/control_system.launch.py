#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare(package='tadeo_ecar_control').find('tadeo_ecar_control')
    
    # Configuration files
    control_config = os.path.join(
        FindPackageShare('tadeo_ecar_config').find('tadeo_ecar_config'),
        'config', 'control_params.yaml'
    )
    
    robot_config = os.path.join(
        FindPackageShare('tadeo_ecar_config').find('tadeo_ecar_config'),
        'config', 'robot_params.yaml'
    )
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_4ws_arg = DeclareLaunchArgument(
        'enable_4ws',
        default_value='true',
        description='Enable four wheel steering'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='50.0',
        description='Control loop frequency'
    )
    
    # Control nodes group
    control_nodes = GroupAction([
        # Wheel Controller Node
        Node(
            package='tadeo_ecar_control',
            executable='wheel_controller_node',
            name='wheel_controller',
            output='screen',
            parameters=[
                control_config,
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'control.frequency': LaunchConfiguration('control_frequency')
                }
            ],
            remappings=[
                ('cmd_vel', 'cmd_vel_raw'),
                ('joint_states', 'joint_states'),
                ('wheel_states', 'wheel_states'),
            ],
        ),
        
        # Four Wheel Steering Controller Node
        Node(
            package='tadeo_ecar_control',
            executable='four_wheel_steering_controller_node',
            name='four_wheel_steering_controller',
            output='screen',
            parameters=[
                control_config,
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'control.enable_4ws': LaunchConfiguration('enable_4ws'),
                    'control.frequency': LaunchConfiguration('control_frequency')
                }
            ],
            remappings=[
                ('cmd_vel', 'cmd_vel'),
                ('cmd_vel_out', 'cmd_vel_raw'),
                ('wheel_states', 'wheel_states'),
                ('odom', 'odom'),
                ('robot_status', 'robot_status'),
            ],
        ),
        
        # Vehicle Dynamics Node
        Node(
            package='tadeo_ecar_control',
            executable='vehicle_dynamics_node',
            name='vehicle_dynamics',
            output='screen',
            parameters=[
                control_config,
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'dynamics.frequency': 100.0
                }
            ],
            remappings=[
                ('cmd_vel', 'cmd_vel_raw'),
                ('dynamics/cmd_vel', 'cmd_vel_dynamics'),
                ('wheel_states', 'wheel_states'),
                ('imu', 'imu'),
            ],
        ),
    ])
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_4ws_arg,
        control_frequency_arg,
        control_nodes,
    ])