#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare(package='tadeo_ecar_safety').find('tadeo_ecar_safety')
    
    # Configuration files
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
    
    enable_emergency_stop_arg = DeclareLaunchArgument(
        'enable_emergency_stop',
        default_value='true',
        description='Enable emergency stop system'
    )
    
    enable_collision_avoidance_arg = DeclareLaunchArgument(
        'enable_collision_avoidance',
        default_value='true',
        description='Enable collision avoidance system'
    )
    
    enable_safety_monitor_arg = DeclareLaunchArgument(
        'enable_safety_monitor',
        default_value='true',
        description='Enable safety monitoring system'
    )
    
    enable_watchdog_arg = DeclareLaunchArgument(
        'enable_watchdog',
        default_value='true',
        description='Enable system watchdog'
    )
    
    enable_joystick_emergency_arg = DeclareLaunchArgument(
        'enable_joystick_emergency',
        default_value='true',
        description='Enable joystick emergency stop'
    )
    
    # Safety nodes group
    safety_nodes = GroupAction([
        # Emergency Stop Node
        Node(
            package='tadeo_ecar_safety',
            executable='emergency_stop_node',
            name='emergency_stop',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_emergency_stop').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'monitor.frequency': 20.0,
                    'emergency.enable_joystick': LaunchConfiguration('enable_joystick_emergency'),
                    'emergency.joystick_button': 0,
                    'emergency.auto_reset_timeout': 5.0,
                    'emergency.require_manual_reset': True,
                    'safety.max_cmd_age': 1.0,
                    'safety.enable_heartbeat': True,
                    'safety.heartbeat_timeout': 2.0
                }
            ],
            remappings=[
                ('joy', 'joy'),
                ('cmd_vel_input', 'cmd_vel_raw'),
                ('cmd_vel_safe', 'cmd_vel_emergency_filtered'),
                ('safety_status', 'safety_status'),
                ('system_health', 'system_health'),
            ],
        ),
        
        # Collision Avoidance Node
        Node(
            package='tadeo_ecar_safety',
            executable='collision_avoidance_node',
            name='collision_avoidance',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_collision_avoidance').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'safety.frequency': 20.0,
                    'zones.emergency_distance': 0.3,
                    'zones.warning_distance': 1.0,
                    'zones.slow_distance': 2.0,
                    'zones.angular_range': 1.57,
                    'collision.prediction_time': 3.0,
                    'collision.min_confidence': 0.7,
                    'avoidance.enable_lateral': True,
                    'avoidance.lateral_gain': 0.5,
                    'avoidance.deceleration_gain': 0.8,
                    'avoidance.min_velocity': 0.1
                }
            ],
            remappings=[
                ('cmd_vel_raw', 'cmd_vel_emergency_filtered'),
                ('cmd_vel_safe', 'cmd_vel_collision_filtered'),
                ('scan', 'scan'),
                ('perception/fused_obstacles', 'perception/fused_obstacles'),
                ('robot_status', 'robot_status'),
                ('safety_status', 'collision_safety_status'),
            ],
        ),
        
        # Safety Monitor Node
        Node(
            package='tadeo_ecar_safety',
            executable='safety_monitor_node',
            name='safety_monitor',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_safety_monitor').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'monitor.frequency': 50.0,
                    'limits.max_tilt_angle': 0.524,  # 30 degrees
                    'limits.max_lateral_acceleration': 8.0,
                    'limits.max_linear_velocity': 2.0,
                    'limits.max_angular_velocity': 1.5,
                    'limits.max_acceleration': 3.0,
                    'limits.max_deceleration': -5.0,
                    'battery.critical_voltage': 20.0,
                    'battery.warning_voltage': 22.0,
                    'battery.critical_percentage': 10.0,
                    'battery.warning_percentage': 20.0,
                    'stability.min_factor': 0.3,
                    'stability.warning_factor': 0.5,
                    'temperature.max_operating': 70.0,
                    'temperature.critical': 85.0
                }
            ],
            remappings=[
                ('imu/data', 'imu/data'),
                ('battery_state', 'battery_state'),
                ('cmd_vel', 'cmd_vel_collision_filtered'),
                ('robot_status', 'robot_status'),
                ('system_health', 'system_health'),
                ('safety_monitor_status', 'safety_monitor_status'),
            ],
        ),
        
        # Watchdog Node
        Node(
            package='tadeo_ecar_safety',
            executable='watchdog_node',
            name='watchdog',
            output='screen',
            condition=lambda: LaunchConfiguration('enable_watchdog').perform() == 'true',
            parameters=[
                robot_config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'watchdog.frequency': 10.0,
                    'heartbeat.frequency': 1.0,
                    'timeouts.control_system': 2.0,
                    'timeouts.perception_system': 3.0,
                    'timeouts.navigation_system': 5.0,
                    'timeouts.safety_system': 1.0,
                    'timeouts.hardware_interface': 2.0,
                    'critical_components.control': True,
                    'critical_components.perception': False,
                    'critical_components.navigation': False,
                    'critical_components.safety': True,
                    'critical_components.hardware': True,
                    'emergency.enable_auto_stop': True,
                    'emergency.critical_component_failures': 2,
                    'emergency.total_failure_threshold': 0.7
                }
            ],
            remappings=[
                ('system_health', 'system_health'),
                ('watchdog_status', 'watchdog_status'),
                ('system_alive', 'system_alive'),
                ('failed_components', 'failed_components'),
            ],
        ),
    ])
    
    return LaunchDescription([
        use_sim_time_arg,
        enable_emergency_stop_arg,
        enable_collision_avoidance_arg,
        enable_safety_monitor_arg,
        enable_watchdog_arg,
        enable_joystick_emergency_arg,
        safety_nodes,
    ])