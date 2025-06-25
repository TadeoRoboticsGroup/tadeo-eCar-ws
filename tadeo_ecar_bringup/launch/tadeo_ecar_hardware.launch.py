#!/usr/bin/env python3

"""
Launch file especializado para hardware real del Tadeo eCar 4WD4WS
Incluye drivers de hardware, calibración y configuración específica
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directorios de paquetes
    bringup_dir = get_package_share_directory('tadeo_ecar_bringup')
    config_dir = get_package_share_directory('tadeo_ecar_config')
    
    # Archivos de configuración
    robot_params_file = os.path.join(config_dir, 'config', 'robot_params.yaml')
    hardware_params_file = os.path.join(bringup_dir, 'params', 'hardware_params.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'tadeo_ecar_hardware.rviz')
    
    # Argumentos de lanzamiento
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='tadeo_ecar',
        description='Nombre del robot'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Lanzar RViz para visualización'
    )
    
    declare_use_foxglove = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Lanzar Foxglove Bridge para monitoreo remoto'
    )
    
    declare_auto_calibrate = DeclareLaunchArgument(
        'auto_calibrate',
        default_value='true',
        description='Realizar calibración automática al inicio'
    )
    
    declare_enable_safety = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Habilitar sistema de seguridad completo'
    )
    
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Habilitar navegación autónoma'
    )
    
    declare_enable_mapping = DeclareLaunchArgument(
        'enable_mapping',
        default_value='false',
        description='Habilitar modo de mapeo'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nivel de logging'
    )
    
    declare_record_data = DeclareLaunchArgument(
        'record_data',
        default_value='false',
        description='Grabar datos de sensores con rosbag'
    )
    
    declare_battery_monitoring = DeclareLaunchArgument(
        'battery_monitoring',
        default_value='true',
        description='Habilitar monitoreo de batería'
    )
    
    # Variables de configuración
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_foxglove = LaunchConfiguration('use_foxglove')
    auto_calibrate = LaunchConfiguration('auto_calibrate')
    enable_safety = LaunchConfiguration('enable_safety')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_mapping = LaunchConfiguration('enable_mapping')
    log_level = LaunchConfiguration('log_level')
    record_data = LaunchConfiguration('record_data')
    battery_monitoring = LaunchConfiguration('battery_monitoring')
    
    # Sistema principal para hardware
    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_bringup'),
                'launch',
                'tadeo_ecar_full_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_simulation': 'false',
            'robot_name': robot_name,
            'use_rviz': use_rviz,
            'use_foxglove': use_foxglove,
            'slam_mode': 'slam_toolbox',
            'navigation_mode': 'full',
            'behavior_mode': 'full',
            'safety_mode': 'full',
            'log_level': log_level
        }.items()
    )
    
    # Grupo de Hardware Drivers
    hardware_drivers_group = GroupAction([
        # Driver de motores (placeholder - adaptar según hardware real)
        Node(
            package='tadeo_ecar_control',
            executable='motor_driver_node',
            name='motor_driver_node',
            parameters=[hardware_params_file],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Driver de sensores de wheel encoder (placeholder)
        Node(
            package='tadeo_ecar_control',
            executable='encoder_driver_node',
            name='encoder_driver_node',
            parameters=[hardware_params_file],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Driver de LiDAR (placeholder - adaptar según modelo)
        Node(
            package='tadeo_ecar_perception',
            executable='lidar_driver_node',
            name='lidar_driver_node',
            parameters=[hardware_params_file],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Driver de cámara (placeholder)
        Node(
            package='tadeo_ecar_perception',
            executable='camera_driver_node',
            name='camera_driver_node',
            parameters=[hardware_params_file],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Driver de IMU (placeholder)
        Node(
            package='tadeo_ecar_perception',
            executable='imu_driver_node',
            name='imu_driver_node',
            parameters=[hardware_params_file],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        )
    ])
    
    # Grupo de Calibración
    calibration_group = GroupAction([
        # Calibración automática de ruedas
        Node(
            condition=IfCondition(auto_calibrate),
            package='tadeo_ecar_control',
            executable='wheel_calibration_node',
            name='wheel_calibration_node',
            parameters=[hardware_params_file],
            output='screen'
        ),
        
        # Calibración de sensores
        Node(
            condition=IfCondition(auto_calibrate),
            package='tadeo_ecar_perception',
            executable='sensor_calibration_node',
            name='sensor_calibration_node',
            parameters=[hardware_params_file],
            output='screen'
        )
    ])
    
    # Grupo de Monitoreo de Hardware
    hardware_monitoring_group = GroupAction([
        # Monitor de batería
        Node(
            condition=IfCondition(battery_monitoring),
            package='tadeo_ecar_safety',
            executable='battery_monitor_node',
            name='battery_monitor_node',
            parameters=[hardware_params_file],
            output='screen'
        ),
        
        # Monitor de temperatura
        Node(
            package='tadeo_ecar_safety',
            executable='temperature_monitor_node',
            name='temperature_monitor_node',
            parameters=[hardware_params_file],
            output='screen'
        ),
        
        # Monitor de voltajes
        Node(
            package='tadeo_ecar_safety',
            executable='voltage_monitor_node',
            name='voltage_monitor_node',
            parameters=[hardware_params_file],
            output='screen'
        )
    ])
    
    # Grupo de Grabación de Datos
    data_recording_group = GroupAction([
        # Grabación de tópicos importantes
        Node(
            condition=IfCondition(record_data),
            package='rosbag2',
            executable='record',
            name='data_recorder',
            arguments=[
                '--output', '/tmp/tadeo_ecar_data',
                '--topics', 
                '/scan', '/camera/image_raw', '/imu/data',
                '/odom', '/tf', '/tf_static',
                '/cmd_vel', '/joint_states',
                '/system_health', '/emergency_stop',
                '/battery_state', '/robot_state'
            ],
            output='screen'
        )
    ])
    
    # Sistema de watchdog para hardware
    watchdog_node = Node(
        package='tadeo_ecar_safety',
        executable='hardware_watchdog_node',
        name='hardware_watchdog_node',
        parameters=[hardware_params_file],
        output='screen',
        respawn=True,
        respawn_delay=5
    )
    
    return LaunchDescription([
        # Argumentos
        declare_robot_name,
        declare_use_rviz,
        declare_use_foxglove,
        declare_auto_calibrate,
        declare_enable_safety,
        declare_enable_navigation,
        declare_enable_mapping,
        declare_log_level,
        declare_record_data,
        declare_battery_monitoring,
        
        # Sistema principal
        full_system_launch,
        
        # Grupos específicos para hardware
        hardware_drivers_group,
        calibration_group,
        hardware_monitoring_group,
        data_recording_group,
        
        # Watchdog crítico
        watchdog_node
    ])