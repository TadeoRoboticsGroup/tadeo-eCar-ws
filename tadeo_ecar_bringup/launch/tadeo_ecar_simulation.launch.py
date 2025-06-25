#!/usr/bin/env python3

"""
Launch file especializado para simulación del Tadeo eCar 4WD4WS
Incluye Gazebo/Ignition, sensores simulados y visualización
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
    description_dir = get_package_share_directory('tadeo_ecar_description')
    
    # Archivos de configuración
    world_file = os.path.join(description_dir, 'worlds', 'empty_world_ignition.sdf')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'tadeo_ecar_simulation.rviz')
    
    # Argumentos de lanzamiento
    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=world_file,
        description='Archivo de mundo para simulación'
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='tadeo_ecar',
        description='Nombre del robot'
    )
    
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Posición X de spawn'
    )
    
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Posición Y de spawn'
    )
    
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.5',
        description='Posición Z de spawn'
    )
    
    declare_spawn_yaw = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
        description='Orientación inicial (yaw)'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Lanzar RViz'
    )
    
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Lanzar GUI del simulador'
    )
    
    declare_simulator = DeclareLaunchArgument(
        'simulator',
        default_value='ignition',
        choices=['gazebo', 'ignition'],
        description='Simulador a usar'
    )
    
    declare_enable_sensors = DeclareLaunchArgument(
        'enable_sensors',
        default_value='true',
        description='Habilitar sensores en simulación'
    )
    
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Habilitar stack de navegación'
    )
    
    declare_enable_slam = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Habilitar SLAM'
    )
    
    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='slam_toolbox',
        choices=['slam_toolbox', 'cartographer'],
        description='Modo SLAM'
    )
    
    # Variables de configuración
    world_file = LaunchConfiguration('world_file')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gui = LaunchConfiguration('use_gui')
    simulator = LaunchConfiguration('simulator')
    enable_sensors = LaunchConfiguration('enable_sensors')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_slam = LaunchConfiguration('enable_slam')
    slam_mode = LaunchConfiguration('slam_mode')
    
    # Sistema principal en modo simulación
    full_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_bringup'),
                'launch',
                'tadeo_ecar_full_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_simulation': 'true',
            'robot_name': robot_name,
            'use_rviz': use_rviz,
            'slam_mode': slam_mode,
            'navigation_mode': 'full',
            'behavior_mode': 'full',
            'safety_mode': 'full',
            'log_level': 'info'
        }.items()
    )
    
    # RViz especializado para simulación
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Teleop para control manual (opcional)
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        # Argumentos
        declare_world_file,
        declare_robot_name,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_use_rviz,
        declare_use_gui,
        declare_simulator,
        declare_enable_sensors,
        declare_enable_navigation,
        declare_enable_slam,
        declare_slam_mode,
        
        # Sistema completo
        full_system_launch,
        
        # Nodos adicionales para simulación
        teleop_node
    ])