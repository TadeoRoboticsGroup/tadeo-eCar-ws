#!/usr/bin/env python3

"""
Launch file para el sistema completo de navegación del Tadeo eCar
Incluye todos los nodos de navegación: controller, mission executor, waypoint manager y monitor
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Rutas de paquetes
    tadeo_navigation_dir = get_package_share_directory('tadeo_ecar_navigation')
    tadeo_config_dir = get_package_share_directory('tadeo_ecar_config')
    
    # Archivos de configuración
    navigation_params_file = os.path.join(tadeo_navigation_dir, 'config', 'navigation_params.yaml')
    
    # Argumentos de lanzamiento
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulación si está en true'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=navigation_params_file,
        description='Ruta completa al archivo de parámetros de navegación'
    )
    
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace del robot'
    )
    
    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Si usar namespace o no'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Iniciar automáticamente los nodos del ciclo de vida'
    )
    
    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Usar nodos compuestos para mejorar el rendimiento'
    )
    
    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Si respawn los nodos cuando mueran'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nivel de logging'
    )
    
    declare_enable_monitor = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='Habilitar el nodo de monitoreo de navegación'
    )
    
    declare_enable_waypoint_manager = DeclareLaunchArgument(
        'enable_waypoint_manager',
        default_value='true',
        description='Habilitar el gestor de waypoints'
    )
    
    declare_enable_mission_executor = DeclareLaunchArgument(
        'enable_mission_executor',
        default_value='true',
        description='Habilitar el ejecutor de misiones'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(tadeo_config_dir, 'maps', 'tadeo_world.yaml'),
        description='Ruta al archivo de mapa'
    )
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    enable_monitor = LaunchConfiguration('enable_monitor')
    enable_waypoint_manager = LaunchConfiguration('enable_waypoint_manager')
    enable_mission_executor = LaunchConfiguration('enable_mission_executor')
    map_file = LaunchConfiguration('map_file')
    
    # Configurar parámetros globales
    stdout_linebuf_envvar = SetParameter(name='RCUTILS_LOGGING_BUFFERED_STREAM', value='1')
    
    # Grupo de nodos de navegación con namespace condicional
    navigation_group = GroupAction([
        # Navigation Controller Node - Nodo principal de control de navegación
        Node(
            package='tadeo_ecar_navigation',
            executable='navigation_controller_node',
            name='navigation_controller_node',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odom', '/odom'),
                ('/scan', '/scan'),
                ('/move_base_simple/goal', '/move_base_simple/goal'),
                ('/mission_path', '/mission_path'),
                ('/navigation_mode', '/navigation_mode'),
                ('/emergency_stop', '/emergency_stop'),
                ('/navigation_status', '/navigation_status'),
                ('/current_goal', '/current_goal'),
                ('/navigation_markers', '/navigation_markers'),
                ('/navigation/controller_health', '/navigation/controller_health'),
                ('/navigate_to_pose', '/navigate_to_pose'),
                ('/follow_path', '/follow_path')
            ]
        ),
        
        # Mission Executor Node - Ejecutor de misiones y control de waypoints
        Node(
            condition=IfCondition(enable_mission_executor),
            package='tadeo_ecar_navigation',
            executable='mission_executor_node',
            name='mission_executor_node',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('/mission_command', '/mission_command'),
                ('/execute_mission_id', '/execute_mission_id'),
                ('/navigation_status', '/navigation_status'),
                ('/mission_path', '/mission_path'),
                ('/mission_status', '/mission_status'),
                ('/mission_progress', '/mission_progress'),
                ('/mission_visualization', '/mission_visualization'),
                ('/navigation/mission_health', '/navigation/mission_health'),
                ('/follow_path', '/follow_path'),
                ('/navigate_to_pose', '/navigate_to_pose')
            ]
        ),
        
        # Waypoint Manager Node - Gestor interactivo de waypoints
        Node(
            condition=IfCondition(enable_waypoint_manager),
            package='tadeo_ecar_navigation',
            executable='waypoint_manager_node',
            name='waypoint_manager_node',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('/waypoint_command', '/waypoint_command'),
                ('/new_waypoint', '/new_waypoint'),
                ('/clicked_point', '/clicked_point'),
                ('/waypoint_list', '/waypoint_list'),
                ('/current_mission', '/current_mission'),
                ('/waypoint_markers', '/waypoint_markers'),
                ('/waypoint_status', '/waypoint_status'),
                ('/navigation/waypoint_health', '/navigation/waypoint_health'),
                ('/add_waypoint_service', '/add_waypoint_service')
            ]
        ),
        
        # Navigation Monitor Node - Monitoreo de rendimiento y seguridad
        Node(
            condition=IfCondition(enable_monitor),
            package='tadeo_ecar_navigation',
            executable='navigation_monitor_node',
            name='navigation_monitor_node',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('/navigation_status', '/navigation_status'),
                ('/mission_status', '/mission_status'),
                ('/odom', '/odom'),
                ('/cmd_vel', '/cmd_vel'),
                ('/scan', '/scan'),
                ('/global_costmap/costmap', '/global_costmap/costmap'),
                ('/current_goal', '/current_goal'),
                ('/navigation_metrics', '/navigation_metrics'),
                ('/safety_alerts', '/safety_alerts'),
                ('/navigation_quality', '/navigation_quality'),
                ('/monitor_visualization', '/monitor_visualization'),
                ('/navigation/monitor_health', '/navigation/monitor_health')
            ]
        )
    ])
    
    # Incluir lanzamiento de Nav2 si está disponible
    nav2_launch_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    ])
    
    nav2_group = GroupAction([
        IncludeLaunchDescription(
            nav2_launch_file,
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'map': map_file
            }.items()
        )
    ])
    
    # Static Transform Publishers para marcos de referencia
    static_transforms_group = GroupAction([
        # Transform del laser al base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_link',
            arguments=['0.32', '0', '0.18', '0', '0', '0', 'base_link', 'laser_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Transform del IMU al base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_link',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Transform de las cámaras al base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_camera_to_base_link',
            arguments=['0.35', '0', '0.25', '0', '0', '0', 'base_link', 'front_camera_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rear_camera_to_base_link',
            arguments=['-0.35', '0', '0.25', '0', '0', '3.14159', 'base_link', 'rear_camera_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
    
    # Map Server para cargar el mapa
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=namespace,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time},
            {'topic_name': 'map'},
            {'frame_id': 'map'}
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # Lifecycle Manager para gestión de nodos del ciclo de vida
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'waypoint_follower'
            ]}
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # RViz para visualización (opcional)
    rviz_config_file = os.path.join(tadeo_navigation_dir, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz', default='false'))
    )
    
    return LaunchDescription([
        # Argumentos de lanzamiento
        declare_use_sim_time,
        declare_params_file,
        declare_namespace,
        declare_use_namespace,
        declare_autostart,
        declare_use_composition,
        declare_use_respawn,
        declare_log_level,
        declare_enable_monitor,
        declare_enable_waypoint_manager,
        declare_enable_mission_executor,
        declare_map_file,
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Lanzar RViz para visualización'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Lanzar el stack completo de Nav2'
        ),
        
        # Configuración global
        stdout_linebuf_envvar,
        
        # Transformadas estáticas
        static_transforms_group,
        
        # Map Server
        map_server_node,
        
        # Lifecycle Manager
        lifecycle_manager_node,
        
        # Nav2 Stack (condicional)
        GroupAction([
            nav2_group
        ], condition=IfCondition(LaunchConfiguration('use_nav2', default='true'))),
        
        # Nodos de navegación Tadeo eCar
        navigation_group,
        
        # RViz (opcional)
        rviz_node
    ])