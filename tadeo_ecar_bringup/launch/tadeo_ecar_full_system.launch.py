#!/usr/bin/env python3

"""
Launch file principal para el sistema completo del Tadeo eCar 4WD4WS
Incluye todos los subsistemas necesarios para operación autónoma
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener directorios de paquetes
    bringup_dir = get_package_share_directory('tadeo_ecar_bringup')
    description_dir = get_package_share_directory('tadeo_ecar_description')
    config_dir = get_package_share_directory('tadeo_ecar_config')
    
    # Archivos de configuración
    robot_params_file = os.path.join(config_dir, 'config', 'robot_params.yaml')
    nav2_params_file = os.path.join(config_dir, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'tadeo_ecar_full_system.rviz')
    
    # Argumentos de lanzamiento
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
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulación'
    )
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='tadeo_ecar',
        description='Nombre del robot'
    )
    
    declare_use_simulation = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Lanzar en modo simulación'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Lanzar RViz para visualización'
    )
    
    declare_use_foxglove = DeclareLaunchArgument(
        'use_foxglove',
        default_value='false',
        description='Lanzar Foxglove Bridge para visualización web'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nivel de logging'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart de lifecycle managers'
    )
    
    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Usar composición de nodos para mejor rendimiento'
    )
    
    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='none',
        choices=['none', 'slam_toolbox', 'cartographer'],
        description='Modo SLAM: none, slam_toolbox, cartographer'
    )
    
    declare_navigation_mode = DeclareLaunchArgument(
        'navigation_mode',
        default_value='full',
        choices=['full', 'localization_only', 'planning_only', 'none'],
        description='Modo de navegación'
    )
    
    declare_behavior_mode = DeclareLaunchArgument(
        'behavior_mode',
        default_value='full',
        choices=['full', 'state_machine_only', 'none'],
        description='Modo de comportamientos'
    )
    
    declare_safety_mode = DeclareLaunchArgument(
        'safety_mode',
        default_value='full',
        choices=['full', 'emergency_only', 'none'],
        description='Modo de seguridad'
    )
    
    # Variables de configuración
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    use_simulation = LaunchConfiguration('use_simulation')
    use_rviz = LaunchConfiguration('use_rviz')
    use_foxglove = LaunchConfiguration('use_foxglove')
    log_level = LaunchConfiguration('log_level')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    slam_mode = LaunchConfiguration('slam_mode')
    navigation_mode = LaunchConfiguration('navigation_mode')
    behavior_mode = LaunchConfiguration('behavior_mode')
    safety_mode = LaunchConfiguration('safety_mode')
    
    # Configuración de entorno
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # Parámetros globales
    global_params = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # Grupo de Robot Description y TF
    robot_description_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_description'),
                    'launch',
                    'robot_state_publisher.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_name': robot_name,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level
            }.items()
        )
    ])
    
    # Grupo de Simulación (solo si está habilitado)
    simulation_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_description'),
                    'launch',
                    'simulation.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'robot_name': robot_name,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level
            }.items()
        )
    ], condition=IfCondition(use_simulation))
    
    # Grupo de Sensores y Percepción
    perception_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_perception'),
                    'launch',
                    'perception_system.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level,
                'use_simulation': use_simulation
            }.items()
        )
    ])
    
    # Grupo de Control
    control_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_control'),
                    'launch',
                    'control_system.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level,
                'use_simulation': use_simulation
            }.items()
        )
    ])
    
    # Grupo de Seguridad
    safety_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_safety'),
                    'launch',
                    'safety_system.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level,
                'safety_mode': safety_mode
            }.items()
        )
    ], condition=UnlessCondition(LaunchConfiguration('safety_mode', default='none')))
    
    # Grupo de Localización
    localization_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_localization'),
                    'launch',
                    'localization_system.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level,
                'use_simulation': use_simulation
            }.items()
        )
    ])
    
    # Grupo de SLAM (solo si está habilitado)
    slam_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_slam'),
                    'launch',
                    'slam_system.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level,
                'slam_mode': slam_mode
            }.items()
        )
    ], condition=UnlessCondition(LaunchConfiguration('slam_mode', default='none')))
    
    # Grupo de Planificación
    planning_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_planning'),
                    'launch',
                    'planning_system.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level
            }.items()
        )
    ])
    
    # Grupo de Navegación
    navigation_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_navigation'),
                    'launch',
                    'navigation_system.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level,
                'navigation_mode': navigation_mode,
                'autostart': autostart
            }.items()
        )
    ], condition=UnlessCondition(LaunchConfiguration('navigation_mode', default='none')))
    
    # Grupo de Comportamientos
    behavior_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_behavior'),
                    'launch',
                    'behavior_system.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'log_level': log_level,
                'behavior_mode': behavior_mode
            }.items()
        )
    ], condition=UnlessCondition(LaunchConfiguration('behavior_mode', default='none')))
    
    # Grupo de Visualización
    visualization_group = GroupAction([
        # RViz
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=namespace,
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # Foxglove Bridge
        Node(
            condition=IfCondition(use_foxglove),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            namespace=namespace,
            parameters=[
                {'port': 8765},
                {'address': '0.0.0.0'},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    ])
    
    # Grupo de Monitoreo y Diagnóstico
    monitoring_group = GroupAction([
        # Monitor de sistema
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            namespace=namespace,
            parameters=[robot_params_file],
            output='screen'
        ),
        
        # Monitor de TF
        Node(
            package='tf2_ros',
            executable='tf2_monitor',
            name='tf2_monitor',
            namespace=namespace,
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
    
    return LaunchDescription([
        # Argumentos de lanzamiento
        declare_namespace,
        declare_use_namespace,
        declare_use_sim_time,
        declare_robot_name,
        declare_use_simulation,
        declare_use_rviz,
        declare_use_foxglove,
        declare_log_level,
        declare_autostart,
        declare_use_composition,
        declare_slam_mode,
        declare_navigation_mode,
        declare_behavior_mode,
        declare_safety_mode,
        
        # Configuración de entorno
        stdout_linebuf_envvar,
        global_params,
        
        # Grupos de sistemas
        robot_description_group,
        simulation_group,
        perception_group,
        control_group,
        safety_group,
        localization_group,
        slam_group,
        planning_group,
        navigation_group,
        behavior_group,
        visualization_group,
        monitoring_group
    ])