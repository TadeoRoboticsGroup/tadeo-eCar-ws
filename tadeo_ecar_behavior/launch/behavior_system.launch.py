#!/usr/bin/env python3

"""
Launch file para el sistema completo de comportamientos del Tadeo eCar
Incluye behavior manager, state machine, monitor y árboles de comportamientos
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Rutas de paquetes
    tadeo_behavior_dir = get_package_share_directory('tadeo_ecar_behavior')
    
    # Archivos de configuración
    behavior_params_file = os.path.join(tadeo_behavior_dir, 'config', 'behavior_params.yaml')
    
    # Argumentos de lanzamiento
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulación si está en true'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=behavior_params_file,
        description='Ruta completa al archivo de parámetros de comportamientos'
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
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nivel de logging'
    )
    
    declare_enable_behavior_manager = DeclareLaunchArgument(
        'enable_behavior_manager',
        default_value='true',
        description='Habilitar el gestor de comportamientos con BehaviorTree'
    )
    
    declare_enable_state_machine = DeclareLaunchArgument(
        'enable_state_machine',
        default_value='true',
        description='Habilitar la máquina de estados'
    )
    
    declare_enable_monitor = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='Habilitar el monitor de comportamientos'
    )
    
    declare_behavior_tree_file = DeclareLaunchArgument(
        'behavior_tree_file',
        default_value='main_behavior.xml',
        description='Archivo XML del árbol de comportamientos a cargar'
    )
    
    declare_enable_groot = DeclareLaunchArgument(
        'enable_groot',
        default_value='false',
        description='Habilitar monitoreo con Groot (requiere instalación)'
    )
    
    declare_groot_port = DeclareLaunchArgument(
        'groot_port',
        default_value='1666',
        description='Puerto para publicador ZMQ de Groot'
    )
    
    # Variables de configuración
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    log_level = LaunchConfiguration('log_level')
    enable_behavior_manager = LaunchConfiguration('enable_behavior_manager')
    enable_state_machine = LaunchConfiguration('enable_state_machine')
    enable_monitor = LaunchConfiguration('enable_monitor')
    behavior_tree_file = LaunchConfiguration('behavior_tree_file')
    enable_groot = LaunchConfiguration('enable_groot')
    groot_port = LaunchConfiguration('groot_port')
    
    # Configurar variable de entorno para BehaviorTree
    behavior_trees_path = PathJoinSubstitution([
        FindPackageShare('tadeo_ecar_behavior'),
        'behavior_trees'
    ])
    
    set_bt_path = SetEnvironmentVariable(
        'BEHAVIOR_TREES_PATH',
        behavior_trees_path
    )
    
    # Configurar parámetros globales
    stdout_linebuf_envvar = SetParameter(name='RCUTILS_LOGGING_BUFFERED_STREAM', value='1')
    
    # Grupo de nodos de comportamientos
    behavior_group = GroupAction([
        
        # Behavior Manager Node - Gestor principal con BehaviorTree.CPP
        Node(
            condition=IfCondition(enable_behavior_manager),
            package='tadeo_ecar_behavior',
            executable='behavior_manager_node',
            name='behavior_manager_node',
            namespace=namespace,
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'default_tree_file': behavior_tree_file,
                    'enable_groot_monitoring': enable_groot,
                    'groot_publisher_port': groot_port
                }
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('/behavior_command', '/behavior_command'),
                ('/odom', '/odom'),
                ('/battery_state', '/battery_state'),
                ('/emergency_stop', '/emergency_stop'),
                ('/navigation_status', '/navigation_status'),
                ('/system_health', '/system_health'),
                ('/behavior_status', '/behavior_status'),
                ('/robot_state', '/robot_state'),
                ('/behavior_metrics', '/behavior_metrics'),
                ('/behavior_visualization', '/behavior_visualization'),
                ('/behavior/manager_health', '/behavior/manager_health'),
                ('/execute_behavior', '/execute_behavior')
            ]
        ),
        
        # State Machine Node - Máquina de estados del robot
        Node(
            condition=IfCondition(enable_state_machine),
            package='tadeo_ecar_behavior',
            executable='state_machine_node',
            name='state_machine_node',
            namespace=namespace,
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('/emergency_stop', '/emergency_stop'),
                ('/battery_state', '/battery_state'),
                ('/state_command', '/state_command'),
                ('/navigation_status', '/navigation_status'),
                ('/mission_status', '/mission_status'),
                ('/robot_state', '/robot_state'),
                ('/state_transitions', '/state_transitions'),
                ('/behavior/state_health', '/behavior/state_health')
            ]
        ),
        
        # Behavior Monitor Node - Monitor de rendimiento y diagnóstico
        Node(
            condition=IfCondition(enable_monitor),
            package='tadeo_ecar_behavior',
            executable='behavior_monitor_node',
            name='behavior_monitor_node',
            namespace=namespace,
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time}
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                ('/behavior_status', '/behavior_status'),
                ('/robot_state', '/robot_state'),
                ('/behavior_metrics', '/behavior_metrics'),
                ('/system_alerts', '/system_alerts'),
                ('/behavior_performance', '/behavior_performance'),
                ('/behavior_diagnostics', '/behavior_diagnostics'),
                ('/behavior_monitor_viz', '/behavior_monitor_viz'),
                ('/behavior/monitor_health', '/behavior/monitor_health')
            ]
        )
        
    ])
    
    # Nodos de utilidad para testing y desarrollo
    utility_group = GroupAction([
        
        # Publicador de comandos de comportamiento (para testing)
        Node(
            package='std_msgs',
            executable='publisher',
            name='behavior_command_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                'behavior_command',
                'std_msgs/String',
                'data: "START"',
                '--once'
            ],
            condition=IfCondition(LaunchConfiguration('test_mode', default='false'))
        ),
        
        # Monitor de tópicos de comportamientos
        Node(
            package='topic_tools',
            executable='echo',
            name='behavior_status_echo',
            namespace=namespace,
            arguments=['/behavior_status'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('echo_topics', default='false'))
        )
        
    ])
    
    # Herramientas de desarrollo (Groot, RQT, etc.)
    development_tools = GroupAction([
        
        # RQT para monitoreo gráfico
        Node(
            package='rqt_topic',
            executable='rqt_topic',
            name='rqt_behavior_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rqt', default='false'))
        ),
        
        # RQT Graph para visualizar conexiones
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph_behavior',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rqt_graph', default='false'))
        )
        
    ])
    
    # Configuración de logging avanzado
    logging_config = GroupAction([
        
        # Logger de comportamientos
        Node(
            package='rosbag2_transport',
            executable='record',
            name='behavior_logger',
            output='screen',
            arguments=[
                '--topics',
                '/behavior_status',
                '/robot_state', 
                '/behavior_metrics',
                '/system_alerts',
                '/state_transitions',
                '--output', '/tmp/tadeo_behavior_logs'
            ],
            condition=IfCondition(LaunchConfiguration('record_behaviors', default='false'))
        )
        
    ])
    
    return LaunchDescription([
        # Argumentos de lanzamiento
        declare_use_sim_time,
        declare_params_file,
        declare_namespace,
        declare_use_namespace,
        declare_log_level,
        declare_enable_behavior_manager,
        declare_enable_state_machine,
        declare_enable_monitor,
        declare_behavior_tree_file,
        declare_enable_groot,
        declare_groot_port,
        
        # Argumentos adicionales opcionales
        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='Ejecutar en modo de prueba con comandos automáticos'
        ),
        DeclareLaunchArgument(
            'echo_topics',
            default_value='false',
            description='Hacer echo de tópicos principales para debugging'
        ),
        DeclareLaunchArgument(
            'use_rqt',
            default_value='false',
            description='Lanzar herramientas RQT para monitoreo'
        ),
        DeclareLaunchArgument(
            'use_rqt_graph',
            default_value='false',
            description='Lanzar RQT Graph para visualizar conexiones'
        ),
        DeclareLaunchArgument(
            'record_behaviors',
            default_value='false',
            description='Grabar tópicos de comportamientos con rosbag'
        ),
        
        # Variables de entorno
        set_bt_path,
        
        # Configuración global
        stdout_linebuf_envvar,
        
        # Grupos de nodos
        behavior_group,
        utility_group,
        development_tools,
        logging_config
    ])