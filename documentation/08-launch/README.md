# Capítulo 8: Launch Files en ROS2

## Tabla de Contenidos

1. [Concepto de Launch Files](#concepto-de-launch-files)
2. [Sintaxis y Estructura](#sintaxis-y-estructura)
3. [Argumentos y Condiciones](#argumentos-y-condiciones)
4. [Composición y Namespaces](#composición-y-namespaces)
5. [Integración con Parámetros](#integración-con-parámetros)
6. [Launch Files del eCar](#launch-files-del-ecar)
7. [Debugging y Monitoreo](#debugging-y-monitoreo)
8. [Launch Avanzado](#launch-avanzado)

## Concepto de Launch Files

### ¿Qué son los Launch Files?

Los launch files en ROS2 son scripts en Python que permiten iniciar múltiples nodos, configurar parámetros, y orquestar sistemas robóticos complejos de manera declarativa y reproducible.

```python
# Concepto básico: Un launch file orquesta todo el sistema
def generate_launch_description():
    return LaunchDescription([
        # Sensores
        Node(package='tadeo_ecar_perception', executable='lidar_node'),
        Node(package='tadeo_ecar_perception', executable='camera_node'),
        
        # Control
        Node(package='tadeo_ecar_control', executable='wheel_controller'),
        
        # Navegación
        Node(package='tadeo_ecar_navigation', executable='navigation_node'),
        
        # Seguridad
        Node(package='tadeo_ecar_safety', executable='safety_monitor')
    ])
```

### Evolución desde ROS1

```xml
<!-- ROS1: XML estático -->
<launch>
  <node pkg="package" type="executable" name="node_name"/>
  <param name="param" value="value"/>
</launch>
```

```python
# ROS2: Python dinámico
def generate_launch_description():
    # Lógica condicional
    if use_simulation:
        nodes = simulation_nodes()
    else:
        nodes = hardware_nodes()
    
    return LaunchDescription(nodes)
```

### Ventajas de los Launch Files

**1. Orquestación Completa**
```bash
# Un comando inicia todo el sistema eCar
ros2 launch tadeo_ecar_bringup ecar_full_system.launch.py
```

**2. Configuración Flexible**
```bash
# Diferentes configuraciones con argumentos
ros2 launch tadeo_ecar_bringup ecar_system.launch.py use_sim:=true environment:=outdoor
```

**3. Reproducibilidad**
```python
# Siempre la misma configuración
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='ecar_001'),
        # Configuración determinística...
    ])
```

**4. Debugging Simplificado**
```python
# Lanzar solo subsistemas específicos
def generate_launch_description():
    if debug_mode:
        return LaunchDescription([perception_nodes_only()])
    else:
        return LaunchDescription([full_system_nodes()])
```

## Sintaxis y Estructura

### Estructura Básica

```python
# launch/basic_example.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Función principal que debe existir en todo launch file.
    Retorna LaunchDescription con todas las acciones.
    """
    return LaunchDescription([
        # Lista de acciones (nodos, argumentos, includes, etc.)
        Node(
            package='tadeo_ecar_control',
            executable='wheel_controller_node',
            name='wheel_controller',
            output='screen'
        ),
        
        Node(
            package='tadeo_ecar_perception',
            executable='lidar_processor_node',
            name='lidar_processor',
            output='screen'
        )
    ])
```

### Imports Esenciales

```python
# Imports básicos para launch files
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace
```

### Nodos Básicos

```python
def generate_launch_description():
    return LaunchDescription([
        # Nodo básico
        Node(
            package='tadeo_ecar_control',
            executable='wheel_controller_node',
            name='wheel_controller_node',
            output='screen',  # 'screen', 'log', o None
            emulate_tty=True,  # Para colores en terminal
        ),
        
        # Nodo con namespace
        Node(
            package='tadeo_ecar_perception',
            executable='camera_node',
            name='camera_node',
            namespace='sensors',  # Resultado: /sensors/camera_node
            output='screen'
        ),
        
        # Nodo con parámetros
        Node(
            package='tadeo_ecar_navigation',
            executable='navigation_node',
            name='navigation_node',
            parameters=[{
                'max_velocity': 2.0,
                'use_sim_time': False
            }],
            output='screen'
        ),
        
        # Nodo con archivo de parámetros
        Node(
            package='tadeo_ecar_safety',
            executable='safety_monitor_node',
            name='safety_monitor',
            parameters=[PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_config'),
                'config', 'safety_config.yaml'
            ])],
            output='screen'
        )
    ])
```

### Procesos Externos

```python
def generate_launch_description():
    return LaunchDescription([
        # Ejecutar comando del sistema
        ExecuteProcess(
            cmd=['rviz2', '-d', PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_config'),
                'rviz', 'ecar_visualization.rviz'
            ])],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
        
        # Ejecutar script personalizado
        ExecuteProcess(
            cmd=[
                FindExecutable(name='python3'),
                PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_utils'),
                    'scripts', 'system_check.py'
                ])
            ],
            output='screen',
            name='system_check'
        )
    ])
```

## Argumentos y Condiciones

### Declaración de Argumentos

```python
def generate_launch_description():
    # Argumentos de launch
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware',
        choices=['true', 'false']
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='tadeo_ecar',
        description='Name of the robot'
    )
    
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='indoor',
        description='Operating environment',
        choices=['indoor', 'outdoor', 'warehouse']
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to custom configuration file'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level',
        choices=['DEBUG', 'INFO', 'WARN', 'ERROR']
    )
    
    return LaunchDescription([
        use_sim_arg,
        robot_name_arg,
        environment_arg,
        config_file_arg,
        log_level_arg,
        
        # Usar argumentos en nodos...
    ])
```

### Uso de Argumentos

```python
def generate_launch_description():
    # Argumentos
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='tadeo_ecar')
    
    # Rutas dinámicas basadas en argumentos
    config_path = PathJoinSubstitution([
        FindPackageShare('tadeo_ecar_config'),
        'config',
        'environments',
        [LaunchConfiguration('environment'), '.yaml']
    ])
    
    # Nodo que usa argumentos
    wheel_controller = Node(
        package='tadeo_ecar_control',
        executable='wheel_controller_node',
        name=[LaunchConfiguration('robot_name'), '_wheel_controller'],
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            config_path,
            {
                'robot_name': LaunchConfiguration('robot_name'),
                'use_sim_time': LaunchConfiguration('use_sim')
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_arg,
        robot_name_arg,
        wheel_controller
    ])
```

### Condiciones

```python
def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_hardware_arg = DeclareLaunchArgument('use_hardware', default_value='true')
    
    return LaunchDescription([
        use_sim_arg,
        use_rviz_arg,
        use_hardware_arg,
        
        # Nodos solo en simulación
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_ecar',
            arguments=['-entity', 'tadeo_ecar', '-topic', 'robot_description'],
            condition=IfCondition(LaunchConfiguration('use_sim')),
            output='screen'
        ),
        
        # Nodos solo con hardware real
        Node(
            package='tadeo_ecar_hardware',
            executable='hardware_interface_node',
            name='hardware_interface',
            condition=UnlessCondition(LaunchConfiguration('use_sim')),
            output='screen'
        ),
        
        # RViz opcional
        ExecuteProcess(
            cmd=['rviz2', '-d', PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_config'),
                'rviz', 'ecar_navigation.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen'
        ),
        
        # Nodos con condiciones complejas
        Node(
            package='tadeo_ecar_simulation',
            executable='fake_sensor_node',
            name='fake_lidar',
            condition=IfCondition(LaunchConfiguration('use_sim')),
            output='screen'
        )
    ])
```

### Funciones Opacas para Lógica Compleja

```python
def generate_ecar_nodes(context, *args, **kwargs):
    """
    Función opaca que permite lógica Python compleja
    basada en el contexto de launch.
    """
    # Obtener valores de argumentos
    use_sim = LaunchConfiguration('use_sim').perform(context)
    environment = LaunchConfiguration('environment').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    
    nodes = []
    
    # Lógica condicional compleja
    if use_sim == 'true':
        # Configuración de simulación
        if environment == 'outdoor':
            nodes.extend([
                Node(
                    package='tadeo_ecar_simulation',
                    executable='outdoor_physics_node',
                    name='physics_simulator',
                    parameters=[{'wind_enabled': True, 'terrain_roughness': 0.1}]
                ),
                Node(
                    package='tadeo_ecar_simulation',
                    executable='weather_simulator_node',
                    name='weather_simulator'
                )
            ])
        else:  # indoor
            nodes.append(Node(
                package='tadeo_ecar_simulation',
                executable='indoor_physics_node',
                name='physics_simulator',
                parameters=[{'friction_coefficient': 0.8}]
            ))
    else:
        # Configuración de hardware real
        if environment == 'outdoor':
            nodes.extend([
                Node(
                    package='tadeo_ecar_hardware',
                    executable='gps_driver_node',
                    name='gps_driver'
                ),
                Node(
                    package='tadeo_ecar_hardware',
                    executable='compass_driver_node',
                    name='compass_driver'
                )
            ])
        
        # Hardware común
        nodes.extend([
            Node(
                package='tadeo_ecar_hardware',
                executable='lidar_driver_node',
                name='lidar_driver',
                parameters=[{'device_port': '/dev/ttyUSB0'}]
            ),
            Node(
                package='tadeo_ecar_hardware',
                executable='motor_driver_node',
                name='motor_driver',
                parameters=[{'can_interface': 'can0'}]
            )
        ])
    
    # Nodos comunes independientes del modo
    common_nodes = [
        Node(
            package='tadeo_ecar_control',
            executable='wheel_controller_node',
            name=f'{robot_name}_wheel_controller',
            namespace=robot_name,
            parameters=[{
                'robot_name': robot_name,
                'use_sim_time': use_sim == 'true'
            }]
        ),
        Node(
            package='tadeo_ecar_safety',
            executable='safety_monitor_node',
            name=f'{robot_name}_safety_monitor',
            namespace=robot_name
        )
    ]
    
    nodes.extend(common_nodes)
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('environment', default_value='indoor'),
        DeclareLaunchArgument('robot_name', default_value='tadeo_ecar'),
        
        # Usar función opaca para lógica compleja
        OpaqueFunction(function=generate_ecar_nodes)
    ])
```

## Composición y Namespaces

### Inclusión de Launch Files

```python
# launch/ecar_system.launch.py - Launch principal
def generate_launch_description():
    
    # Argumentos globales
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='tadeo_ecar')
    
    # Incluir launch file de sensores
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_perception'),
                'launch', 'sensors.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'robot_name': LaunchConfiguration('robot_name'),
            'publish_tf': 'true'
        }.items()
    )
    
    # Incluir launch file de control
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_control'),
                'launch', 'control_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'robot_name': LaunchConfiguration('robot_name')
        }.items()
    )
    
    # Incluir launch file de navegación
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_navigation'),
                'launch', 'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'robot_name': LaunchConfiguration('robot_name'),
            'map_file': PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_maps'),
                'maps', 'default_map.yaml'
            ])
        }.items()
    )
    
    return LaunchDescription([
        use_sim_arg,
        robot_name_arg,
        
        # Lanzar subsistemas en orden
        sensors_launch,
        
        # Esperar antes de iniciar control
        TimerAction(
            period=2.0,
            actions=[control_launch]
        ),
        
        # Esperar antes de iniciar navegación
        TimerAction(
            period=5.0,
            actions=[navigation_launch]
        )
    ])
```

### Uso de Namespaces

```python
def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='tadeo_ecar'),
        
        # Grupo con namespace común
        GroupAction([
            PushRosNamespace(robot_name),
            
            # Todos estos nodos estarán en el namespace del robot
            Node(
                package='tadeo_ecar_perception',
                executable='lidar_processor_node',
                name='lidar_processor',  # Resultado: /tadeo_ecar/lidar_processor
                output='screen'
            ),
            
            Node(
                package='tadeo_ecar_control',
                executable='wheel_controller_node',
                name='wheel_controller',  # Resultado: /tadeo_ecar/wheel_controller
                output='screen'
            ),
            
            # Subgrupo con namespace adicional
            GroupAction([
                PushRosNamespace('sensors'),
                
                Node(
                    package='tadeo_ecar_perception',
                    executable='camera_node',
                    name='camera',  # Resultado: /tadeo_ecar/sensors/camera
                    output='screen'
                ),
                
                Node(
                    package='tadeo_ecar_perception',
                    executable='imu_node',
                    name='imu',  # Resultado: /tadeo_ecar/sensors/imu
                    output='screen'
                )
            ])
        ])
    ])
```

### Múltiples Robots

```python
def create_robot_nodes(robot_name, robot_x, robot_y):
    """Crear nodos para un robot específico"""
    return GroupAction([
        PushRosNamespace(robot_name),
        
        Node(
            package='tadeo_ecar_control',
            executable='wheel_controller_node',
            name='wheel_controller',
            parameters=[{
                'robot_name': robot_name,
                'initial_pose_x': robot_x,
                'initial_pose_y': robot_y
            }],
            output='screen'
        ),
        
        Node(
            package='tadeo_ecar_perception',
            executable='lidar_processor_node',
            name='lidar_processor',
            remappings=[
                ('scan', f'/{robot_name}/scan'),
                ('scan_filtered', f'/{robot_name}/scan_filtered')
            ],
            output='screen'
        ),
        
        Node(
            package='tadeo_ecar_safety',
            executable='safety_monitor_node',
            name='safety_monitor',
            parameters=[{
                'robot_name': robot_name
            }],
            output='screen'
        )
    ])

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='2'),
        
        # Robot 1
        create_robot_nodes('ecar_1', 0.0, 0.0),
        
        # Robot 2
        create_robot_nodes('ecar_2', 2.0, 0.0),
        
        # Robot 3 (condicional)
        GroupAction([
            create_robot_nodes('ecar_3', 4.0, 0.0)
        ], condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('num_robots'), "' >= '3'"])
        )),
        
        # Coordinador multi-robot
        Node(
            package='tadeo_ecar_coordination',
            executable='fleet_coordinator_node',
            name='fleet_coordinator',
            parameters=[{
                'num_robots': LaunchConfiguration('num_robots'),
                'robot_names': ['ecar_1', 'ecar_2', 'ecar_3']
            }],
            output='screen'
        )
    ])
```

## Integración con Parámetros

### Carga de Archivos de Parámetros

```python
def generate_launch_description():
    
    # Rutas de configuración
    package_share = FindPackageShare('tadeo_ecar_config')
    
    # Configuración base
    base_config = PathJoinSubstitution([
        package_share, 'config', 'ecar_base_config.yaml'
    ])
    
    # Configuración por entorno
    env_config = PathJoinSubstitution([
        package_share, 'config', 'environments',
        [LaunchConfiguration('environment'), '.yaml']
    ])
    
    # Configuración de hardware/simulación
    hw_config = PathJoinSubstitution([
        package_share, 'config',
        [LaunchConfiguration('use_sim'), '_hardware.yaml'] if LaunchConfiguration('use_sim') == 'true' 
        else 'real_hardware.yaml'
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='indoor'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        
        # Nodo con múltiples archivos de parámetros
        Node(
            package='tadeo_ecar_control',
            executable='wheel_controller_node',
            name='wheel_controller',
            parameters=[
                base_config,      # Parámetros base
                env_config,       # Sobrescribir con parámetros del entorno
                hw_config,        # Sobrescribir con parámetros de hardware
                {                 # Parámetros finales (máxima prioridad)
                    'use_sim_time': LaunchConfiguration('use_sim'),
                    'debug_mode': True
                }
            ],
            output='screen'
        )
    ])
```

### Generación Dinámica de Parámetros

```python
def generate_dynamic_config(context, *args, **kwargs):
    """Generar configuración dinámica basada en argumentos"""
    
    # Obtener valores de argumentos
    robot_name = LaunchConfiguration('robot_name').perform(context)
    environment = LaunchConfiguration('environment').perform(context)
    num_wheels = int(LaunchConfiguration('num_wheels').perform(context))
    
    # Generar configuración dinámica
    config = {
        'robot_name': robot_name,
        'environment': environment,
        'wheel_count': num_wheels,
        'wheel_names': [f'wheel_{i}' for i in range(num_wheels)],
        'wheel_positions': [i * 0.3 for i in range(num_wheels)]  # Separación de 30cm
    }
    
    # Configuración específica por entorno
    if environment == 'outdoor':
        config.update({
            'max_velocity': 3.0,
            'safety_distance': 1.5,
            'use_gps': True
        })
    elif environment == 'indoor':
        config.update({
            'max_velocity': 1.0,
            'safety_distance': 0.5,
            'use_gps': False
        })
    elif environment == 'warehouse':
        config.update({
            'max_velocity': 0.8,
            'safety_distance': 0.3,
            'precision_mode': True
        })
    
    # Configuración específica para robots especiales
    if 'heavy' in robot_name:
        config.update({
            'max_payload': 100.0,  # kg
            'acceleration_limit': 0.5  # m/s²
        })
    
    return [Node(
        package='tadeo_ecar_control',
        executable='wheel_controller_node',
        name='wheel_controller',
        parameters=[config],
        output='screen'
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='tadeo_ecar'),
        DeclareLaunchArgument('environment', default_value='indoor'),
        DeclareLaunchArgument('num_wheels', default_value='4'),
        
        OpaqueFunction(function=generate_dynamic_config)
    ])
```

## Launch Files del eCar

### Launch Principal del Sistema

```python
# launch/ecar_full_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    OpaqueFunction,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    
    # =================== ARGUMENTOS ===================
    
    # Configuración del robot
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', 
        default_value='tadeo_ecar',
        description='Name of the robot instance'
    )
    
    # Modo de operación
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware',
        choices=['true', 'false']
    )
    
    # Entorno de operación
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='indoor',
        description='Operating environment configuration',
        choices=['indoor', 'outdoor', 'warehouse', 'testing']
    )
    
    # Subsistemas opcionales
    use_navigation_arg = DeclareLaunchArgument(
        'use_navigation',
        default_value='true',
        description='Enable navigation subsystem'
    )
    
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Enable SLAM instead of localization'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='false',
        description='Launch Foxglove bridge for remote visualization'
    )
    
    # Configuración personalizada
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to custom configuration file'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map file for localization'
    )
    
    # Debug y logging
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Logging level for all nodes',
        choices=['DEBUG', 'INFO', 'WARN', 'ERROR']
    )
    
    # =================== RUTAS DE CONFIGURACIÓN ===================
    
    ecar_config_share = FindPackageShare('tadeo_ecar_config')
    
    # Configuración base
    base_config = PathJoinSubstitution([
        ecar_config_share, 'config', 'ecar_base_config.yaml'
    ])
    
    # Configuración por entorno
    environment_config = PathJoinSubstitution([
        ecar_config_share, 'config', 'environments',
        [LaunchConfiguration('environment'), '.yaml']
    ])
    
    # Configuración de hardware vs simulación
    hardware_config = PathJoinSubstitution([
        ecar_config_share, 'config',
        'simulation.yaml' if LaunchConfiguration('use_sim') == 'true' else 'hardware.yaml'
    ])
    
    # =================== SUBSISTEMAS ===================
    
    # Hardware/Simulación
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_hardware'),
                'launch', 'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'robot_name': LaunchConfiguration('robot_name'),
            'config_file': hardware_config
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_sim'))
    )
    
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_simulation'),
                'launch', 'simulation.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'environment': LaunchConfiguration('environment'),
            'world_file': PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_simulation'),
                'worlds', [LaunchConfiguration('environment'), '.world']
            ])
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )
    
    # Sistema de control
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_control'),
                'launch', 'control_system.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim': LaunchConfiguration('use_sim'),
            'config_file': base_config
        }.items()
    )
    
    # Percepción
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_perception'),
                'launch', 'perception.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim': LaunchConfiguration('use_sim'),
            'environment': LaunchConfiguration('environment')
        }.items()
    )
    
    # Localización o SLAM
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_localization'),
                'launch', 'localization.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'map_file': LaunchConfiguration('map_file')
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_slam'))
    )
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_slam'),
                'launch', 'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim': LaunchConfiguration('use_sim')
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )
    
    # Navegación
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_navigation'),
                'launch', 'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_sim': LaunchConfiguration('use_sim'),
            'environment': LaunchConfiguration('environment')
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_navigation'))
    )
    
    # Comportamientos
    behavior_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_behavior'),
                'launch', 'behavior_system.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_navigation': LaunchConfiguration('use_navigation')
        }.items()
    )
    
    # Seguridad
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_safety'),
                'launch', 'safety_system.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'environment': LaunchConfiguration('environment')
        }.items()
    )
    
    # =================== HERRAMIENTAS DE VISUALIZACIÓN ===================
    
    # RViz
    rviz_launch = ExecuteProcess(
        cmd=['rviz2', '-d', PathJoinSubstitution([
            ecar_config_share, 'rviz', 'ecar_full_system.rviz'
        ])],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Foxglove Bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': [
                '.*'  # Permitir todos los tópicos
            ]
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_foxglove'))
    )
    
    # =================== SISTEMA COMPLETO ===================
    
    return LaunchDescription([
        # Argumentos
        robot_name_arg,
        use_sim_arg,
        environment_arg,
        use_navigation_arg,
        use_slam_arg,
        use_rviz_arg,
        use_foxglove_arg,
        config_file_arg,
        map_file_arg,
        log_level_arg,
        
        # Inicialización del sistema en fases
        
        # Fase 1: Hardware/Simulación
        GroupAction([
            hardware_launch,
            simulation_launch
        ]),
        
        # Fase 2: Control y Percepción (después de 2 segundos)
        TimerAction(
            period=2.0,
            actions=[
                control_launch,
                perception_launch
            ]
        ),
        
        # Fase 3: Localización/SLAM (después de 5 segundos)
        TimerAction(
            period=5.0,
            actions=[
                localization_launch,
                slam_launch
            ]
        ),
        
        # Fase 4: Navegación y Comportamientos (después de 8 segundos)
        TimerAction(
            period=8.0,
            actions=[
                navigation_launch,
                behavior_launch
            ]
        ),
        
        # Fase 5: Seguridad (siempre activa, después de 1 segundo)
        TimerAction(
            period=1.0,
            actions=[safety_launch]
        ),
        
        # Herramientas de visualización (inmediatas)
        rviz_launch,
        foxglove_bridge,
        
        # Mensaje de confirmación
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'sleep 10 && echo "eCar system fully initialized and ready!"'
            ],
            output='screen'
        )
    ])
```

### Launch de Subsistema: Percepción

```python
# launch/perception.launch.py
def generate_launch_description():
    
    # Argumentos
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='tadeo_ecar')
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')
    environment_arg = DeclareLaunchArgument('environment', default_value='indoor')
    
    # Configuración
    config_path = PathJoinSubstitution([
        FindPackageShare('tadeo_ecar_config'),
        'config', 'perception_config.yaml'
    ])
    
    return LaunchDescription([
        robot_name_arg,
        use_sim_arg,
        environment_arg,
        
        # Grupo de percepción con namespace
        GroupAction([
            PushRosNamespace([LaunchConfiguration('robot_name'), '/sensors']),
            
            # LiDAR
            Node(
                package='tadeo_ecar_perception',
                executable='lidar_processor_node',
                name='lidar_processor',
                parameters=[
                    config_path,
                    {
                        'frame_id': 'laser_frame',
                        'use_sim_time': LaunchConfiguration('use_sim')
                    }
                ],
                remappings=[
                    ('scan_raw', '/scan'),
                    ('scan_filtered', 'lidar/scan_filtered'),
                    ('obstacles', 'lidar/obstacles')
                ],
                output='screen'
            ),
            
            # Cámara (solo si no es simulación o si es explícitamente solicitada)
            Node(
                package='tadeo_ecar_perception',
                executable='camera_processor_node',
                name='camera_processor',
                parameters=[
                    config_path,
                    {
                        'camera_index': 0,
                        'frame_id': 'camera_frame',
                        'use_sim_time': LaunchConfiguration('use_sim')
                    }
                ],
                remappings=[
                    ('image_raw', 'camera/image_raw'),
                    ('camera_info', 'camera/camera_info'),
                    ('image_processed', 'camera/image_processed')
                ],
                condition=UnlessCondition(LaunchConfiguration('use_sim')),
                output='screen'
            ),
            
            # IMU
            Node(
                package='tadeo_ecar_perception',
                executable='imu_processor_node',
                name='imu_processor',
                parameters=[
                    config_path,
                    {
                        'frame_id': 'imu_frame',
                        'use_sim_time': LaunchConfiguration('use_sim')
                    }
                ],
                remappings=[
                    ('imu_raw', '/imu/data_raw'),
                    ('imu_filtered', 'imu/data_filtered')
                ],
                output='screen'
            ),
            
            # Fusión de sensores
            Node(
                package='tadeo_ecar_perception',
                executable='sensor_fusion_node',
                name='sensor_fusion',
                parameters=[
                    config_path,
                    {
                        'sensors': ['lidar', 'camera', 'imu'],
                        'fusion_rate': 20.0,
                        'use_sim_time': LaunchConfiguration('use_sim')
                    }
                ],
                remappings=[
                    ('fused_obstacles', 'perception/obstacles'),
                    ('perception_status', 'perception/status')
                ],
                output='screen'
            )
        ]),
        
        # TF estático para sensores
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '0.3', '0.0', '0.2',  # x, y, z
                '0.0', '0.0', '0.0',  # roll, pitch, yaw
                'base_link', 'laser_frame'
            ],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '0.4', '0.0', '0.3',
                '0.0', '0.0', '0.0',
                'base_link', 'camera_frame'
            ],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=[
                '0.0', '0.0', '0.1',
                '0.0', '0.0', '0.0',
                'base_link', 'imu_frame'
            ],
            output='screen'
        )
    ])
```

### Launch de Testing y Debugging

```python
# launch/debug_system.launch.py
def generate_launch_description():
    
    # Argumentos de debug
    subsystem_arg = DeclareLaunchArgument(
        'subsystem',
        default_value='all',
        description='Subsystem to debug',
        choices=['all', 'control', 'perception', 'navigation', 'safety']
    )
    
    debug_level_arg = DeclareLaunchArgument(
        'debug_level',
        default_value='DEBUG',
        description='Debug level',
        choices=['DEBUG', 'INFO', 'WARN']
    )
    
    record_data_arg = DeclareLaunchArgument(
        'record_data',
        default_value='false',
        description='Record rosbag for debugging'
    )
    
    # Función para nodos de debug según subsistema
    def debug_nodes(context, *args, **kwargs):
        subsystem = LaunchConfiguration('subsystem').perform(context)
        debug_level = LaunchConfiguration('debug_level').perform(context)
        
        nodes = []
        
        if subsystem in ['all', 'control']:
            nodes.extend([
                Node(
                    package='tadeo_ecar_control',
                    executable='wheel_controller_node',
                    name='wheel_controller_debug',
                    parameters=[{
                        'debug_mode': True,
                        'publish_debug_info': True
                    }],
                    arguments=['--ros-args', '--log-level', debug_level],
                    output='screen'
                ),
                Node(
                    package='rqt_controller_manager',
                    executable='rqt_controller_manager',
                    name='controller_manager_gui',
                    output='screen'
                )
            ])
        
        if subsystem in ['all', 'perception']:
            nodes.extend([
                Node(
                    package='tadeo_ecar_perception',
                    executable='lidar_processor_node',
                    name='lidar_debug',
                    parameters=[{
                        'debug_mode': True,
                        'publish_markers': True,
                        'verbose_output': True
                    }],
                    arguments=['--ros-args', '--log-level', debug_level],
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['rqt_image_view'],
                    output='screen'
                )
            ])
        
        if subsystem in ['all', 'navigation']:
            nodes.extend([
                Node(
                    package='tadeo_ecar_navigation',
                    executable='navigation_controller_node',
                    name='navigation_debug',
                    parameters=[{
                        'debug_mode': True,
                        'publish_debug_paths': True
                    }],
                    arguments=['--ros-args', '--log-level', debug_level],
                    output='screen'
                )
            ])
        
        # Herramientas de debug comunes
        nodes.extend([
            # rqt_graph para visualizar conexiones
            ExecuteProcess(
                cmd=['rqt_graph'],
                output='screen'
            ),
            
            # rqt_console para logs
            ExecuteProcess(
                cmd=['rqt_console'],
                output='screen'
            ),
            
            # Monitor de tópicos
            Node(
                package='tadeo_ecar_utils',
                executable='topic_monitor.py',
                name='topic_monitor',
                parameters=[{
                    'monitor_topics': [
                        '/cmd_vel', '/scan', '/odom', '/battery_state'
                    ],
                    'publish_statistics': True
                }],
                output='screen'
            )
        ])
        
        return nodes
    
    # Rosbag recording
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/cmd_vel', '/scan', '/odom', '/tf', '/tf_static',
            '/battery_state', '/system_health', '/diagnostics',
            '-o', 'ecar_debug_session'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_data'))
    )
    
    return LaunchDescription([
        subsystem_arg,
        debug_level_arg,
        record_data_arg,
        
        # Nodos de debug
        OpaqueFunction(function=debug_nodes),
        
        # Recording opcional
        rosbag_record,
        
        # RViz con configuración de debug
        ExecuteProcess(
            cmd=['rviz2', '-d', PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_config'),
                'rviz', 'debug_configuration.rviz'
            ])],
            output='screen'
        ),
        
        # Script de información del sistema
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                '''
                echo "=== eCar Debug Session Started ==="
                echo "Subsystem: $subsystem"
                echo "Debug Level: $debug_level" 
                echo "Recording: $record_data"
                echo "=================================="
                '''
            ],
            output='screen'
        )
    ])
```

## Debugging y Monitoreo

### Launch con Logging Avanzado

```python
# launch/monitored_system.launch.py
def generate_launch_description():
    
    # Configuración de logging
    log_config = {
        'log_level': LaunchConfiguration('log_level'),
        'enable_rosout_logs': True,
        'log_file_path': PathJoinSubstitution([
            '/tmp', 'ecar_logs', 
            [LaunchConfiguration('robot_name'), '_', 
             LaunchConfiguration('session_id'), '.log']
        ])
    }
    
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='INFO'),
        DeclareLaunchArgument('robot_name', default_value='tadeo_ecar'),
        DeclareLaunchArgument('session_id', default_value='default'),
        
        # Crear directorio de logs
        ExecuteProcess(
            cmd=['mkdir', '-p', '/tmp/ecar_logs'],
            output='screen'
        ),
        
        # Nodos con logging específico
        Node(
            package='tadeo_ecar_control',
            executable='wheel_controller_node',
            name='wheel_controller',
            parameters=[log_config],
            arguments=[
                '--ros-args', 
                '--log-level', LaunchConfiguration('log_level'),
                '--log-config-file', PathJoinSubstitution([
                    FindPackageShare('tadeo_ecar_config'),
                    'config', 'logging.conf'
                ])
            ],
            output='both'  # screen y log
        ),
        
        # Monitor de sistema
        Node(
            package='tadeo_ecar_monitoring',
            executable='system_monitor_node',
            name='system_monitor',
            parameters=[{
                'monitor_topics': [
                    '/cmd_vel', '/scan', '/odom', '/battery_state'
                ],
                'check_frequency': 1.0,
                'alert_thresholds': {
                    'cpu_usage': 80.0,
                    'memory_usage': 85.0,
                    'topic_timeout': 2.0
                }
            }],
            output='screen'
        ),
        
        # Aggregator de diagnósticos
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            parameters=[PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_config'),
                'config', 'diagnostic_aggregator.yaml'
            ])],
            output='screen'
        )
    ])
```

### Herramientas de Profiling

```python
# launch/profiling.launch.py
def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument('profile_node', default_value='wheel_controller'),
        DeclareLaunchArgument('profile_duration', default_value='60'),
        
        # Nodo objetivo con profiling
        Node(
            package='tadeo_ecar_control',
            executable='wheel_controller_node',
            name='wheel_controller_profiled',
            prefix=[
                'perf', 'record', '-g', '-o', 'wheel_controller.perf',
                '--'
            ],
            output='screen'
        ),
        
        # Monitor de recursos
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'''
                duration={LaunchConfiguration('profile_duration')}
                node_name={LaunchConfiguration('profile_node')}
                
                echo "Starting profiling session for $node_name ($duration seconds)"
                
                # Monitor CPU y memoria
                while true; do
                    pid=$(pgrep -f $node_name)
                    if [ ! -z "$pid" ]; then
                        echo "$(date): CPU: $(ps -p $pid -o %cpu --no-headers)% Memory: $(ps -p $pid -o %mem --no-headers)%"
                    fi
                    sleep 1
                done &
                
                # Terminar después del tiempo especificado
                sleep $duration
                pkill -f "perf record"
                echo "Profiling session completed"
                '''
            ],
            output='screen'
        )
    ])
```

## Launch Avanzado

### Launch con Estado Persistente

```python
# launch/stateful_system.launch.py
import os
import yaml
from datetime import datetime

def save_launch_state(context, *args, **kwargs):
    """Guardar estado del launch para recuperación"""
    
    state = {
        'timestamp': datetime.now().isoformat(),
        'robot_name': LaunchConfiguration('robot_name').perform(context),
        'environment': LaunchConfiguration('environment').perform(context),
        'use_sim': LaunchConfiguration('use_sim').perform(context),
        'active_nodes': [],  # Se completaría con lógica real
        'session_id': LaunchConfiguration('session_id').perform(context)
    }
    
    state_file = f"/tmp/ecar_state_{state['robot_name']}.yaml"
    
    try:
        with open(state_file, 'w') as f:
            yaml.dump(state, f)
        print(f"Launch state saved to {state_file}")
    except Exception as e:
        print(f"Failed to save launch state: {e}")
    
    return []

def load_previous_state():
    """Cargar estado anterior si existe"""
    # Lógica para cargar estado previo
    pass

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='tadeo_ecar'),
        DeclareLaunchArgument('environment', default_value='indoor'),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('session_id', 
                             default_value=datetime.now().strftime('%Y%m%d_%H%M%S')),
        
        # Guardar estado al inicio
        OpaqueFunction(function=save_launch_state),
        
        # Nodos del sistema...
        Node(
            package='tadeo_ecar_control',
            executable='wheel_controller_node',
            name='wheel_controller',
            on_exit=[
                # Acción cuando el nodo termina
                ExecuteProcess(
                    cmd=['echo', 'Wheel controller exited, attempting restart...'],
                    output='screen'
                )
            ]
        ),
        
        # Cleanup al final
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                '''
                trap 'echo "Cleaning up launch session..."; rm -f /tmp/ecar_state_*.yaml' EXIT
                wait
                '''
            ],
            output='screen'
        )
    ])
```

### Launch con Recuperación Automática

```python
# launch/resilient_system.launch.py
def create_resilient_node(package, executable, name, max_restarts=3):
    """Crear nodo con recuperación automática"""
    
    return Node(
        package=package,
        executable=executable,
        name=name,
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'max_restart_attempts': max_restarts,
            'restart_delay': 2.0
        }],
        output='screen'
    )

def generate_launch_description():
    
    # Nodos críticos con recuperación
    critical_nodes = [
        create_resilient_node(
            'tadeo_ecar_safety', 'safety_monitor_node', 'safety_monitor'
        ),
        create_resilient_node(
            'tadeo_ecar_control', 'wheel_controller_node', 'wheel_controller'
        ),
        create_resilient_node(
            'tadeo_ecar_perception', 'lidar_processor_node', 'lidar_processor'
        )
    ]
    
    # Watchdog del sistema
    system_watchdog = Node(
        package='tadeo_ecar_monitoring',
        executable='system_watchdog_node',
        name='system_watchdog',
        parameters=[{
            'monitored_nodes': [
                'safety_monitor', 'wheel_controller', 'lidar_processor'
            ],
            'check_interval': 1.0,
            'restart_threshold': 3,
            'shutdown_on_critical_failure': True
        }],
        respawn=True,
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('enable_recovery', default_value='true'),
        
        # Nodos críticos
        *critical_nodes,
        
        # Watchdog
        GroupAction([
            system_watchdog
        ], condition=IfCondition(LaunchConfiguration('enable_recovery'))),
        
        # Health check periódico
        TimerAction(
            period=30.0,  # Cada 30 segundos
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        '''
                        echo "=== System Health Check ==="
                        ros2 node list | grep -E "(safety|wheel|lidar)" || echo "Missing critical nodes!"
                        echo "=========================="
                        '''
                    ],
                    output='screen'
                )
            ]
        )
    ])
```

## Ejercicios Prácticos

### Ejercicio 1: Launch File Personalizado

Crear un launch file para un escenario específico:

```python
# launch/warehouse_mission.launch.py
def generate_launch_description():
    # TODO: Crear launch file para misión en almacén
    # - Configuración de velocidades reducidas
    # - Navegación de precisión
    # - Sensores adicionales para espacios estrechos
    # - Logging detallado
    pass
```

### Ejercicio 2: Multi-Robot Launch

```python
# launch/multi_robot_fleet.launch.py
def generate_launch_description():
    # TODO: Launch para múltiples robots eCar
    # - Argumentos para número de robots
    # - Namespaces únicos por robot
    # - Coordinador de flota
    # - Evitar conflictos de recursos
    pass
```

### Ejercicio 3: Testing Launch

```python
# launch/automated_testing.launch.py
def generate_launch_description():
    # TODO: Launch para testing automatizado
    # - Ejecutar suite de tests
    # - Verificar funcionalidad de cada subsistema
    # - Generar reportes de test
    # - Cleanup automático
    pass
```

### Comandos de Testing

```bash
# 1. Verificar sintaxis del launch file
python3 -m py_compile launch/ecar_full_system.launch.py

# 2. Ejecutar con argumentos diferentes
ros2 launch tadeo_ecar_bringup ecar_full_system.launch.py use_sim:=true environment:=outdoor use_rviz:=true

# 3. Debug del launch
ros2 launch --debug tadeo_ecar_bringup ecar_full_system.launch.py

# 4. Listar argumentos disponibles
ros2 launch tadeo_ecar_bringup ecar_full_system.launch.py --show-args

# 5. Generar descripción en XML (para análisis)
ros2 launch tadeo_ecar_bringup ecar_full_system.launch.py --print-description
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Concepto de Launch Files**: Orquestación completa de sistemas robóticos
2. **Sintaxis y Estructura**: Python dinámico vs XML estático
3. **Argumentos y Condiciones**: Configuración flexible y condicional
4. **Composición**: Reutilización e inclusión de launch files
5. **Integración**: Parámetros, namespaces y configuración dinámica
6. **Launch del eCar**: Sistema completo con fases de inicialización
7. **Debugging**: Herramientas para monitoreo y resolución de problemas
8. **Launch Avanzado**: Recuperación automática y estado persistente

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes la estructura y sintaxis de launch files en Python
- [ ] Puedes crear launch files con argumentos y condiciones
- [ ] Sabes usar inclusión y composición de launch files
- [ ] Comprendes la integración con parámetros y configuración
- [ ] Puedes orquestar el sistema eCar completo
- [ ] Sabes usar herramientas de debugging y monitoreo
- [ ] Has implementado launch files con recuperación automática

### Próximo Capítulo

En el Capítulo 9 estudiaremos:
- TF2 y sistema de coordenadas en ROS2
- Transformaciones estáticas y dinámicas
- Herramientas tf2_tools para debugging
- Frames del robot eCar 4WD4WS
- Calibración y configuración de transformaciones

## Referencias

- [ROS2 Launch System](https://docs.ros.org/en/humble/Concepts/About-Launch.html)
- [Launch File Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Launch Actions and Substitutions](https://docs.ros.org/en/humble/Concepts/About-Launch.html#launch-actions)
- [Python Launch API](https://docs.ros.org/en/humble/p/launch/launch_api.html)