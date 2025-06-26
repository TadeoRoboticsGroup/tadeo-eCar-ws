# Capítulo 16: Integración de Sistemas para eCar 4WD4WS

## Tabla de Contenidos

1. [Introducción a la Integración](#introducción-a-la-integración)
2. [Arquitectura de Integración](#arquitectura-de-integración)
3. [Configuración del Sistema Completo](#configuración-del-sistema-completo)
4. [Launch Files Maestros](#launch-files-maestros)
5. [Comunicación Entre Subsistemas](#comunicación-entre-subsistemas)
6. [Sincronización y Coordinación](#sincronización-y-coordinación)
7. [Gestión de Estados del Sistema](#gestión-de-estados-del-sistema)
8. [Validación de Integración](#validación-de-integración)

## Introducción a la Integración

### ¿Qué es la Integración de Sistemas?

La integración de sistemas es el proceso de combinar todos los subsistemas individuales del eCar en un sistema cohesivo y funcional que opera como una unidad coordinada.

```
Sistema Integrado eCar 4WD4WS = Suma de Subsistemas + Coordinación

Subsistemas Principales:
✓ Hardware (motores, sensores, actuadores)
✓ Percepción (LiDAR, cámaras, ultrasónicos)
✓ Navegación (planificación, control, localización)
✓ Seguridad (emergency stop, monitoreo, protocolos)
✓ Behavior Trees (lógica de comportamiento)
✓ Control (4WD4WS, kinemática, PID)
✓ SLAM (mapeo, localización)
✓ Comunicación (ROS2, networking, interfaces)
```

### Desafíos de Integración para el eCar

**1. Complejidad del Hardware 4WD4WS**
- 4 motores independientes para tracción
- 4 servomotores para dirección
- Múltiples sensores distribuidos
- Sistemas de alimentación redundantes

**2. Sincronización Temporal**
- Coordinar movimiento de 8 actuadores
- Fusión de sensores en tiempo real
- Manejo de latencias variables

**3. Gestión de Estados**
- Estados operacionales complejos
- Transiciones seguras entre modos
- Recuperación de fallos

**4. Escalabilidad y Mantenibilidad**
- Configuración modular
- Fácil adición de nuevos sensores
- Actualizaciones incrementales

### Beneficios de la Integración Correcta

```cpp
// Ejemplo de coordinación perfecta en movimiento omnidireccional
class IntegratedMotionController {
public:
    void executeOmnidirectionalMove(double vx, double vy, double vtheta) {
        // 1. Calcular cinemática inversa
        auto wheel_speeds = kinematics_.inverseKinematics(vx, vy, vtheta);
        auto steering_angles = kinematics_.calculateSteeringAngles(vx, vy, vtheta);
        
        // 2. Verificar seguridad
        if (!safety_manager_.isMotionSafe(wheel_speeds, steering_angles)) {
            safety_manager_.executeEmergencyStop();
            return;
        }
        
        // 3. Coordinar movimiento
        motion_synchronizer_.executeCoordinatedMovement(wheel_speeds, steering_angles);
        
        // 4. Monitorear ejecución
        performance_monitor_.trackMovementExecution();
    }
};
```

## Arquitectura de Integración

### Diagrama de Arquitectura del Sistema Completo

```
                    eCar 4WD4WS Integrated System Architecture
                                      |
        ┌─────────────────────────────────────────────────────────────────────┐
        |                    System Integration Layer                         |
        |          (Orquesta y coordina todos los subsistemas)               |
        └─────────────────────┬───────────────────────────────────────────────┘
                              |
        ┌─────────────────────┼───────────────────────────────────────────────┐
        |                     |                                               |
    ┌───▼────┐    ┌───────────▼─────┐    ┌──────────▼────┐    ┌──────────▼────┐
    │Safety  │    │Perception       │    │Navigation     │    │Control        │
    │System  │    │System           │    │System         │    │System         │
    └────────┘    └─────────────────┘    └───────────────┘    └───────────────┘
         |              |                        |                    |
    ┌────▼────┐   ┌─────▼─────┐         ┌───────▼───────┐     ┌───────▼───────┐
    │E-Stop   │   │LiDAR      │         │SLAM           │     │4WD4WS         │
    │Health   │   │Camera     │         │Nav2           │     │Kinematics     │
    │Monitor  │   │Ultrasonic │         │Behavior Trees │     │Motor Control  │
    └─────────┘   └───────────┘         └───────────────┘     └───────────────┘
         |              |                        |                    |
    ┌────▼────┐   ┌─────▼─────┐         ┌───────▼───────┐     ┌───────▼───────┐
    │Protocol │   │Sensor     │         │Path Planning  │     │Servo Control  │
    │Manager  │   │Fusion     │         │Localization   │     │PID Controllers│
    └─────────┘   └───────────┘         └───────────────┘     └───────────────┘
                              |                        |                    |
                    ┌─────────┼────────────────────────┼────────────────────┘
                    |         |                        |
              ┌─────▼─────────▼────────────────────────▼─────┐
              |              Hardware Layer                  |
              |    (Motores, Servos, Sensores, MCUs)        |
              └─────────────────────────────────────────────┘
```

### Subsistemas y Sus Responsabilidades

**1. System Integration Layer**
```cpp
// src/system_integrator.cpp
class SystemIntegrator : public rclcpp::Node
{
public:
    SystemIntegrator() : Node("system_integrator")
    {
        // Coordinador principal del sistema
        initializeSubsystems();
        setupIntersubsystemCommunication();
        startSystemMonitoring();
    }
    
private:
    void initializeSubsystems() {
        // Inicializar en orden de dependencias
        safety_system_ = std::make_shared<SafetySystem>();
        perception_system_ = std::make_shared<PerceptionSystem>();
        navigation_system_ = std::make_shared<NavigationSystem>();
        control_system_ = std::make_shared<ControlSystem>();
    }
};
```

**2. Safety System**
- Emergency stop management
- Health monitoring
- Safety protocols
- Fault detection and recovery

**3. Perception System**
- Sensor data acquisition
- Data preprocessing and filtering
- Sensor fusion
- Object detection and tracking

**4. Navigation System**
- SLAM and localization
- Path planning (global and local)
- Behavior tree execution
- Goal management

**5. Control System**
- 4WD4WS kinematics
- Motor and servo control
- Motion coordination
- Feedback control loops

## Configuración del Sistema Completo

### Master Configuration Manager

```cpp
// src/configuration_manager.cpp
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

class ConfigurationManager : public rclcpp::Node
{
public:
    ConfigurationManager() : Node("configuration_manager")
    {
        // Cargar configuración desde archivos YAML
        loadSystemConfiguration();
        validateConfiguration();
        distributeConfiguration();
        
        RCLCPP_INFO(this->get_logger(), "System configuration loaded and validated");
    }
    
private:
    struct SystemConfig {
        // Hardware configuration
        struct {
            std::vector<std::string> motor_controllers;
            std::vector<std::string> servo_controllers;
            std::vector<std::string> sensor_devices;
            double wheel_base;
            double track_width;
            double wheel_radius;
        } hardware;
        
        // Performance parameters
        struct {
            double max_linear_velocity;
            double max_angular_velocity;
            double max_acceleration;
            double control_frequency;
        } performance;
        
        // Safety parameters
        struct {
            double emergency_stop_deceleration;
            double min_obstacle_distance;
            double battery_critical_voltage;
            std::vector<double> safety_zones;
        } safety;
        
        // Communication settings
        struct {
            std::string network_interface;
            int primary_port;
            int backup_port;
            double timeout_duration;
        } communication;
    };
    
    void loadSystemConfiguration()
    {
        std::string config_path = getConfigurationPath();
        
        try {
            // Cargar configuración principal
            YAML::Node main_config = YAML::LoadFile(config_path + "/main_config.yaml");
            parseMainConfiguration(main_config);
            
            // Cargar configuraciones específicas
            loadHardwareConfiguration(config_path + "/hardware_config.yaml");
            loadPerformanceConfiguration(config_path + "/performance_config.yaml");
            loadSafetyConfiguration(config_path + "/safety_config.yaml");
            loadNavigationConfiguration(config_path + "/navigation_config.yaml");
            
        } catch (const YAML::Exception& e) {
            RCLCPP_FATAL(this->get_logger(), 
                        "Failed to load configuration: %s", e.what());
            throw std::runtime_error("Configuration loading failed");
        }
    }
    
    void parseMainConfiguration(const YAML::Node& config)
    {
        // Sistema principal
        system_config_.hardware.wheel_base = config["robot"]["wheel_base"].as<double>();
        system_config_.hardware.track_width = config["robot"]["track_width"].as<double>();
        system_config_.hardware.wheel_radius = config["robot"]["wheel_radius"].as<double>();
        
        // Límites de rendimiento
        system_config_.performance.max_linear_velocity = 
            config["limits"]["max_linear_velocity"].as<double>();
        system_config_.performance.max_angular_velocity = 
            config["limits"]["max_angular_velocity"].as<double>();
        system_config_.performance.control_frequency = 
            config["control"]["frequency"].as<double>();
    }
    
    void loadHardwareConfiguration(const std::string& config_file)
    {
        YAML::Node hw_config = YAML::LoadFile(config_file);
        
        // Motores
        for (const auto& motor : hw_config["motors"]) {
            system_config_.hardware.motor_controllers.push_back(
                motor["controller"].as<std::string>());
        }
        
        // Servos
        for (const auto& servo : hw_config["servos"]) {
            system_config_.hardware.servo_controllers.push_back(
                servo["controller"].as<std::string>());
        }
        
        // Sensores
        for (const auto& sensor : hw_config["sensors"]) {
            system_config_.hardware.sensor_devices.push_back(
                sensor["device"].as<std::string>());
        }
    }
    
    void loadSafetyConfiguration(const std::string& config_file)
    {
        YAML::Node safety_config = YAML::LoadFile(config_file);
        
        system_config_.safety.emergency_stop_deceleration = 
            safety_config["emergency"]["max_deceleration"].as<double>();
        system_config_.safety.min_obstacle_distance = 
            safety_config["proximity"]["min_distance"].as<double>();
        system_config_.safety.battery_critical_voltage = 
            safety_config["power"]["critical_voltage"].as<double>();
        
        // Zonas de seguridad
        for (const auto& zone : safety_config["zones"]) {
            system_config_.safety.safety_zones.push_back(zone.as<double>());
        }
    }
    
    void validateConfiguration()
    {
        std::vector<std::string> validation_errors;
        
        // Validar parámetros físicos
        if (system_config_.hardware.wheel_base <= 0) {
            validation_errors.push_back("Invalid wheel_base: must be positive");
        }
        
        if (system_config_.hardware.track_width <= 0) {
            validation_errors.push_back("Invalid track_width: must be positive");
        }
        
        // Validar límites de rendimiento
        if (system_config_.performance.max_linear_velocity <= 0) {
            validation_errors.push_back("Invalid max_linear_velocity: must be positive");
        }
        
        if (system_config_.performance.control_frequency <= 0) {
            validation_errors.push_back("Invalid control_frequency: must be positive");
        }
        
        // Validar configuración de seguridad
        if (system_config_.safety.min_obstacle_distance <= 0) {
            validation_errors.push_back("Invalid min_obstacle_distance: must be positive");
        }
        
        // Verificar número de controladores
        if (system_config_.hardware.motor_controllers.size() != 4) {
            validation_errors.push_back("Invalid number of motor controllers: expected 4");
        }
        
        if (system_config_.hardware.servo_controllers.size() != 4) {
            validation_errors.push_back("Invalid number of servo controllers: expected 4");
        }
        
        if (!validation_errors.empty()) {
            for (const auto& error : validation_errors) {
                RCLCPP_ERROR(this->get_logger(), "Configuration error: %s", error.c_str());
            }
            throw std::runtime_error("Configuration validation failed");
        }
        
        RCLCPP_INFO(this->get_logger(), "Configuration validation successful");
    }
    
    void distributeConfiguration()
    {
        // Distribuir configuración a todos los subsistemas
        publishHardwareConfiguration();
        publishPerformanceConfiguration();
        publishSafetyConfiguration();
        publishNavigationConfiguration();
    }
    
    void publishHardwareConfiguration()
    {
        // Crear parámetros para nodos de hardware
        auto hardware_params = createHardwareParameterMap();
        
        // Distribuir a nodos específicos
        for (const auto& [node_name, params] : hardware_params) {
            distributeParametersToNode(node_name, params);
        }
    }
    
    std::map<std::string, std::map<std::string, rclcpp::Parameter>> 
    createHardwareParameterMap()
    {
        std::map<std::string, std::map<std::string, rclcpp::Parameter>> param_map;
        
        // Parámetros para controlador de motores
        param_map["motor_controller"] = {
            {"wheel_base", rclcpp::Parameter("wheel_base", system_config_.hardware.wheel_base)},
            {"track_width", rclcpp::Parameter("track_width", system_config_.hardware.track_width)},
            {"wheel_radius", rclcpp::Parameter("wheel_radius", system_config_.hardware.wheel_radius)},
            {"max_velocity", rclcpp::Parameter("max_velocity", system_config_.performance.max_linear_velocity)}
        };
        
        // Parámetros para controlador de servos
        param_map["servo_controller"] = {
            {"max_steering_angle", rclcpp::Parameter("max_steering_angle", M_PI/3)},
            {"steering_rate_limit", rclcpp::Parameter("steering_rate_limit", 2.0)}
        };
        
        return param_map;
    }
    
    void distributeParametersToNode(const std::string& node_name, 
                                   const std::map<std::string, rclcpp::Parameter>& params)
    {
        // En implementación real: usar servicio de configuración de parámetros
        RCLCPP_INFO(this->get_logger(), 
                   "Distributing %zu parameters to node: %s", 
                   params.size(), node_name.c_str());
    }
    
    std::string getConfigurationPath()
    {
        // Buscar archivos de configuración en ubicaciones estándar
        std::vector<std::string> search_paths = {
            "/opt/ros/humble/share/tadeo_ecar_config/config",
            "~/tadeo-eCar-ws/src/tadeo_ecar_config/config",
            "./config"
        };
        
        for (const auto& path : search_paths) {
            if (std::filesystem::exists(path)) {
                return path;
            }
        }
        
        throw std::runtime_error("Configuration files not found in any search path");
    }
    
    SystemConfig system_config_;
};
```

### Configuraciones YAML Maestras

```yaml
# config/main_config.yaml
---
robot:
  name: "eCar_4WD4WS"
  version: "1.0.0"
  wheel_base: 0.6  # metros
  track_width: 0.4  # metros
  wheel_radius: 0.1  # metros
  
limits:
  max_linear_velocity: 2.0  # m/s
  max_angular_velocity: 1.0  # rad/s
  max_acceleration: 1.5  # m/s²
  max_deceleration: 3.0  # m/s²
  
control:
  frequency: 50.0  # Hz
  timeout: 0.1  # segundos
  
system:
  startup_timeout: 30.0  # segundos
  shutdown_timeout: 10.0  # segundos
  health_check_interval: 1.0  # segundos
```

```yaml
# config/hardware_config.yaml
---
motors:
  - name: "front_left"
    controller: "/dev/ttyUSB0"
    id: 1
    max_rpm: 3000
    reduction_ratio: 10.0
    encoder_resolution: 4096
    
  - name: "front_right"
    controller: "/dev/ttyUSB0"
    id: 2
    max_rpm: 3000
    reduction_ratio: 10.0
    encoder_resolution: 4096
    
  - name: "rear_left"
    controller: "/dev/ttyUSB0"
    id: 3
    max_rpm: 3000
    reduction_ratio: 10.0
    encoder_resolution: 4096
    
  - name: "rear_right"
    controller: "/dev/ttyUSB0"
    id: 4
    max_rpm: 3000
    reduction_ratio: 10.0
    encoder_resolution: 4096

servos:
  - name: "steering_front_left"
    controller: "/dev/ttyUSB1"
    id: 1
    max_angle: 60.0  # grados
    center_position: 1500  # microsegundos
    
  - name: "steering_front_right"
    controller: "/dev/ttyUSB1"
    id: 2
    max_angle: 60.0
    center_position: 1500
    
  - name: "steering_rear_left"
    controller: "/dev/ttyUSB1"
    id: 3
    max_angle: 60.0
    center_position: 1500
    
  - name: "steering_rear_right"
    controller: "/dev/ttyUSB1"
    id: 4
    max_angle: 60.0
    center_position: 1500

sensors:
  lidar:
    device: "/dev/ttyUSB2"
    model: "SICK_TiM571"
    frequency: 15.0
    range_max: 25.0
    
  cameras:
    front:
      device: "/dev/video0"
      resolution: [1920, 1080]
      fps: 30
      
    rear:
      device: "/dev/video1"
      resolution: [1280, 720]
      fps: 30
  
  imu:
    device: "/dev/ttyUSB3"
    model: "BNO055"
    frequency: 100.0
    
  ultrasonic:
    count: 8
    frequency: 20.0
    range_max: 4.0
```

```yaml
# config/integration_config.yaml
---
system_integration:
  startup_sequence:
    - safety_system
    - hardware_interfaces
    - perception_system
    - control_system
    - navigation_system
    - behavior_system
    
  shutdown_sequence:
    - behavior_system
    - navigation_system
    - control_system
    - perception_system
    - hardware_interfaces
    - safety_system
    
  critical_nodes:
    - emergency_stop_manager
    - health_monitor
    - motor_controller
    - servo_controller
    
  monitoring:
    node_timeout: 5.0
    heartbeat_interval: 1.0
    restart_attempts: 3
    
synchronization:
  control_loop_frequency: 50.0
  perception_frequency: 20.0
  navigation_frequency: 10.0
  safety_frequency: 100.0
  
communication:
  internal_network: "192.168.1.0/24"
  external_network: "10.0.0.0/8"
  discovery_timeout: 30.0
  
state_management:
  default_state: "STANDBY"
  allowed_transitions:
    STANDBY: ["INITIALIZATION", "SHUTDOWN"]
    INITIALIZATION: ["READY", "ERROR", "STANDBY"]
    READY: ["AUTONOMOUS", "MANUAL", "MAINTENANCE", "STANDBY"]
    AUTONOMOUS: ["READY", "EMERGENCY", "ERROR"]
    MANUAL: ["READY", "EMERGENCY", "ERROR"]
    MAINTENANCE: ["READY", "STANDBY"]
    EMERGENCY: ["STANDBY", "SHUTDOWN"]
    ERROR: ["STANDBY", "MAINTENANCE", "SHUTDOWN"]
    SHUTDOWN: []
```

## Launch Files Maestros

### Launch Principal del Sistema

```python
# launch/ecar_system.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Argumentos de configuración
    config_args = [
        DeclareLaunchArgument('robot_name', default_value='ecar_001'),
        DeclareLaunchArgument('use_simulation', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_safety', default_value='true'),
        DeclareLaunchArgument('use_perception', default_value='true'),
        DeclareLaunchArgument('use_navigation', default_value='true'),
        DeclareLaunchArgument('debug_mode', default_value='false'),
    ]
    
    # Rutas de configuración
    config_dir = PathJoinSubstitution([
        FindPackageShare('tadeo_ecar_config'),
        'config'
    ])
    
    # 1. Configuration Manager (Primero - crítico)
    config_manager = Node(
        package='tadeo_ecar_integration',
        executable='configuration_manager',
        name='configuration_manager',
        output='screen',
        parameters=[{
            'config_path': config_dir,
            'robot_name': LaunchConfiguration('robot_name'),
            'debug_mode': LaunchConfiguration('debug_mode'),
        }]
    )
    
    # 2. Safety System (Segundo - crítico)
    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_safety'),
                'launch',
                'safety_system.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_hardware_estop': 'true',
            'enable_health_monitor': 'true',
            'enable_safety_protocols': 'true',
            'debug_mode': LaunchConfiguration('debug_mode'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_safety'))
    )
    
    # 3. Hardware Interfaces
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_hardware'),
                'launch',
                'hardware_interfaces.launch.py'
            ])
        ]),
        launch_arguments={
            'use_simulation': LaunchConfiguration('use_simulation'),
            'config_file': PathJoinSubstitution([config_dir, 'hardware_config.yaml']),
        }.items()
    )
    
    # 4. Perception System
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_perception'),
                'launch',
                'perception.launch.py'
            ])
        ]),
        launch_arguments={
            'use_camera': 'true',
            'use_lidar': 'true',
            'use_ultrasonic': 'true',
            'debug_visualization': LaunchConfiguration('debug_mode'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_perception'))
    )
    
    # 5. Control System
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_control'),
                'launch',
                'control_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_4wd4ws': 'true',
            'control_frequency': '50.0',
            'enable_pid_tuning': LaunchConfiguration('debug_mode'),
        }.items()
    )
    
    # 6. Navigation System
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_slam': 'true',
            'use_nav2': 'true',
            'use_behavior_trees': 'true',
            'map_file': PathJoinSubstitution([config_dir, 'maps', 'default_map.yaml']),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_navigation'))
    )
    
    # 7. Behavior System
    behavior_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_behavior'),
                'launch',
                'behavior_trees.launch.py'
            ])
        ]),
        launch_arguments={
            'behavior_tree_file': PathJoinSubstitution([
                config_dir, 'behavior_trees', 'main_behavior.xml'
            ]),
            'enable_groot': LaunchConfiguration('debug_mode'),
        }.items()
    )
    
    # 8. System Monitor
    system_monitor = Node(
        package='tadeo_ecar_integration',
        executable='system_monitor',
        name='system_monitor',
        output='screen',
        parameters=[{
            'monitoring_frequency': 1.0,
            'critical_nodes': [
                'configuration_manager',
                'emergency_stop_manager',
                'health_monitor',
                'motor_controller',
                'servo_controller'
            ],
            'restart_attempts': 3,
            'restart_delay': 5.0,
        }]
    )
    
    # 9. TF2 Static Transforms
    tf_static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_description'),
                'launch',
                'tf_static.launch.py'
            ])
        ])
    )
    
    # 10. Visualization (opcional)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tadeo_ecar_visualization'),
                'launch',
                'rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz_config': PathJoinSubstitution([
                config_dir, 'rviz', 'ecar_integration.rviz'
            ]),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # 11. Logging and Diagnostics
    logging_setup = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', 
            '/tf', '/tf_static', '/diagnostics', '/safety_status',
            '/health_alerts', '/cmd_vel', '/odom', '/scan',
            '-o', f'/tmp/ecar_logs_{LaunchConfiguration("robot_name")}'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug_mode'))
    )
    
    return LaunchDescription(
        config_args + [
            # Orden crítico de inicio
            config_manager,
            safety_launch,
            hardware_launch,
            tf_static_transforms,
            control_launch,
            perception_launch,
            navigation_launch,
            behavior_launch,
            system_monitor,
            rviz_launch,
            logging_setup,
        ]
    )
```

### System Monitor

```cpp
// src/system_monitor.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

class SystemMonitor : public rclcpp::Node
{
public:
    SystemMonitor() : Node("system_monitor")
    {
        // Parámetros
        this->declare_parameter("monitoring_frequency", 1.0);
        this->declare_parameter("critical_nodes", std::vector<std::string>{});
        this->declare_parameter("restart_attempts", 3);
        this->declare_parameter("restart_delay", 5.0);
        
        // Obtener lista de nodos críticos
        critical_nodes_ = this->get_parameter("critical_nodes").as_string_array();
        
        // Publicadores
        system_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/system_status", 10);
        
        system_diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/system_diagnostics", 10);
        
        // Timer de monitoreo
        double frequency = this->get_parameter("monitoring_frequency").as_double();
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / frequency)),
            std::bind(&SystemMonitor::monitorSystem, this));
        
        // Inicializar estado
        initializeNodeStates();
        
        RCLCPP_INFO(this->get_logger(), 
                   "System Monitor initialized - monitoring %zu critical nodes", 
                   critical_nodes_.size());
    }
    
private:
    struct NodeState {
        std::string name;
        bool is_alive = false;
        rclcpp::Time last_seen;
        int restart_attempts = 0;
        std::string last_error;
    };
    
    void initializeNodeStates()
    {
        for (const auto& node_name : critical_nodes_) {
            NodeState state;
            state.name = node_name;
            state.last_seen = this->now();
            node_states_[node_name] = state;
        }
    }
    
    void monitorSystem()
    {
        // 1. Verificar nodos críticos
        checkCriticalNodes();
        
        // 2. Verificar salud del sistema
        checkSystemHealth();
        
        // 3. Verificar comunicación
        checkCommunication();
        
        // 4. Publicar estado del sistema
        publishSystemStatus();
        
        // 5. Publicar diagnósticos
        publishSystemDiagnostics();
    }
    
    void checkCriticalNodes()
    {
        auto node_names = this->get_node_graph_interface()->get_node_names();
        
        for (auto& [node_name, state] : node_states_) {
            // Verificar si el nodo está presente
            bool node_found = std::find(node_names.begin(), node_names.end(), 
                                       node_name) != node_names.end();
            
            if (node_found) {
                if (!state.is_alive) {
                    RCLCPP_INFO(this->get_logger(), 
                               "Critical node recovered: %s", node_name.c_str());
                    state.restart_attempts = 0;  // Reset counter
                }
                state.is_alive = true;
                state.last_seen = this->now();
            } else {
                if (state.is_alive) {
                    RCLCPP_ERROR(this->get_logger(), 
                                "Critical node lost: %s", node_name.c_str());
                    handleNodeFailure(node_name);
                }
                state.is_alive = false;
            }
        }
    }
    
    void handleNodeFailure(const std::string& node_name)
    {
        auto& state = node_states_[node_name];
        int max_attempts = this->get_parameter("restart_attempts").as_int();
        
        if (state.restart_attempts < max_attempts) {
            state.restart_attempts++;
            
            RCLCPP_WARN(this->get_logger(), 
                       "Attempting to restart node %s (attempt %d/%d)", 
                       node_name.c_str(), state.restart_attempts, max_attempts);
            
            // Intentar reiniciar el nodo
            restartNode(node_name);
        } else {
            RCLCPP_FATAL(this->get_logger(), 
                        "Node %s failed permanently after %d attempts", 
                        node_name.c_str(), max_attempts);
            
            // Entrar en modo de emergencia
            triggerSystemEmergency("Critical node permanent failure: " + node_name);
        }
    }
    
    void restartNode(const std::string& node_name)
    {
        // En implementación real: usar launch service o systemd
        // Por ahora, log del intento
        RCLCPP_INFO(this->get_logger(), "Restarting node: %s", node_name.c_str());
        
        // Delay antes del próximo check
        double delay = this->get_parameter("restart_delay").as_double();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(delay * 1000)));
    }
    
    void checkSystemHealth()
    {
        // Verificar uso de recursos del sistema
        SystemResourceUsage usage = getSystemResourceUsage();
        
        // CPU
        if (usage.cpu_percent > 90.0) {
            RCLCPP_WARN(this->get_logger(), 
                       "High CPU usage: %.1f%%", usage.cpu_percent);
        }
        
        // Memoria
        if (usage.memory_percent > 85.0) {
            RCLCPP_WARN(this->get_logger(), 
                       "High memory usage: %.1f%%", usage.memory_percent);
        }
        
        // Disco
        if (usage.disk_percent > 90.0) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Low disk space: %.1f%% used", usage.disk_percent);
        }
        
        // Temperatura (si disponible)
        if (usage.temperature > 80.0) {
            RCLCPP_ERROR(this->get_logger(), 
                        "High system temperature: %.1f°C", usage.temperature);
        }
    }
    
    void checkCommunication()
    {
        // Verificar latencia de red
        auto network_stats = getNetworkStatistics();
        
        if (network_stats.latency_ms > 100.0) {
            RCLCPP_WARN(this->get_logger(), 
                       "High network latency: %.1fms", network_stats.latency_ms);
        }
        
        if (network_stats.packet_loss_percent > 5.0) {
            RCLCPP_ERROR(this->get_logger(), 
                        "High packet loss: %.1f%%", network_stats.packet_loss_percent);
        }
    }
    
    void triggerSystemEmergency(const std::string& reason)
    {
        RCLCPP_FATAL(this->get_logger(), "SYSTEM EMERGENCY: %s", reason.c_str());
        
        // Publicar evento de emergencia
        std_msgs::msg::String emergency_msg;
        emergency_msg.data = "EMERGENCY: " + reason;
        system_status_pub_->publish(emergency_msg);
        
        // En implementación real: activar protocolo de emergencia del sistema
    }
    
    void publishSystemStatus()
    {
        // Calcular estado general del sistema
        std::string overall_status = calculateOverallStatus();
        
        std_msgs::msg::String status_msg;
        status_msg.data = overall_status;
        system_status_pub_->publish(status_msg);
    }
    
    std::string calculateOverallStatus()
    {
        int alive_nodes = 0;
        int total_nodes = node_states_.size();
        
        for (const auto& [name, state] : node_states_) {
            if (state.is_alive) {
                alive_nodes++;
            }
        }
        
        if (alive_nodes == total_nodes) {
            return "SYSTEM_HEALTHY";
        } else if (alive_nodes >= total_nodes * 0.8) {
            return "SYSTEM_DEGRADED";
        } else if (alive_nodes >= total_nodes * 0.5) {
            return "SYSTEM_CRITICAL";
        } else {
            return "SYSTEM_FAILURE";
        }
    }
    
    void publishSystemDiagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = this->now();
        
        // Diagnóstico de nodos críticos
        diagnostic_msgs::msg::DiagnosticStatus nodes_diag;
        nodes_diag.name = "Critical Nodes";
        nodes_diag.hardware_id = "system_monitor";
        
        int alive_count = 0;
        for (const auto& [name, state] : node_states_) {
            if (state.is_alive) alive_count++;
            
            diagnostic_msgs::msg::KeyValue node_kv;
            node_kv.key = name;
            node_kv.value = state.is_alive ? "ALIVE" : "DEAD";
            nodes_diag.values.push_back(node_kv);
        }
        
        if (alive_count == static_cast<int>(node_states_.size())) {
            nodes_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            nodes_diag.message = "All critical nodes operational";
        } else {
            nodes_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            nodes_diag.message = std::to_string(node_states_.size() - alive_count) + 
                                " critical nodes failed";
        }
        
        diag_array.status.push_back(nodes_diag);
        
        // Diagnóstico de recursos del sistema
        auto resources_diag = createResourceDiagnostic();
        diag_array.status.push_back(resources_diag);
        
        system_diagnostics_pub_->publish(diag_array);
    }
    
    diagnostic_msgs::msg::DiagnosticStatus createResourceDiagnostic()
    {
        diagnostic_msgs::msg::DiagnosticStatus resource_diag;
        resource_diag.name = "System Resources";
        resource_diag.hardware_id = "system_monitor";
        
        auto usage = getSystemResourceUsage();
        
        // Determinar nivel de diagnóstico
        if (usage.cpu_percent > 90 || usage.memory_percent > 85 || usage.disk_percent > 90) {
            resource_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            resource_diag.message = "Critical resource usage";
        } else if (usage.cpu_percent > 80 || usage.memory_percent > 75 || usage.disk_percent > 80) {
            resource_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            resource_diag.message = "High resource usage";
        } else {
            resource_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            resource_diag.message = "Normal resource usage";
        }
        
        // Agregar valores
        diagnostic_msgs::msg::KeyValue cpu_kv;
        cpu_kv.key = "cpu_percent";
        cpu_kv.value = std::to_string(usage.cpu_percent);
        resource_diag.values.push_back(cpu_kv);
        
        diagnostic_msgs::msg::KeyValue memory_kv;
        memory_kv.key = "memory_percent";
        memory_kv.value = std::to_string(usage.memory_percent);
        resource_diag.values.push_back(memory_kv);
        
        diagnostic_msgs::msg::KeyValue disk_kv;
        disk_kv.key = "disk_percent";
        disk_kv.value = std::to_string(usage.disk_percent);
        resource_diag.values.push_back(disk_kv);
        
        return resource_diag;
    }
    
    struct SystemResourceUsage {
        double cpu_percent = 0.0;
        double memory_percent = 0.0;
        double disk_percent = 0.0;
        double temperature = 0.0;
    };
    
    struct NetworkStatistics {
        double latency_ms = 0.0;
        double packet_loss_percent = 0.0;
        double bandwidth_mbps = 0.0;
    };
    
    SystemResourceUsage getSystemResourceUsage()
    {
        // Implementar lectura real de recursos del sistema
        SystemResourceUsage usage;
        usage.cpu_percent = 45.0;    // Placeholder
        usage.memory_percent = 60.0; // Placeholder
        usage.disk_percent = 25.0;   // Placeholder
        usage.temperature = 55.0;    // Placeholder
        return usage;
    }
    
    NetworkStatistics getNetworkStatistics()
    {
        // Implementar medición real de red
        NetworkStatistics stats;
        stats.latency_ms = 10.0;           // Placeholder
        stats.packet_loss_percent = 0.1;   // Placeholder
        stats.bandwidth_mbps = 100.0;      // Placeholder
        return stats;
    }
    
    // Miembros
    std::vector<std::string> critical_nodes_;
    std::map<std::string, NodeState> node_states_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr system_diagnostics_pub_;
    
    rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Validation Script

```bash
#!/bin/bash
# scripts/validate_integration.sh

echo "=== Validación de Integración del Sistema eCar ==="

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Función para verificar si un nodo está corriendo
check_node() {
    local node_name=$1
    if ros2 node list | grep -q "$node_name"; then
        echo -e "${GREEN}✓${NC} $node_name está corriendo"
        return 0
    else
        echo -e "${RED}✗${NC} $node_name NO está corriendo"
        return 1
    fi
}

# Función para verificar topic
check_topic() {
    local topic_name=$1
    if ros2 topic list | grep -q "$topic_name"; then
        echo -e "${GREEN}✓${NC} Topic $topic_name disponible"
        return 0
    else
        echo -e "${RED}✗${NC} Topic $topic_name NO disponible"
        return 1
    fi
}

# Función para verificar frecuencia de topic
check_topic_frequency() {
    local topic_name=$1
    local expected_min_freq=$2
    
    echo "Verificando frecuencia de $topic_name..."
    local freq=$(timeout 5s ros2 topic hz "$topic_name" 2>/dev/null | tail -n 1 | grep -o '[0-9]*\.[0-9]*')
    
    if [ -n "$freq" ]; then
        if (( $(echo "$freq >= $expected_min_freq" | bc -l) )); then
            echo -e "${GREEN}✓${NC} $topic_name frecuencia: ${freq}Hz (>= ${expected_min_freq}Hz)"
            return 0
        else
            echo -e "${YELLOW}⚠${NC} $topic_name frecuencia baja: ${freq}Hz (< ${expected_min_freq}Hz)"
            return 1
        fi
    else
        echo -e "${RED}✗${NC} No se pudo medir frecuencia de $topic_name"
        return 1
    fi
}

# Inicializar contadores
total_checks=0
passed_checks=0

echo ""
echo "1. Verificando Nodos Críticos..."
critical_nodes=(
    "configuration_manager"
    "emergency_stop_manager"
    "health_monitor"
    "system_monitor"
    "motor_controller"
    "servo_controller"
)

for node in "${critical_nodes[@]}"; do
    ((total_checks++))
    if check_node "$node"; then
        ((passed_checks++))
    fi
done

echo ""
echo "2. Verificando Topics Críticos..."
critical_topics=(
    "/cmd_vel"
    "/odom"
    "/scan"
    "/diagnostics"
    "/estop_status"
    "/health_alerts"
    "/system_status"
)

for topic in "${critical_topics[@]}"; do
    ((total_checks++))
    if check_topic "$topic"; then
        ((passed_checks++))
    fi
done

echo ""
echo "3. Verificando Frecuencias de Topics..."
topic_frequencies=(
    "/cmd_vel:1.0"
    "/odom:10.0"
    "/scan:5.0"
    "/diagnostics:0.5"
)

for topic_freq in "${topic_frequencies[@]}"; do
    IFS=':' read -r topic min_freq <<< "$topic_freq"
    ((total_checks++))
    if check_topic_frequency "$topic" "$min_freq"; then
        ((passed_checks++))
    fi
done

echo ""
echo "4. Verificando TF Tree..."
((total_checks++))
if timeout 5s ros2 run tf2_ros tf2_echo map base_link >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} TF tree funcional (map -> base_link)"
    ((passed_checks++))
else
    echo -e "${RED}✗${NC} TF tree no funcional"
fi

echo ""
echo "5. Verificando Servicios Críticos..."
critical_services=(
    "/emergency_stop_manager/set_parameters"
    "/health_monitor/get_parameters"
)

for service in "${critical_services[@]}"; do
    ((total_checks++))
    if ros2 service list | grep -q "$service"; then
        echo -e "${GREEN}✓${NC} Servicio $service disponible"
        ((passed_checks++))
    else
        echo -e "${RED}✗${NC} Servicio $service NO disponible"
    fi
done

echo ""
echo "6. Test de Comunicación End-to-End..."
((total_checks++))

# Test: publicar cmd_vel y verificar que llegue a motores
echo "Enviando comando de prueba..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once >/dev/null 2>&1

sleep 2

# Verificar que el comando se procesó
if timeout 3s ros2 topic echo /motor_commands --once >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Comunicación end-to-end funcional"
    ((passed_checks++))
else
    echo -e "${RED}✗${NC} Comunicación end-to-end fallida"
fi

echo ""
echo "7. Verificando Estado del Sistema..."
((total_checks++))

# Verificar estado general del sistema
system_status=$(timeout 3s ros2 topic echo /system_status std_msgs/msg/String --once 2>/dev/null | grep "data:" | cut -d'"' -f2)

if [ "$system_status" = "SYSTEM_HEALTHY" ]; then
    echo -e "${GREEN}✓${NC} Estado del sistema: HEALTHY"
    ((passed_checks++))
elif [ "$system_status" = "SYSTEM_DEGRADED" ]; then
    echo -e "${YELLOW}⚠${NC} Estado del sistema: DEGRADED"
    ((passed_checks++))
else
    echo -e "${RED}✗${NC} Estado del sistema: $system_status"
fi

echo ""
echo "8. Test de Seguridad..."
((total_checks++))

# Verificar que emergency stop responde
echo "Verificando respuesta de emergency stop..."
if timeout 3s ros2 topic echo /estop_status std_msgs/msg/Bool --once >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Sistema de emergency stop responde"
    ((passed_checks++))
else
    echo -e "${RED}✗${NC} Sistema de emergency stop no responde"
fi

echo ""
echo "=== RESUMEN DE VALIDACIÓN ==="
echo "Checks pasados: $passed_checks/$total_checks"

percentage=$((passed_checks * 100 / total_checks))

if [ $percentage -eq 100 ]; then
    echo -e "${GREEN}🎉 INTEGRACIÓN EXITOSA (100%)${NC}"
    echo "El sistema eCar está completamente integrado y funcionando correctamente."
    exit 0
elif [ $percentage -ge 80 ]; then
    echo -e "${YELLOW}⚠ INTEGRACIÓN PARCIAL ($percentage%)${NC}"
    echo "El sistema está mayormente funcional pero requiere atención."
    exit 1
else
    echo -e "${RED}❌ INTEGRACIÓN FALLIDA ($percentage%)${NC}"
    echo "El sistema requiere corrección antes de operación."
    exit 2
fi
```

Este capítulo cubre todos los aspectos críticos de la integración del sistema eCar 4WD4WS, desde la configuración del sistema completo hasta la validación de la integración exitosa.