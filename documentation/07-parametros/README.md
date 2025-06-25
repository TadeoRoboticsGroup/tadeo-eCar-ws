# Capítulo 7: Parámetros en ROS2

## Tabla de Contenidos

1. [Concepto de Parámetros](#concepto-de-parámetros)
2. [Tipos de Parámetros](#tipos-de-parámetros)
3. [Declaración y Uso](#declaración-y-uso)
4. [Archivos YAML](#archivos-yaml)
5. [Parámetros Dinámicos](#parámetros-dinámicos)
6. [Implementación en C++](#implementación-en-c++)
7. [Implementación en Python](#implementación-en-python)
8. [Configuración del eCar](#configuración-del-ecar)

## Concepto de Parámetros

### ¿Qué son los Parámetros?

Los parámetros en ROS2 son valores de configuración que los nodos pueden declarar, leer y modificar durante la ejecución. Permiten ajustar el comportamiento del robot sin recompilar código.

```
Parámetros = Variables de Configuración Dinámicas

┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Node A          │    │ Parameter       │    │ External        │
│ linear_vel=0.5  │ ←──│ Server          │←── │ Configuration   │
│ angular_vel=0.3 │    │ (per node)      │    │ (YAML, CLI)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Ventajas de los Parámetros

**1. Configuración sin Recompilación**
```bash
# Cambiar velocidad máxima sin recompilar
ros2 param set /wheel_controller_node max_velocity 2.0
```

**2. Configuración por Entorno**
```yaml
# config/indoor.yaml - Para uso interior
wheel_controller_node:
  ros__parameters:
    max_velocity: 0.5
    safety_distance: 0.5

# config/outdoor.yaml - Para uso exterior  
wheel_controller_node:
  ros__parameters:
    max_velocity: 2.0
    safety_distance: 1.0
```

**3. Ajuste en Tiempo Real**
```cpp
// Callback automático cuando cambia parámetro
void parameterCallback(const std::vector<rclcpp::Parameter>& parameters) {
    for (const auto& param : parameters) {
        if (param.get_name() == "max_velocity") {
            max_velocity_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated max_velocity to %.2f", max_velocity_);
        }
    }
}
```

### Casos de Uso en el eCar

**Configuración de Hardware:**
- Dimensiones del robot (wheelbase, track width)
- Parámetros de motores (gear ratios, limits)
- Calibración de sensores (offsets, noise models)

**Comportamiento de Control:**
- Velocidades máximas por modo de operación
- Ganancias PID para control de velocidad
- Tolerancias de navegación

**Configuración de Seguridad:**
- Distancias de parada de emergencia
- Timeouts de comunicación
- Límites de temperatura y voltaje

## Tipos de Parámetros

### Tipos Básicos Soportados

```cpp
// Tipos primitivos
bool emergency_stop_enabled = true;
int64_t wheel_count = 4;
double max_velocity = 1.5;
std::string robot_name = "tadeo_ecar";

// Arrays
std::vector<double> wheel_positions = {0.0, 0.0, 0.0, 0.0};
std::vector<int64_t> motor_ids = {1, 2, 3, 4};
std::vector<std::string> sensor_topics = {"/scan", "/camera/image_raw", "/imu/data"};

// Nota: ROS2 no soporta parámetros de objeto/mapa directamente
// Usar múltiples parámetros o archivos YAML para estructuras complejas
```

### Jerarquía de Parámetros

```yaml
# Estructura jerárquica de parámetros del eCar
wheel_controller_node:
  ros__parameters:
    # Parámetros de hardware
    hardware:
      wheel_base: 1.2
      track_width: 0.8
      wheel_radius: 0.15
      gear_ratio: 10.0
    
    # Parámetros de control
    control:
      max_linear_velocity: 2.0
      max_angular_velocity: 1.0
      acceleration_limit: 1.5
      
    # Parámetros PID
    pid:
      linear:
        kp: 1.0
        ki: 0.1
        kd: 0.05
      angular:
        kp: 2.0
        ki: 0.2
        kd: 0.1
    
    # Configuración de sensores
    sensors:
      use_imu: true
      use_encoders: true
      encoder_resolution: 1024
```

### Convenciones de Nombres

```cpp
// Convenciones para nombres de parámetros
std::string good_names[] = {
    "max_velocity",           // snake_case
    "wheel_base",            // descriptivo
    "pid_kp",                // específico
    "safety.emergency_distance"  // namespaced con punto
};

std::string avoid_names[] = {
    "MaxVelocity",           // CamelCase
    "vel",                   // abreviado
    "p1",                    // no descriptivo
    "max-velocity"           // guiones no recomendados
};
```

## Declaración y Uso

### Declaración Básica en C++

```cpp
class ConfigurableNode : public rclcpp::Node
{
public:
    ConfigurableNode() : Node("configurable_node")
    {
        // Declarar parámetros con valores por defecto
        this->declare_parameter("max_velocity", 1.0);
        this->declare_parameter("robot_name", std::string("default_robot"));
        this->declare_parameter("wheel_count", 4);
        this->declare_parameter("debug_mode", false);
        
        // Declarar arrays
        this->declare_parameter("wheel_positions", std::vector<double>{0.0, 0.0, 0.0, 0.0});
        
        // Obtener valores
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        robot_name_ = this->get_parameter("robot_name").as_string();
        wheel_count_ = this->get_parameter("wheel_count").as_int();
        debug_mode_ = this->get_parameter("debug_mode").as_bool();
        
        // Obtener array
        wheel_positions_ = this->get_parameter("wheel_positions").as_double_array();
        
        // Log configuración
        RCLCPP_INFO(this->get_logger(), "Node configured:");
        RCLCPP_INFO(this->get_logger(), "  Robot name: %s", robot_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Max velocity: %.2f m/s", max_velocity_);
        RCLCPP_INFO(this->get_logger(), "  Wheel count: %ld", wheel_count_);
        RCLCPP_INFO(this->get_logger(), "  Debug mode: %s", debug_mode_ ? "ON" : "OFF");
    }

private:
    double max_velocity_;
    std::string robot_name_;
    int64_t wheel_count_;
    bool debug_mode_;
    std::vector<double> wheel_positions_;
};
```

### Declaración con Descriptores

```cpp
class AdvancedParameterNode : public rclcpp::Node
{
public:
    AdvancedParameterNode() : Node("advanced_parameter_node")
    {
        // Declarar con descriptor completo
        rcl_interfaces::msg::ParameterDescriptor max_vel_desc;
        max_vel_desc.description = "Maximum linear velocity in m/s";
        max_vel_desc.additional_constraints = "Must be positive and less than 5.0";
        max_vel_desc.read_only = false;
        
        // Configurar rango válido
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value = 5.0;
        range.step = 0.1;
        max_vel_desc.floating_point_range.push_back(range);
        
        this->declare_parameter("max_velocity", 1.0, max_vel_desc);
        
        // Parámetro read-only
        rcl_interfaces::msg::ParameterDescriptor robot_id_desc;
        robot_id_desc.description = "Unique robot identifier";
        robot_id_desc.read_only = true;
        this->declare_parameter("robot_id", std::string("ecar_001"), robot_id_desc);
        
        // Parámetro con opciones específicas
        rcl_interfaces::msg::ParameterDescriptor mode_desc;
        mode_desc.description = "Robot operation mode";
        mode_desc.additional_constraints = "Valid values: manual, autonomous, maintenance";
        this->declare_parameter("operation_mode", std::string("manual"), mode_desc);
    }
};
```

## Archivos YAML

### Estructura de Archivos YAML

```yaml
# config/ecar_config.yaml
# Configuración completa del eCar

# Configuración global
/**:
  ros__parameters:
    # Información del robot
    robot_name: "tadeo_ecar"
    robot_version: "1.0.0"
    use_sim_time: false

# Configuración específica por nodo
wheel_controller_node:
  ros__parameters:
    # Hardware
    wheel_base: 1.2        # metros
    track_width: 0.8       # metros
    wheel_radius: 0.15     # metros
    wheel_count: 4
    
    # Cinemática 4WD4WS
    max_linear_velocity: 2.0    # m/s
    max_angular_velocity: 1.0   # rad/s
    max_lateral_velocity: 1.0   # m/s (4WS permite movimiento lateral)
    
    # Control PID
    velocity_pid:
      kp: 1.0
      ki: 0.1
      kd: 0.05
      
    steering_pid:
      kp: 2.0
      ki: 0.2
      kd: 0.1
    
    # Límites de seguridad
    acceleration_limit: 1.5     # m/s²
    deceleration_limit: 2.0     # m/s²
    steering_rate_limit: 1.0    # rad/s

lidar_processor_node:
  ros__parameters:
    # Configuración del LiDAR
    topic_name: "/scan"
    frame_id: "laser_frame"
    
    # Filtros
    min_range: 0.1             # metros
    max_range: 30.0            # metros
    angle_min: -3.14159        # radianes
    angle_max: 3.14159         # radianes
    
    # Procesamiento
    noise_threshold: 0.05      # metros
    cluster_tolerance: 0.2     # metros
    min_cluster_size: 3        # puntos

navigation_controller_node:
  ros__parameters:
    # Planificación de rutas
    global_planner: "NavfnPlanner"
    local_planner: "DWBLocalPlanner"
    
    # Tolerancias
    xy_goal_tolerance: 0.2     # metros
    yaw_goal_tolerance: 0.1    # radianes
    
    # Velocidades de navegación
    max_vel_x: 1.5             # m/s
    min_vel_x: 0.1             # m/s
    max_vel_theta: 1.0         # rad/s
    
    # Costmaps
    global_costmap:
      resolution: 0.05         # m/pixel
      width: 100               # metros
      height: 100              # metros
      
    local_costmap:
      resolution: 0.025        # m/pixel
      width: 10                # metros
      height: 10               # metros

safety_monitor_node:
  ros__parameters:
    # Distancias de seguridad
    emergency_stop_distance: 0.3    # metros
    warning_distance: 1.0           # metros
    
    # Timeouts
    sensor_timeout: 2.0             # segundos
    command_timeout: 0.5            # segundos
    
    # Límites del sistema
    max_temperature: 70.0           # °C
    min_battery_voltage: 11.0       # V
    max_current_draw: 20.0          # A
```

### Archivos por Entorno

```yaml
# config/environments/indoor.yaml
# Configuración optimizada para uso interior

wheel_controller_node:
  ros__parameters:
    max_linear_velocity: 0.8    # Velocidad reducida
    max_angular_velocity: 0.6
    acceleration_limit: 1.0     # Aceleración suave

safety_monitor_node:
  ros__parameters:
    emergency_stop_distance: 0.2  # Distancia reducida
    warning_distance: 0.5

lidar_processor_node:
  ros__parameters:
    max_range: 10.0             # Rango reducido para interiores
    noise_threshold: 0.02       # Mayor precisión
```

```yaml
# config/environments/outdoor.yaml
# Configuración para uso exterior

wheel_controller_node:
  ros__parameters:
    max_linear_velocity: 3.0    # Velocidad aumentada
    max_angular_velocity: 1.5
    acceleration_limit: 2.0     # Aceleración más agresiva

safety_monitor_node:
  ros__parameters:
    emergency_stop_distance: 0.5  # Mayor distancia de seguridad
    warning_distance: 2.0

lidar_processor_node:
  ros__parameters:
    max_range: 30.0             # Rango completo
    noise_threshold: 0.1        # Tolerancia al ruido exterior
```

### Uso en Launch Files

```python
# launch/ecar_with_config.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Argumento para seleccionar entorno
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='indoor',
        description='Environment configuration: indoor, outdoor, simulation'
    )
    
    # Argumento para configuración personalizada
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to custom configuration file'
    )
    
    # Rutas de configuración
    package_share = FindPackageShare('tadeo_ecar_config')
    
    # Configuración base
    base_config = PathJoinSubstitution([
        package_share, 'config', 'ecar_config.yaml'
    ])
    
    # Configuración por entorno
    env_config = PathJoinSubstitution([
        package_share, 'config', 'environments', 
        [LaunchConfiguration('environment'), '.yaml']
    ])
    
    # Nodos con configuración
    wheel_controller_node = Node(
        package='tadeo_ecar_control',
        executable='wheel_controller_node',
        name='wheel_controller_node',
        parameters=[
            base_config,
            env_config,
            # Configuración personalizada si se proporciona
            LaunchConfiguration('config_file')
        ]
    )
    
    lidar_processor_node = Node(
        package='tadeo_ecar_perception',
        executable='lidar_processor_node',
        name='lidar_processor_node',
        parameters=[base_config, env_config]
    )
    
    navigation_node = Node(
        package='tadeo_ecar_navigation',
        executable='navigation_controller_node',
        name='navigation_controller_node',
        parameters=[base_config, env_config]
    )
    
    return LaunchDescription([
        environment_arg,
        config_file_arg,
        wheel_controller_node,
        lidar_processor_node,
        navigation_node
    ])
```

## Parámetros Dinámicos

### Callbacks de Parámetros

```cpp
// include/tadeo_ecar_control/dynamic_controller.hpp
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class DynamicController : public rclcpp::Node
{
public:
    DynamicController() : Node("dynamic_controller")
    {
        // Declarar parámetros
        this->declare_parameter("max_velocity", 1.0);
        this->declare_parameter("pid_kp", 1.0);
        this->declare_parameter("pid_ki", 0.1);
        this->declare_parameter("pid_kd", 0.05);
        this->declare_parameter("debug_mode", false);
        
        // Obtener valores iniciales
        updateParameters();
        
        // Registrar callback para cambios de parámetros
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DynamicController::parametersCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Dynamic controller initialized with parameter callbacks");
    }

private:
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";
        
        for (const auto & param : parameters) {
            RCLCPP_INFO(this->get_logger(), "Parameter change: %s", param.get_name().c_str());
            
            if (param.get_name() == "max_velocity") {
                double new_velocity = param.as_double();
                if (new_velocity <= 0.0 || new_velocity > 5.0) {
                    result.successful = false;
                    result.reason = "max_velocity must be between 0 and 5 m/s";
                    return result;
                }
                max_velocity_ = new_velocity;
                RCLCPP_INFO(this->get_logger(), "Updated max_velocity to %.2f m/s", max_velocity_);
                
            } else if (param.get_name() == "pid_kp") {
                double new_kp = param.as_double();
                if (new_kp < 0.0) {
                    result.successful = false;
                    result.reason = "pid_kp must be non-negative";
                    return result;
                }
                pid_kp_ = new_kp;
                updatePIDController();
                
            } else if (param.get_name() == "pid_ki") {
                pid_ki_ = param.as_double();
                updatePIDController();
                
            } else if (param.get_name() == "pid_kd") {
                pid_kd_ = param.as_double();
                updatePIDController();
                
            } else if (param.get_name() == "debug_mode") {
                debug_mode_ = param.as_bool();
                if (debug_mode_) {
                    RCLCPP_INFO(this->get_logger(), "Debug mode ENABLED");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Debug mode DISABLED");
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown parameter: %s", param.get_name().c_str());
            }
        }
        
        return result;
    }
    
    void updateParameters()
    {
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        pid_kp_ = this->get_parameter("pid_kp").as_double();
        pid_ki_ = this->get_parameter("pid_ki").as_double();
        pid_kd_ = this->get_parameter("pid_kd").as_double();
        debug_mode_ = this->get_parameter("debug_mode").as_bool();
    }
    
    void updatePIDController()
    {
        RCLCPP_INFO(this->get_logger(), 
                    "PID parameters updated: Kp=%.3f, Ki=%.3f, Kd=%.3f",
                    pid_kp_, pid_ki_, pid_kd_);
        // Actualizar controlador PID real aquí
    }
    
    // Variables de estado
    double max_velocity_;
    double pid_kp_, pid_ki_, pid_kd_;
    bool debug_mode_;
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};
```

### Monitor de Parámetros

```cpp
class ParameterMonitor : public rclcpp::Node
{
public:
    ParameterMonitor() : Node("parameter_monitor")
    {
        // Timer para verificar cambios de parámetros
        monitor_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&ParameterMonitor::monitorParameters, this));
        
        // Almacenar valores iniciales
        stored_parameters_["max_velocity"] = 1.0;
        stored_parameters_["pid_kp"] = 1.0;
        
        RCLCPP_INFO(this->get_logger(), "Parameter monitor started");
    }

private:
    void monitorParameters()
    {
        // Lista de nodos a monitorear
        std::vector<std::string> node_names = {
            "/wheel_controller_node",
            "/navigation_controller_node",
            "/safety_monitor_node"
        };
        
        for (const auto& node_name : node_names) {
            checkNodeParameters(node_name);
        }
    }
    
    void checkNodeParameters(const std::string& node_name)
    {
        // Cliente para obtener parámetros
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, node_name);
        
        if (parameters_client->wait_for_service(std::chrono::seconds(1))) {
            try {
                // Obtener parámetros específicos
                auto parameters = parameters_client->get_parameters({"max_velocity", "pid_kp"});
                
                for (const auto& param : parameters) {
                    std::string key = node_name + "." + param.get_name();
                    
                    // Verificar si cambió
                    if (stored_parameters_.find(key) != stored_parameters_.end()) {
                        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                            double current_value = param.as_double();
                            double stored_value = stored_parameters_[key];
                            
                            if (std::abs(current_value - stored_value) > 0.001) {
                                RCLCPP_INFO(this->get_logger(),
                                           "Parameter changed: %s.%s: %.3f -> %.3f",
                                           node_name.c_str(), param.get_name().c_str(),
                                           stored_value, current_value);
                                stored_parameters_[key] = current_value;
                            }
                        }
                    } else {
                        // Nuevo parámetro
                        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                            stored_parameters_[key] = param.as_double();
                        }
                    }
                }
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), 
                           "Failed to get parameters from %s: %s", 
                           node_name.c_str(), e.what());
            }
        }
    }
    
    rclcpp::TimerInterface::SharedPtr monitor_timer_;
    std::map<std::string, double> stored_parameters_;
};
```

## Implementación en C++

### Nodo Completo con Parámetros

```cpp
// include/tadeo_ecar_control/configurable_wheel_controller.hpp
#ifndef TADEO_ECAR_CONTROL__CONFIGURABLE_WHEEL_CONTROLLER_HPP_
#define TADEO_ECAR_CONTROL__CONFIGURABLE_WHEEL_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace tadeo_ecar_control
{
class ConfigurableWheelController : public rclcpp::Node
{
public:
    ConfigurableWheelController();

private:
    // Callbacks
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wheelStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void controlLoop();
    
    // Parameter management
    void declareParameters();
    void loadParameters();
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters);
    bool validateParameters(const std::vector<rclcpp::Parameter> & parameters, 
                           std::string& error_msg);
    
    // Control functions
    void updateControllerGains();
    void calculateWheelCommands(const geometry_msgs::msg::Twist& cmd_vel);
    void publishWheelCommands();
    
    // Publishers and Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_cmd_pub_;
    
    // Timer
    rclcpp::TimerInterface::SharedPtr control_timer_;
    
    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    
    // Configuration parameters
    struct HardwareConfig {
        double wheel_base;
        double track_width;
        double wheel_radius;
        std::vector<double> gear_ratios;
    } hardware_config_;
    
    struct ControlConfig {
        double max_linear_velocity;
        double max_angular_velocity;
        double max_lateral_velocity;  // 4WS capability
        double acceleration_limit;
        double steering_rate_limit;
    } control_config_;
    
    struct PIDConfig {
        double kp, ki, kd;
        double max_integral;
        double max_output;
    } velocity_pid_, steering_pid_;
    
    struct SafetyConfig {
        double emergency_deceleration;
        double max_wheel_speed;
        double max_steering_angle;
        bool enable_safety_limits;
    } safety_config_;
    
    // Control state
    geometry_msgs::msg::Twist current_cmd_vel_;
    std::vector<double> current_wheel_speeds_;
    std::vector<double> current_steering_angles_;
    std::vector<double> target_wheel_speeds_;
    std::vector<double> target_steering_angles_;
    
    // PID state
    std::vector<double> velocity_error_integral_;
    std::vector<double> steering_error_integral_;
    std::vector<double> last_velocity_error_;
    std::vector<double> last_steering_error_;
    
    rclcpp::Time last_control_time_;
    bool debug_mode_;
};
}  // namespace tadeo_ecar_control

#endif  // TADEO_ECAR_CONTROL__CONFIGURABLE_WHEEL_CONTROLLER_HPP_
```

```cpp
// src/configurable_wheel_controller.cpp
#include "tadeo_ecar_control/configurable_wheel_controller.hpp"
#include <cmath>

namespace tadeo_ecar_control
{
ConfigurableWheelController::ConfigurableWheelController()
    : Node("configurable_wheel_controller"),
      debug_mode_(false)
{
    // Inicializar vectores
    current_wheel_speeds_.resize(4, 0.0);
    current_steering_angles_.resize(4, 0.0);
    target_wheel_speeds_.resize(4, 0.0);
    target_steering_angles_.resize(4, 0.0);
    velocity_error_integral_.resize(4, 0.0);
    steering_error_integral_.resize(4, 0.0);
    last_velocity_error_.resize(4, 0.0);
    last_steering_error_.resize(4, 0.0);
    
    // Declarar y cargar parámetros
    declareParameters();
    loadParameters();
    
    // Configurar callback de parámetros
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ConfigurableWheelController::parametersCallback, this, std::placeholders::_1));
    
    // Publishers y Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&ConfigurableWheelController::cmdVelCallback, this, std::placeholders::_1));
    
    wheel_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "wheel_states", 10,
        std::bind(&ConfigurableWheelController::wheelStateCallback, this, std::placeholders::_1));
    
    wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_commands", 10);
    steering_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("steering_commands", 10);
    
    // Timer de control
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz
        std::bind(&ConfigurableWheelController::controlLoop, this));
    
    last_control_time_ = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "Configurable wheel controller initialized");
    RCLCPP_INFO(this->get_logger(), "Wheel base: %.2f m, Track width: %.2f m", 
                hardware_config_.wheel_base, hardware_config_.track_width);
}

void ConfigurableWheelController::declareParameters()
{
    // Hardware parameters
    this->declare_parameter("hardware.wheel_base", 1.2);
    this->declare_parameter("hardware.track_width", 0.8);
    this->declare_parameter("hardware.wheel_radius", 0.15);
    this->declare_parameter("hardware.gear_ratios", std::vector<double>{10.0, 10.0, 10.0, 10.0});
    
    // Control parameters
    this->declare_parameter("control.max_linear_velocity", 2.0);
    this->declare_parameter("control.max_angular_velocity", 1.0);
    this->declare_parameter("control.max_lateral_velocity", 1.0);
    this->declare_parameter("control.acceleration_limit", 1.5);
    this->declare_parameter("control.steering_rate_limit", 1.0);
    
    // PID parameters for velocity
    this->declare_parameter("velocity_pid.kp", 1.0);
    this->declare_parameter("velocity_pid.ki", 0.1);
    this->declare_parameter("velocity_pid.kd", 0.05);
    this->declare_parameter("velocity_pid.max_integral", 10.0);
    this->declare_parameter("velocity_pid.max_output", 100.0);
    
    // PID parameters for steering
    this->declare_parameter("steering_pid.kp", 2.0);
    this->declare_parameter("steering_pid.ki", 0.2);
    this->declare_parameter("steering_pid.kd", 0.1);
    this->declare_parameter("steering_pid.max_integral", 5.0);
    this->declare_parameter("steering_pid.max_output", 10.0);
    
    // Safety parameters
    this->declare_parameter("safety.emergency_deceleration", 3.0);
    this->declare_parameter("safety.max_wheel_speed", 50.0);
    this->declare_parameter("safety.max_steering_angle", 1.57);  // 90 degrees
    this->declare_parameter("safety.enable_safety_limits", true);
    
    // Debug
    this->declare_parameter("debug_mode", false);
}

void ConfigurableWheelController::loadParameters()
{
    // Hardware
    hardware_config_.wheel_base = this->get_parameter("hardware.wheel_base").as_double();
    hardware_config_.track_width = this->get_parameter("hardware.track_width").as_double();
    hardware_config_.wheel_radius = this->get_parameter("hardware.wheel_radius").as_double();
    hardware_config_.gear_ratios = this->get_parameter("hardware.gear_ratios").as_double_array();
    
    // Control
    control_config_.max_linear_velocity = this->get_parameter("control.max_linear_velocity").as_double();
    control_config_.max_angular_velocity = this->get_parameter("control.max_angular_velocity").as_double();
    control_config_.max_lateral_velocity = this->get_parameter("control.max_lateral_velocity").as_double();
    control_config_.acceleration_limit = this->get_parameter("control.acceleration_limit").as_double();
    control_config_.steering_rate_limit = this->get_parameter("control.steering_rate_limit").as_double();
    
    // Velocity PID
    velocity_pid_.kp = this->get_parameter("velocity_pid.kp").as_double();
    velocity_pid_.ki = this->get_parameter("velocity_pid.ki").as_double();
    velocity_pid_.kd = this->get_parameter("velocity_pid.kd").as_double();
    velocity_pid_.max_integral = this->get_parameter("velocity_pid.max_integral").as_double();
    velocity_pid_.max_output = this->get_parameter("velocity_pid.max_output").as_double();
    
    // Steering PID
    steering_pid_.kp = this->get_parameter("steering_pid.kp").as_double();
    steering_pid_.ki = this->get_parameter("steering_pid.ki").as_double();
    steering_pid_.kd = this->get_parameter("steering_pid.kd").as_double();
    steering_pid_.max_integral = this->get_parameter("steering_pid.max_integral").as_double();
    steering_pid_.max_output = this->get_parameter("steering_pid.max_output").as_double();
    
    // Safety
    safety_config_.emergency_deceleration = this->get_parameter("safety.emergency_deceleration").as_double();
    safety_config_.max_wheel_speed = this->get_parameter("safety.max_wheel_speed").as_double();
    safety_config_.max_steering_angle = this->get_parameter("safety.max_steering_angle").as_double();
    safety_config_.enable_safety_limits = this->get_parameter("safety.enable_safety_limits").as_bool();
    
    // Debug
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    
    // Actualizar controlador
    updateControllerGains();
}

rcl_interfaces::msg::SetParametersResult ConfigurableWheelController::parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";
    
    // Validar parámetros antes de aplicar
    if (!validateParameters(parameters, result.reason)) {
        result.successful = false;
        return result;
    }
    
    // Aplicar cambios
    bool need_controller_update = false;
    
    for (const auto & param : parameters) {
        std::string name = param.get_name();
        
        if (name.find("velocity_pid") != std::string::npos ||
            name.find("steering_pid") != std::string::npos) {
            need_controller_update = true;
        }
        
        if (name == "debug_mode") {
            debug_mode_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Debug mode: %s", debug_mode_ ? "ON" : "OFF");
        }
        
        RCLCPP_INFO(this->get_logger(), "Parameter updated: %s", name.c_str());
    }
    
    // Actualizar controlador si es necesario
    if (need_controller_update) {
        updateControllerGains();
        RCLCPP_INFO(this->get_logger(), "Controller gains updated");
    }
    
    return result;
}

bool ConfigurableWheelController::validateParameters(
    const std::vector<rclcpp::Parameter> & parameters, 
    std::string& error_msg)
{
    for (const auto & param : parameters) {
        std::string name = param.get_name();
        
        if (name == "control.max_linear_velocity") {
            double value = param.as_double();
            if (value <= 0.0 || value > 10.0) {
                error_msg = "max_linear_velocity must be between 0 and 10 m/s";
                return false;
            }
        } else if (name == "hardware.wheel_base") {
            double value = param.as_double();
            if (value <= 0.0 || value > 5.0) {
                error_msg = "wheel_base must be between 0 and 5 meters";
                return false;
            }
        } else if (name.find("pid.kp") != std::string::npos ||
                   name.find("pid.ki") != std::string::npos ||
                   name.find("pid.kd") != std::string::npos) {
            double value = param.as_double();
            if (value < 0.0) {
                error_msg = "PID gains must be non-negative";
                return false;
            }
        }
    }
    
    return true;
}

void ConfigurableWheelController::updateControllerGains()
{
    // Recargar parámetros PID
    velocity_pid_.kp = this->get_parameter("velocity_pid.kp").as_double();
    velocity_pid_.ki = this->get_parameter("velocity_pid.ki").as_double();
    velocity_pid_.kd = this->get_parameter("velocity_pid.kd").as_double();
    
    steering_pid_.kp = this->get_parameter("steering_pid.kp").as_double();
    steering_pid_.ki = this->get_parameter("steering_pid.ki").as_double();
    steering_pid_.kd = this->get_parameter("steering_pid.kd").as_double();
    
    // Reset integrators cuando cambian las ganancias
    std::fill(velocity_error_integral_.begin(), velocity_error_integral_.end(), 0.0);
    std::fill(steering_error_integral_.begin(), steering_error_integral_.end(), 0.0);
    
    if (debug_mode_) {
        RCLCPP_INFO(this->get_logger(), 
                    "PID gains updated - Velocity: Kp=%.3f Ki=%.3f Kd=%.3f, Steering: Kp=%.3f Ki=%.3f Kd=%.3f",
                    velocity_pid_.kp, velocity_pid_.ki, velocity_pid_.kd,
                    steering_pid_.kp, steering_pid_.ki, steering_pid_.kd);
    }
}

void ConfigurableWheelController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    current_cmd_vel_ = *msg;
    
    // Aplicar límites de seguridad
    if (safety_config_.enable_safety_limits) {
        current_cmd_vel_.linear.x = std::clamp(current_cmd_vel_.linear.x, 
                                              -control_config_.max_linear_velocity,
                                               control_config_.max_linear_velocity);
        current_cmd_vel_.linear.y = std::clamp(current_cmd_vel_.linear.y,
                                              -control_config_.max_lateral_velocity,
                                               control_config_.max_lateral_velocity);
        current_cmd_vel_.angular.z = std::clamp(current_cmd_vel_.angular.z,
                                               -control_config_.max_angular_velocity,
                                                control_config_.max_angular_velocity);
    }
    
    if (debug_mode_) {
        RCLCPP_DEBUG(this->get_logger(), 
                     "Received cmd_vel: linear=[%.2f, %.2f] angular=%.2f",
                     current_cmd_vel_.linear.x, current_cmd_vel_.linear.y, current_cmd_vel_.angular.z);
    }
}

void ConfigurableWheelController::wheelStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Actualizar estado actual de ruedas
    if (msg->velocity.size() >= 4) {
        for (size_t i = 0; i < 4; ++i) {
            current_wheel_speeds_[i] = msg->velocity[i];
        }
    }
    
    // Actualizar ángulos de steering si están disponibles
    if (msg->position.size() >= 8) {  // 4 wheels + 4 steering angles
        for (size_t i = 0; i < 4; ++i) {
            current_steering_angles_[i] = msg->position[i + 4];
        }
    }
}

void ConfigurableWheelController::controlLoop()
{
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;
    
    // Calcular comandos de ruedas basados en cinemática 4WD4WS
    calculateWheelCommands(current_cmd_vel_);
    
    // Aplicar control PID
    for (size_t i = 0; i < 4; ++i) {
        // Control de velocidad
        double velocity_error = target_wheel_speeds_[i] - current_wheel_speeds_[i];
        velocity_error_integral_[i] += velocity_error * dt;
        
        // Anti-windup
        velocity_error_integral_[i] = std::clamp(velocity_error_integral_[i],
                                               -velocity_pid_.max_integral,
                                                velocity_pid_.max_integral);
        
        double velocity_derivative = (velocity_error - last_velocity_error_[i]) / dt;
        last_velocity_error_[i] = velocity_error;
        
        double velocity_output = velocity_pid_.kp * velocity_error +
                               velocity_pid_.ki * velocity_error_integral_[i] +
                               velocity_pid_.kd * velocity_derivative;
        
        velocity_output = std::clamp(velocity_output, -velocity_pid_.max_output, velocity_pid_.max_output);
        target_wheel_speeds_[i] = velocity_output;
        
        // Control de steering
        double steering_error = target_steering_angles_[i] - current_steering_angles_[i];
        steering_error_integral_[i] += steering_error * dt;
        
        steering_error_integral_[i] = std::clamp(steering_error_integral_[i],
                                               -steering_pid_.max_integral,
                                                steering_pid_.max_integral);
        
        double steering_derivative = (steering_error - last_steering_error_[i]) / dt;
        last_steering_error_[i] = steering_error;
        
        double steering_output = steering_pid_.kp * steering_error +
                               steering_pid_.ki * steering_error_integral_[i] +
                               steering_pid_.kd * steering_derivative;
        
        steering_output = std::clamp(steering_output, -steering_pid_.max_output, steering_pid_.max_output);
        target_steering_angles_[i] = current_steering_angles_[i] + steering_output * dt;
        
        // Aplicar límites de steering
        target_steering_angles_[i] = std::clamp(target_steering_angles_[i],
                                              -safety_config_.max_steering_angle,
                                               safety_config_.max_steering_angle);
    }
    
    // Publicar comandos
    publishWheelCommands();
}

void ConfigurableWheelController::calculateWheelCommands(const geometry_msgs::msg::Twist& cmd_vel)
{
    // Cinemática 4WD4WS (simplificada)
    double vx = cmd_vel.linear.x;
    double vy = cmd_vel.linear.y;
    double omega = cmd_vel.angular.z;
    
    double L = hardware_config_.wheel_base;
    double W = hardware_config_.track_width;
    double R = hardware_config_.wheel_radius;
    
    // Velocidades de ruedas (4WD)
    double v_fl = vx - omega * W/2;  // Front Left
    double v_fr = vx + omega * W/2;  // Front Right
    double v_rl = vx - omega * W/2;  // Rear Left
    double v_rr = vx + omega * W/2;  // Rear Right
    
    // Convertir a velocidades angulares
    target_wheel_speeds_[0] = v_fl / R;
    target_wheel_speeds_[1] = v_fr / R;
    target_wheel_speeds_[2] = v_rl / R;
    target_wheel_speeds_[3] = v_rr / R;
    
    // Ángulos de steering (4WS)
    if (std::abs(omega) > 0.001) {
        // Ackermann steering modificado para 4WS
        double turn_radius = vx / omega;
        
        target_steering_angles_[0] = atan(L / (turn_radius - W/2));  // Front Left
        target_steering_angles_[1] = atan(L / (turn_radius + W/2));  // Front Right
        target_steering_angles_[2] = -atan(L / (turn_radius - W/2)); // Rear Left (opuesto)
        target_steering_angles_[3] = -atan(L / (turn_radius + W/2)); // Rear Right (opuesto)
    } else if (std::abs(vy) > 0.001) {
        // Movimiento lateral puro (4WS)
        double lateral_angle = atan2(vy, vx);
        for (size_t i = 0; i < 4; ++i) {
            target_steering_angles_[i] = lateral_angle;
        }
    } else {
        // Sin giro - ruedas rectas
        for (size_t i = 0; i < 4; ++i) {
            target_steering_angles_[i] = 0.0;
        }
    }
}

void ConfigurableWheelController::publishWheelCommands()
{
    // Publicar comandos de velocidad
    auto wheel_cmd_msg = std_msgs::msg::Float64MultiArray();
    wheel_cmd_msg.data = target_wheel_speeds_;
    wheel_cmd_pub_->publish(wheel_cmd_msg);
    
    // Publicar comandos de steering
    auto steering_cmd_msg = std_msgs::msg::Float64MultiArray();
    steering_cmd_msg.data = target_steering_angles_;
    steering_cmd_pub_->publish(steering_cmd_msg);
    
    if (debug_mode_) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Published commands - Wheels: [%.2f, %.2f, %.2f, %.2f], Steering: [%.3f, %.3f, %.3f, %.3f]",
                     target_wheel_speeds_[0], target_wheel_speeds_[1], target_wheel_speeds_[2], target_wheel_speeds_[3],
                     target_steering_angles_[0], target_steering_angles_[1], target_steering_angles_[2], target_steering_angles_[3]);
    }
}
}  // namespace tadeo_ecar_control

// Main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_control::ConfigurableWheelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Implementación en Python

### Nodo con Parámetros en Python

```python
#!/usr/bin/env python3
# scripts/adaptive_safety_monitor.py

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import json
import time

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class AdaptiveSafetyMonitor(Node):
    def __init__(self):
        super().__init__('adaptive_safety_monitor')
        
        # Declarar parámetros con descriptores
        self.declare_parameters()
        
        # Cargar configuración inicial
        self.load_parameters()
        
        # Configurar callback de parámetros
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Publishers
        self.emergency_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.safety_status_pub = self.create_publisher(String, 'safety_status', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, 10)
        
        # Timers
        self.safety_timer = self.create_timer(0.1, self.safety_check_loop)  # 10 Hz
        self.diagnostics_timer = self.create_timer(1.0, self.publish_diagnostics)  # 1 Hz
        
        # Estado interno
        self.last_cmd_vel_time = None
        self.last_scan_time = None
        self.current_min_distance = float('inf')
        self.current_battery_level = 100.0
        self.emergency_active = False
        self.safety_violations = []
        
        self.get_logger().info('Adaptive Safety Monitor initialized')
        self.log_configuration()
    
    def declare_parameters(self):
        """Declarar todos los parámetros con descriptores"""
        
        # Distancias de seguridad
        self.declare_parameter('safety_distances.emergency_stop', 0.3,
                             descriptor='Emergency stop distance in meters')
        self.declare_parameter('safety_distances.warning', 1.0,
                             descriptor='Warning distance in meters')
        self.declare_parameter('safety_distances.slow_down', 2.0,
                             descriptor='Slow down distance in meters')
        
        # Timeouts
        self.declare_parameter('timeouts.cmd_vel_timeout', 0.5,
                             descriptor='Command velocity timeout in seconds')
        self.declare_parameter('timeouts.sensor_timeout', 2.0,
                             descriptor='Sensor timeout in seconds')
        self.declare_parameter('timeouts.battery_timeout', 5.0,
                             descriptor='Battery timeout in seconds')
        
        # Límites de batería
        self.declare_parameter('battery.critical_level', 10.0,
                             descriptor='Critical battery level percentage')
        self.declare_parameter('battery.warning_level', 20.0,
                             descriptor='Warning battery level percentage')
        self.declare_parameter('battery.min_voltage', 11.0,
                             descriptor='Minimum battery voltage')
        
        # Límites de velocidad
        self.declare_parameter('velocity_limits.max_linear', 2.0,
                             descriptor='Maximum linear velocity in m/s')
        self.declare_parameter('velocity_limits.max_angular', 1.0,
                             descriptor='Maximum angular velocity in rad/s')
        self.declare_parameter('velocity_limits.emergency_decel', 3.0,
                             descriptor='Emergency deceleration in m/s²')
        
        # Configuración adaptiva
        self.declare_parameter('adaptive.enable_adaptive_distances', True,
                             descriptor='Enable adaptive safety distances based on speed')
        self.declare_parameter('adaptive.speed_factor', 1.5,
                             descriptor='Factor to multiply distances by speed')
        self.declare_parameter('adaptive.min_distance_factor', 0.5,
                             descriptor='Minimum distance factor')
        
        # Debug y logging
        self.declare_parameter('debug.verbose_logging', False,
                             descriptor='Enable verbose debug logging')
        self.declare_parameter('debug.publish_diagnostics', True,
                             descriptor='Publish diagnostic messages')
        self.declare_parameter('debug.log_violations', True,
                             descriptor='Log safety violations')
    
    def load_parameters(self):
        """Cargar todos los parámetros"""
        
        # Distancias de seguridad
        self.emergency_distance = self.get_parameter('safety_distances.emergency_stop').value
        self.warning_distance = self.get_parameter('safety_distances.warning').value
        self.slow_down_distance = self.get_parameter('safety_distances.slow_down').value
        
        # Timeouts
        self.cmd_vel_timeout = self.get_parameter('timeouts.cmd_vel_timeout').value
        self.sensor_timeout = self.get_parameter('timeouts.sensor_timeout').value
        self.battery_timeout = self.get_parameter('timeouts.battery_timeout').value
        
        # Batería
        self.critical_battery = self.get_parameter('battery.critical_level').value
        self.warning_battery = self.get_parameter('battery.warning_level').value
        self.min_voltage = self.get_parameter('battery.min_voltage').value
        
        # Velocidad
        self.max_linear_vel = self.get_parameter('velocity_limits.max_linear').value
        self.max_angular_vel = self.get_parameter('velocity_limits.max_angular').value
        self.emergency_decel = self.get_parameter('velocity_limits.emergency_decel').value
        
        # Adaptivo
        self.adaptive_distances = self.get_parameter('adaptive.enable_adaptive_distances').value
        self.speed_factor = self.get_parameter('adaptive.speed_factor').value
        self.min_distance_factor = self.get_parameter('adaptive.min_distance_factor').value
        
        # Debug
        self.verbose_logging = self.get_parameter('debug.verbose_logging').value
        self.publish_diag = self.get_parameter('debug.publish_diagnostics').value
        self.log_violations = self.get_parameter('debug.log_violations').value
    
    def parameters_callback(self, params):
        """Callback para cambios de parámetros"""
        result = SetParametersResult()
        result.successful = True
        result.reason = ""
        
        # Validar parámetros
        validation_errors = self.validate_parameters(params)
        if validation_errors:
            result.successful = False
            result.reason = "; ".join(validation_errors)
            return result
        
        # Aplicar cambios
        changes = []
        for param in params:
            old_value = self.get_parameter(param.name).value
            changes.append(f"{param.name}: {old_value} -> {param.value}")
        
        # Recargar parámetros
        self.load_parameters()
        
        # Log cambios
        if self.verbose_logging:
            self.get_logger().info(f"Parameters updated: {'; '.join(changes)}")
        
        # Recalcular distancias adaptivas si es necesario
        self.update_adaptive_distances()
        
        return result
    
    def validate_parameters(self, params):
        """Validar parámetros antes de aplicar"""
        errors = []
        
        for param in params:
            name = param.name
            value = param.value
            
            if 'distance' in name and isinstance(value, float):
                if value < 0 or value > 10:
                    errors.append(f"{name} must be between 0 and 10 meters")
            
            elif 'timeout' in name and isinstance(value, float):
                if value < 0.1 or value > 30:
                    errors.append(f"{name} must be between 0.1 and 30 seconds")
            
            elif 'level' in name and 'battery' in name and isinstance(value, float):
                if value < 0 or value > 100:
                    errors.append(f"{name} must be between 0 and 100 percent")
            
            elif 'max_linear' in name and isinstance(value, float):
                if value < 0 or value > 10:
                    errors.append(f"{name} must be between 0 and 10 m/s")
            
            elif 'max_angular' in name and isinstance(value, float):
                if value < 0 or value > 5:
                    errors.append(f"{name} must be between 0 and 5 rad/s")
        
        return errors
    
    def log_configuration(self):
        """Log configuración actual"""
        config = {
            'safety_distances': {
                'emergency': self.emergency_distance,
                'warning': self.warning_distance,
                'slow_down': self.slow_down_distance
            },
            'timeouts': {
                'cmd_vel': self.cmd_vel_timeout,
                'sensor': self.sensor_timeout,
                'battery': self.battery_timeout
            },
            'battery_limits': {
                'critical': self.critical_battery,
                'warning': self.warning_battery,
                'min_voltage': self.min_voltage
            },
            'adaptive': {
                'enabled': self.adaptive_distances,
                'speed_factor': self.speed_factor
            }
        }
        
        self.get_logger().info(f"Safety monitor configuration: {json.dumps(config, indent=2)}")
    
    def cmd_vel_callback(self, msg):
        """Callback para comandos de velocidad"""
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Verificar límites de velocidad
        if abs(msg.linear.x) > self.max_linear_vel:
            self.add_violation(f"Linear velocity {msg.linear.x:.2f} exceeds limit {self.max_linear_vel:.2f}")
        
        if abs(msg.angular.z) > self.max_angular_vel:
            self.add_violation(f"Angular velocity {msg.angular.z:.2f} exceeds limit {self.max_angular_vel:.2f}")
    
    def scan_callback(self, msg):
        """Callback para datos LiDAR"""
        self.last_scan_time = self.get_clock().now()
        
        # Encontrar distancia mínima
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if valid_ranges:
            self.current_min_distance = min(valid_ranges)
        else:
            self.current_min_distance = float('inf')
    
    def battery_callback(self, msg):
        """Callback para estado de batería"""
        self.current_battery_level = msg.percentage
        
        # Verificar niveles críticos
        if msg.percentage <= self.critical_battery:
            self.add_violation(f"Critical battery level: {msg.percentage:.1f}%")
        elif msg.percentage <= self.warning_battery:
            self.add_violation(f"Low battery level: {msg.percentage:.1f}%")
        
        if msg.voltage < self.min_voltage:
            self.add_violation(f"Low battery voltage: {msg.voltage:.2f}V")
    
    def safety_check_loop(self):
        """Loop principal de verificación de seguridad"""
        current_time = self.get_clock().now()
        
        # Verificar timeouts
        self.check_timeouts(current_time)
        
        # Verificar distancias
        self.check_distances()
        
        # Determinar si activar emergencia
        should_emergency = self.should_trigger_emergency()
        
        if should_emergency and not self.emergency_active:
            self.trigger_emergency()
        elif not should_emergency and self.emergency_active:
            self.clear_emergency()
        
        # Publicar estado
        self.publish_safety_status()
        
        # Limpiar violaciones antiguas (más de 5 segundos)
        self.cleanup_old_violations(current_time)
    
    def check_timeouts(self, current_time):
        """Verificar timeouts de comunicación"""
        
        # Timeout de cmd_vel
        if self.last_cmd_vel_time is not None:
            cmd_vel_age = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9
            if cmd_vel_age > self.cmd_vel_timeout:
                self.add_violation(f"Command velocity timeout: {cmd_vel_age:.1f}s")
        
        # Timeout de sensor
        if self.last_scan_time is not None:
            scan_age = (current_time - self.last_scan_time).nanoseconds / 1e9
            if scan_age > self.sensor_timeout:
                self.add_violation(f"LiDAR sensor timeout: {scan_age:.1f}s")
    
    def check_distances(self):
        """Verificar distancias de seguridad"""
        if self.current_min_distance == float('inf'):
            return
        
        # Calcular distancias adaptivas
        emergency_dist, warning_dist = self.get_adaptive_distances()
        
        if self.current_min_distance <= emergency_dist:
            self.add_violation(f"Emergency distance violation: {self.current_min_distance:.2f}m < {emergency_dist:.2f}m")
        elif self.current_min_distance <= warning_dist:
            self.add_violation(f"Warning distance violation: {self.current_min_distance:.2f}m < {warning_dist:.2f}m")
    
    def get_adaptive_distances(self):
        """Calcular distancias adaptivas basadas en velocidad"""
        if not self.adaptive_distances:
            return self.emergency_distance, self.warning_distance
        
        # Estimar velocidad basada en timeouts (simplificado)
        # En implementación real, usar odometría
        estimated_speed = 1.0  # m/s por defecto
        
        # Ajustar distancias por velocidad
        speed_multiplier = max(self.min_distance_factor, estimated_speed * self.speed_factor)
        
        adaptive_emergency = self.emergency_distance * speed_multiplier
        adaptive_warning = self.warning_distance * speed_multiplier
        
        return adaptive_emergency, adaptive_warning
    
    def update_adaptive_distances(self):
        """Actualizar distancias adaptivas cuando cambian parámetros"""
        if self.adaptive_distances:
            emergency_dist, warning_dist = self.get_adaptive_distances()
            if self.verbose_logging:
                self.get_logger().info(
                    f"Adaptive distances updated: emergency={emergency_dist:.2f}m, warning={warning_dist:.2f}m")
    
    def add_violation(self, violation):
        """Agregar violación de seguridad"""
        current_time = self.get_clock().now()
        
        # Evitar duplicados recientes
        for v in self.safety_violations:
            if v['message'] == violation and (current_time - v['time']).nanoseconds < 1e9:  # 1 segundo
                return
        
        self.safety_violations.append({
            'message': violation,
            'time': current_time
        })
        
        if self.log_violations:
            self.get_logger().warn(f"Safety violation: {violation}")
    
    def should_trigger_emergency(self):
        """Determinar si activar parada de emergencia"""
        current_time = self.get_clock().now()
        
        # Verificar violaciones recientes (últimos 2 segundos)
        recent_violations = [
            v for v in self.safety_violations 
            if (current_time - v['time']).nanoseconds < 2e9
        ]
        
        # Criterios de emergencia
        emergency_keywords = ['Emergency', 'Critical', 'timeout']
        
        for violation in recent_violations:
            if any(keyword in violation['message'] for keyword in emergency_keywords):
                return True
        
        return False
    
    def trigger_emergency(self):
        """Activar parada de emergencia"""
        self.emergency_active = True
        
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)
        
        self.get_logger().error("EMERGENCY STOP ACTIVATED!")
    
    def clear_emergency(self):
        """Limpiar parada de emergencia"""
        self.emergency_active = False
        
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_pub.publish(emergency_msg)
        
        self.get_logger().info("Emergency stop cleared")
    
    def cleanup_old_violations(self, current_time):
        """Limpiar violaciones antiguas"""
        self.safety_violations = [
            v for v in self.safety_violations
            if (current_time - v['time']).nanoseconds < 5e9  # 5 segundos
        ]
    
    def publish_safety_status(self):
        """Publicar estado de seguridad"""
        status = {
            'emergency_active': self.emergency_active,
            'min_distance': self.current_min_distance,
            'battery_level': self.current_battery_level,
            'active_violations': len(self.safety_violations),
            'timestamp': time.time()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.safety_status_pub.publish(status_msg)
    
    def publish_diagnostics(self):
        """Publicar diagnósticos"""
        if not self.publish_diag:
            return
        
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Diagnóstico general
        safety_diag = DiagnosticStatus()
        safety_diag.name = "Safety Monitor"
        safety_diag.hardware_id = "safety_system"
        
        if self.emergency_active:
            safety_diag.level = DiagnosticStatus.ERROR
            safety_diag.message = "Emergency stop active"
        elif len(self.safety_violations) > 0:
            safety_diag.level = DiagnosticStatus.WARN
            safety_diag.message = f"{len(self.safety_violations)} active violations"
        else:
            safety_diag.level = DiagnosticStatus.OK
            safety_diag.message = "All systems nominal"
        
        # Agregar valores
        from diagnostic_msgs.msg import KeyValue
        
        safety_diag.values.append(KeyValue(key="min_distance", value=str(self.current_min_distance)))
        safety_diag.values.append(KeyValue(key="battery_level", value=str(self.current_battery_level)))
        safety_diag.values.append(KeyValue(key="emergency_active", value=str(self.emergency_active)))
        safety_diag.values.append(KeyValue(key="violation_count", value=str(len(self.safety_violations))))
        
        diag_array.status.append(safety_diag)
        self.diagnostics_pub.publish(diag_array)

def main(args=None):
    rclpy.init(args=args)
    
    monitor = AdaptiveSafetyMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Configuración YAML para el Monitor

```yaml
# config/safety_monitor_config.yaml
adaptive_safety_monitor:
  ros__parameters:
    # Distancias de seguridad básicas
    safety_distances:
      emergency_stop: 0.3      # metros
      warning: 1.0             # metros  
      slow_down: 2.0           # metros
    
    # Timeouts de comunicación
    timeouts:
      cmd_vel_timeout: 0.5     # segundos
      sensor_timeout: 2.0      # segundos
      battery_timeout: 5.0     # segundos
    
    # Límites de batería
    battery:
      critical_level: 10.0     # porcentaje
      warning_level: 20.0      # porcentaje
      min_voltage: 11.0        # voltios
    
    # Límites de velocidad
    velocity_limits:
      max_linear: 2.0          # m/s
      max_angular: 1.0         # rad/s
      emergency_decel: 3.0     # m/s²
    
    # Configuración adaptiva
    adaptive:
      enable_adaptive_distances: true
      speed_factor: 1.5        # Multiplicador por velocidad
      min_distance_factor: 0.5 # Factor mínimo
    
    # Debug y logging
    debug:
      verbose_logging: false
      publish_diagnostics: true
      log_violations: true
```

## Configuración del eCar

### Configuración Jerárquica Completa

```yaml
# config/ecar_complete_config.yaml
# Configuración completa del sistema eCar

# Configuración global para todos los nodos
/**:
  ros__parameters:
    use_sim_time: false
    robot_name: "tadeo_ecar"
    robot_id: "ecar_001"
    
    # Frame IDs globales
    base_frame: "base_link"
    odom_frame: "odom"
    map_frame: "map"
    
    # QoS profiles globales
    qos_sensor_data:
      reliability: "best_effort"
      history: "keep_last"
      depth: 10
    
    qos_control_commands:
      reliability: "reliable"
      history: "keep_last"
      depth: 1

# Control 4WD4WS
wheel_controller_node:
  ros__parameters:
    # Hardware del eCar
    hardware:
      wheel_base: 1.2          # metros - distancia entre ejes
      track_width: 0.8         # metros - ancho de vía
      wheel_radius: 0.15       # metros - radio de ruedas
      wheel_count: 4           # número de ruedas
      gear_ratios: [10.0, 10.0, 10.0, 10.0]  # ratios de engranajes por rueda
    
    # Cinemática 4WD4WS
    kinematics:
      max_linear_velocity: 2.0     # m/s - velocidad lineal máxima
      max_angular_velocity: 1.0    # rad/s - velocidad angular máxima
      max_lateral_velocity: 1.0    # m/s - velocidad lateral (4WS)
      acceleration_limit: 1.5      # m/s² - aceleración máxima
      steering_rate_limit: 1.0     # rad/s - velocidad de giro de ruedas
      
    # Control PID para velocidad
    velocity_pid:
      kp: 1.0
      ki: 0.1
      kd: 0.05
      max_integral: 10.0
      max_output: 100.0
      
    # Control PID para dirección
    steering_pid:
      kp: 2.0
      ki: 0.2
      kd: 0.1
      max_integral: 5.0
      max_output: 10.0
    
    # Límites de seguridad
    safety:
      max_wheel_speed: 50.0        # rad/s
      max_steering_angle: 1.57     # radianes (90°)
      emergency_deceleration: 3.0   # m/s²
      enable_safety_limits: true
    
    # Configuración de topics
    topics:
      cmd_vel_topic: "cmd_vel"
      wheel_states_topic: "wheel_states"
      wheel_commands_topic: "wheel_commands"
      steering_commands_topic: "steering_commands"
    
    # Debug
    debug_mode: false
    publish_diagnostics: true

# Percepción LiDAR
lidar_processor_node:
  ros__parameters:
    # Configuración del sensor
    sensor:
      topic_name: "/scan"
      frame_id: "laser_frame"
      expected_frequency: 10.0     # Hz
      
    # Rango de medición
    range:
      min_range: 0.1              # metros
      max_range: 30.0             # metros
      angle_min: -3.14159         # radianes
      angle_max: 3.14159          # radianes
      
    # Filtros de ruido
    filters:
      noise_threshold: 0.05        # metros
      median_filter_size: 3        # puntos
      range_filter_enabled: true
      
    # Detección de obstáculos
    obstacle_detection:
      cluster_tolerance: 0.2       # metros
      min_cluster_size: 3          # puntos
      max_cluster_size: 1000       # puntos
      
    # Procesamiento
    processing:
      downsample_factor: 1         # 1 = sin downsample
      intensity_filter_enabled: false
      
    # Debug
    publish_markers: true
    debug_mode: false

# Cámara
camera_processor_node:
  ros__parameters:
    # Configuración de hardware
    camera:
      device_id: 0
      width: 640
      height: 480
      fps: 30.0
      frame_id: "camera_frame"
      
    # Calibración
    calibration:
      fx: 500.0    # focal length x
      fy: 500.0    # focal length y
      cx: 320.0    # principal point x
      cy: 240.0    # principal point y
      k1: 0.0      # distortion coefficients
      k2: 0.0
      k3: 0.0
      p1: 0.0
      p2: 0.0
      
    # Procesamiento de imagen
    image_processing:
      auto_exposure: true
      brightness: 0
      contrast: 0
      saturation: 0
      
    # Topics
    topics:
      image_raw: "camera/image_raw"
      camera_info: "camera/camera_info"
      processed_image: "camera/processed_image"

# Navegación
navigation_controller_node:
  ros__parameters:
    # Planificadores
    planners:
      global_planner: "NavfnPlanner"
      local_planner: "DWBLocalPlanner"
      recovery_behaviors: ["spin", "back_up", "wait"]
      
    # Tolerancias de navegación
    tolerances:
      xy_goal_tolerance: 0.2       # metros
      yaw_goal_tolerance: 0.1      # radianes
      goal_check_frequency: 2.0    # Hz
      
    # Parámetros de velocidad
    velocity:
      max_vel_x: 1.5              # m/s
      min_vel_x: 0.1              # m/s
      max_vel_theta: 1.0          # rad/s
      min_vel_theta: 0.1          # rad/s
      
    # Costmap global
    global_costmap:
      resolution: 0.05            # m/pixel
      width: 100                  # metros
      height: 100                 # metros
      origin_x: -50.0             # metros
      origin_y: -50.0             # metros
      update_frequency: 1.0       # Hz
      publish_frequency: 0.5      # Hz
      
    # Costmap local
    local_costmap:
      resolution: 0.025           # m/pixel
      width: 10                   # metros
      height: 10                  # metros
      update_frequency: 5.0       # Hz
      publish_frequency: 2.0      # Hz
      
    # Behavior Trees
    behavior_tree:
      bt_loop_duration: 10        # ms
      default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"

# Monitor de Seguridad
safety_monitor_node:
  ros__parameters:
    # Distancias de seguridad
    safety_distances:
      emergency_stop: 0.3         # metros
      warning: 1.0                # metros
      slow_down: 2.0              # metros
      
    # Timeouts críticos
    timeouts:
      cmd_vel_timeout: 0.5        # segundos
      sensor_timeout: 2.0         # segundos
      navigation_timeout: 30.0    # segundos
      
    # Límites del sistema
    system_limits:
      max_temperature: 70.0       # °C
      min_battery_voltage: 11.0   # V
      max_current_draw: 20.0      # A
      max_cpu_usage: 80.0         # %
      max_memory_usage: 85.0      # %
      
    # Configuración de batería
    battery:
      critical_level: 10.0        # %
      warning_level: 20.0         # %
      charging_level: 90.0        # %
      
    # Monitoreo adaptivo
    adaptive_monitoring:
      enable_adaptive_distances: true
      speed_factor: 1.5
      environment_factor: 1.0     # 1.0=indoor, 1.5=outdoor
      
    # Respuesta a emergencias
    emergency_response:
      auto_recovery: true
      recovery_timeout: 10.0      # segundos
      max_recovery_attempts: 3

# Localización
localization_node:
  ros__parameters:
    # EKF (Extended Kalman Filter)
    ekf:
      frequency: 30.0             # Hz
      two_d_mode: true
      
    # Sensores para fusión
    sensor_fusion:
      use_odometry: true
      use_imu: true
      use_gps: false              # Típicamente false para uso interior
      
    # Configuración de odometría
    odometry:
      topic: "odom"
      differential: false         # false para 4WD4WS
      relative: false
      
    # Configuración IMU
    imu:
      topic: "imu/data"
      differential: false
      relative: true
      remove_gravitational_acceleration: true
      
    # Parámetros del proceso
    process_noise:
      x: 0.05
      y: 0.05
      z: 0.06
      roll: 0.05
      pitch: 0.05
      yaw: 0.06
      vx: 0.1
      vy: 0.1
      vz: 0.1
      vroll: 0.1
      vpitch: 0.1
      vyaw: 0.1

# SLAM
slam_node:
  ros__parameters:
    # Configuración general
    slam:
      solver_plugin: "solver_plugins::CeresSolver"
      ceres_linear_solver: "SPARSE_NORMAL_CHOLESKY"
      ceres_preconditioner: "SCHUR_JACOBI"
      ceres_trust_strategy: "LEVENBERG_MARQUARDT"
      
    # Parámetros del mapa
    map:
      resolution: 0.05            # m/pixel
      publish_period: 2.0         # segundos
      
    # Matcher de scan
    scan_matcher:
      maximum_travel_distance: 5.0     # metros
      maximum_time_interval: 30.0      # segundos
      minimum_travel_distance: 0.5     # metros
      minimum_time_interval: 1.0       # segundos
      
    # Loop closure
    loop_closure:
      do_loop_closing: true
      loop_match_minimum_response_coarse: 0.35
      loop_match_maximum_distance_coarse: 4.0
      
    # Correlación
    correlation:
      correlation_search_space_dimension: 0.5
      correlation_search_space_resolution: 0.01
      correlation_search_space_smear_deviation: 0.1

# Configuración de diagnósticos
diagnostic_aggregator:
  ros__parameters:
    analyzers:
      # Analizador de sensores
      sensors:
        type: "diagnostic_aggregator/GenericAnalyzer"
        path: "Sensors"
        contains: ["lidar", "camera", "imu", "battery"]
        
      # Analizador de control
      control:
        type: "diagnostic_aggregator/GenericAnalyzer"
        path: "Control"
        contains: ["wheel", "steering", "motor"]
        
      # Analizador de navegación
      navigation:
        type: "diagnostic_aggregator/GenericAnalyzer"
        path: "Navigation"
        contains: ["navigation", "planner", "costmap"]
        
      # Analizador de seguridad
      safety:
        type: "diagnostic_aggregator/GenericAnalyzer"
        path: "Safety"
        contains: ["safety", "emergency", "collision"]
```

### Perfiles de Configuración

```yaml
# config/profiles/indoor_mode.yaml
# Sobrescribir parámetros para uso interior

wheel_controller_node:
  ros__parameters:
    kinematics:
      max_linear_velocity: 0.8    # Reducida para interiores
      max_angular_velocity: 0.6
      acceleration_limit: 1.0     # Más suave
      
safety_monitor_node:
  ros__parameters:
    safety_distances:
      emergency_stop: 0.2         # Distancias reducidas
      warning: 0.5
      slow_down: 1.0
      
    adaptive_monitoring:
      environment_factor: 1.0     # Factor interior

navigation_controller_node:
  ros__parameters:
    velocity:
      max_vel_x: 0.8             # Velocidad reducida
      
    local_costmap:
      width: 6                   # Área más pequeña
      height: 6
```

```yaml
# config/profiles/outdoor_mode.yaml
# Configuración para uso exterior

wheel_controller_node:
  ros__parameters:
    kinematics:
      max_linear_velocity: 3.0    # Velocidad aumentada
      max_angular_velocity: 1.5
      acceleration_limit: 2.0     # Más agresiva
      
safety_monitor_node:
  ros__parameters:
    safety_distances:
      emergency_stop: 0.5         # Distancias aumentadas
      warning: 1.5
      slow_down: 3.0
      
    adaptive_monitoring:
      environment_factor: 1.5     # Factor exterior

lidar_processor_node:
  ros__parameters:
    range:
      max_range: 30.0            # Rango completo
      
    filters:
      noise_threshold: 0.1       # Mayor tolerancia al ruido exterior
```

## Ejercicios Prácticos

### Ejercicio 1: Configuración Personalizada

Crear configuración para un escenario específico:

```yaml
# config/warehouse_mode.yaml
# TODO: Configurar parámetros para uso en almacén
# - Velocidades moderadas
# - Distancias de seguridad conservadoras
# - Mayor precisión en obstáculos
# - Configuración para espacios estrechos
```

### Ejercicio 2: Parámetros Dinámicos

Implementar nodo que ajuste parámetros según condiciones:

```python
class AdaptiveParameterManager(Node):
    def __init__(self):
        super().__init__('adaptive_parameter_manager')
        # TODO: Monitorear condiciones ambientales
        # TODO: Ajustar parámetros automáticamente
        # TODO: Cambiar entre perfiles de configuración
        # TODO: Log cambios de parámetros
```

### Ejercicio 3: Validación de Configuración

```bash
# 1. Validar archivo de configuración
ros2 param describe /wheel_controller_node hardware.wheel_base

# 2. Cargar configuración personalizada
ros2 launch tadeo_ecar_bringup ecar_system.launch.py config_file:=warehouse_mode.yaml

# 3. Cambiar parámetros en tiempo real
ros2 param set /wheel_controller_node kinematics.max_linear_velocity 1.5

# 4. Guardar configuración actual
ros2 param dump /wheel_controller_node --output-dir /tmp/ecar_config/

# 5. Verificar con monitor de parámetros
ros2 run tadeo_ecar_utils parameter_monitor.py
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Concepto de parámetros**: Sistema de configuración dinámico sin recompilación
2. **Tipos y validación**: Parámetros básicos con validación robusta
3. **Archivos YAML**: Configuración jerárquica y por entorno
4. **Parámetros dinámicos**: Callbacks para cambios en tiempo real
5. **Configuración del eCar**: Sistema completo de configuración modular

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes el concepto y beneficios de los parámetros
- [ ] Puedes crear configuraciones YAML jerárquicas
- [ ] Sabes implementar parámetros dinámicos con validación
- [ ] Comprendes los perfiles de configuración por entorno
- [ ] Puedes usar herramientas CLI para gestión de parámetros
- [ ] Has configurado el sistema eCar completamente

### Próximo Capítulo

En el Capítulo 8 estudiaremos:
- Sistema de Launch Files en ROS2
- Argumentos y condiciones de lanzamiento
- Composición y namespaces
- Launch files del sistema eCar completo
- Integración con parámetros y configuraciones

## Referencias

- [ROS2 Parameters Concepts](https://docs.ros.org/en/humble/Concepts/About-Parameters.html)
- [Using Parameters C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
- [Using Parameters Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [Parameter YAML Files](https://docs.ros.org/en/humble/How-To-Guides/Parameters-YAML-files-migration-guide.html)