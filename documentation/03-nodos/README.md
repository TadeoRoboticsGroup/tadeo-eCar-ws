# Capítulo 3: Nodos en ROS2

## Tabla de Contenidos

1. [Concepto de Nodo](#concepto-de-nodo)
2. [Anatomía de un Nodo](#anatomía-de-un-nodo)
3. [Nodos en C++](#nodos-en-c)
4. [Nodos en Python](#nodos-en-python)
5. [Lifecycle Nodes](#lifecycle-nodes)
6. [Comunicación entre Nodos](#comunicación-entre-nodos)
7. [Ejemplos del eCar](#ejemplos-del-ecar)

## Concepto de Nodo

### ¿Qué es un Nodo?

Un nodo en ROS2 es un proceso que ejecuta una tarea específica en el sistema robótico. En el contexto del eCar, cada nodo tiene una responsabilidad clara y bien definida.

```
Sistema eCar = Conjunto de Nodos Especializados

┌─ Nodo LiDAR ─────────┐    ┌─ Nodo Control ──────┐    ┌─ Nodo Navegación ───┐
│ • Lee sensor LiDAR   │    │ • Controla ruedas   │    │ • Planifica rutas   │
│ • Procesa datos      │ ── │ • Calcula cinemática│ ── │ • Evita obstáculos  │
│ • Publica /scan      │    │ • Publica /cmd_vel  │    │ • Sigue waypoints   │
└──────────────────────┘    └─────────────────────┘    └─────────────────────┘
```

### Características de los Nodos

**1. Independencia**: Cada nodo es un proceso separado
```bash
# Cada comando inicia un proceso independiente
ros2 run tadeo_ecar_perception lidar_processor_node    # Proceso A
ros2 run tadeo_ecar_control wheel_controller_node      # Proceso B
ros2 run tadeo_ecar_safety emergency_stop_node        # Proceso C
```

**2. Comunicación**: Los nodos se comunican mediante tópicos, servicios y acciones
```bash
# Flujo típico en el eCar
LiDAR Node → [/scan topic] → Navigation Node → [/cmd_vel topic] → Control Node
```

**3. Descubrimiento automático**: Los nodos se encuentran automáticamente
```bash
# No hay configuración manual de conexiones
# Los nodos se conectan según sus interfaces publicadas/suscritas
```

### Ventajas del Diseño Nodal

**Modularidad**: Desarrollo independiente
```cpp
// Equipo A desarrolla percepción
class LidarProcessor : public rclcpp::Node { /*...*/ };

// Equipo B desarrolla control (en paralelo)
class WheelController : public rclcpp::Node { /*...*/ };
```

**Reutilización**: Nodos genéricos para múltiples robots
```bash
# El mismo nodo de LiDAR funciona en diferentes robots
ros2 run tadeo_ecar_perception lidar_processor_node  # eCar
ros2 run other_robot_perception lidar_processor_node  # Otro robot
```

**Debugging**: Prueba de componentes aislados
```bash
# Probar solo el sistema de control sin sensores
ros2 run tadeo_ecar_control wheel_controller_node
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
```

## Anatomía de un Nodo

### Estructura Básica

Todo nodo ROS2 tiene:

1. **Constructor**: Inicialización del nodo
2. **Publishers**: Para enviar datos
3. **Subscribers**: Para recibir datos
4. **Timers**: Para ejecución periódica
5. **Services/Actions**: Para comunicación específica

### Ejemplo Conceptual

```cpp
class ECarNode : public rclcpp::Node
{
public:
    ECarNode() : Node("ecar_node")
    {
        // 1. Publishers (lo que envía)
        status_pub_ = this->create_publisher<std_msgs::msg::String>("status", 10);
        
        // 2. Subscribers (lo que recibe)
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ECarNode::cmdCallback, this, _1));
        
        // 3. Timers (tareas periódicas)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ECarNode::timerCallback, this));
    }
    
private:
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { /*...*/ }
    void timerCallback() { /*...*/ }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerInterface::SharedPtr timer_;
};
```

### Ciclo de Vida del Nodo

```
1. Construcción → 2. Inicialización → 3. Ejecución → 4. Finalización
      ↓                   ↓               ↓             ↓
   Constructor()      Setup topics     Callbacks     Destructor()
                      Services         Timers        Cleanup
                      Parameters       Spinning
```

## Nodos en C++

### Nodo Básico del eCar

Crear un nodo para monitorear la batería del eCar:

```cpp
// include/tadeo_ecar_safety/battery_monitor.hpp
#ifndef TADEO_ECAR_SAFETY__BATTERY_MONITOR_HPP_
#define TADEO_ECAR_SAFETY__BATTERY_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace tadeo_ecar_safety
{
class BatteryMonitor : public rclcpp::Node
{
public:
    BatteryMonitor();

private:
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void checkBatteryLevel();
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr low_battery_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    
    // Timer
    rclcpp::TimerInterface::SharedPtr check_timer_;
    
    // Parameters
    double low_battery_threshold_;
    double critical_battery_threshold_;
    
    // State
    double current_battery_level_;
    bool low_battery_warned_;
};
}  // namespace tadeo_ecar_safety

#endif  // TADEO_ECAR_SAFETY__BATTERY_MONITOR_HPP_
```

### Implementación del Nodo

```cpp
// src/battery_monitor.cpp
#include "tadeo_ecar_safety/battery_monitor.hpp"

namespace tadeo_ecar_safety
{
BatteryMonitor::BatteryMonitor() 
    : Node("battery_monitor_node"),
      current_battery_level_(100.0),
      low_battery_warned_(false)
{
    // Declarar parámetros
    this->declare_parameter("low_battery_threshold", 20.0);
    this->declare_parameter("critical_battery_threshold", 10.0);
    
    // Obtener parámetros
    low_battery_threshold_ = this->get_parameter("low_battery_threshold").as_double();
    critical_battery_threshold_ = this->get_parameter("critical_battery_threshold").as_double();
    
    // Publisher para alertas de batería baja
    low_battery_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "low_battery_alert", 10);
    
    // Subscriber para estado de batería
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "battery_state", 10,
        std::bind(&BatteryMonitor::batteryCallback, this, std::placeholders::_1));
    
    // Timer para verificación periódica
    check_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&BatteryMonitor::checkBatteryLevel, this));
    
    RCLCPP_INFO(this->get_logger(), "Battery Monitor Node initialized");
    RCLCPP_INFO(this->get_logger(), "Low battery threshold: %.1f%%", low_battery_threshold_);
    RCLCPP_INFO(this->get_logger(), "Critical battery threshold: %.1f%%", critical_battery_threshold_);
}

void BatteryMonitor::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    // Calcular porcentaje de batería
    if (msg->capacity > 0) {
        current_battery_level_ = (msg->charge / msg->capacity) * 100.0;
    } else {
        current_battery_level_ = msg->percentage;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Battery level: %.1f%%", current_battery_level_);
}

void BatteryMonitor::checkBatteryLevel()
{
    auto alert_msg = std_msgs::msg::Bool();
    
    if (current_battery_level_ <= critical_battery_threshold_) {
        RCLCPP_ERROR(this->get_logger(), 
                     "CRITICAL BATTERY LEVEL: %.1f%% - RETURN TO CHARGING STATION!", 
                     current_battery_level_);
        alert_msg.data = true;
        low_battery_pub_->publish(alert_msg);
        
    } else if (current_battery_level_ <= low_battery_threshold_) {
        if (!low_battery_warned_) {
            RCLCPP_WARN(this->get_logger(), 
                        "Low battery warning: %.1f%% - Consider returning to charge", 
                        current_battery_level_);
            low_battery_warned_ = true;
        }
        alert_msg.data = true;
        low_battery_pub_->publish(alert_msg);
        
    } else {
        // Reset warning flag when battery is good
        low_battery_warned_ = false;
        alert_msg.data = false;
        low_battery_pub_->publish(alert_msg);
    }
}
}  // namespace tadeo_ecar_safety

// Main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_safety::BatteryMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Configuración en CMakeLists.txt

```cmake
# Agregar al CMakeLists.txt del paquete tadeo_ecar_safety

# Find dependencies
find_package(sensor_msgs REQUIRED)

# Create executable
add_executable(battery_monitor_node src/battery_monitor.cpp)
ament_target_dependencies(battery_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install
install(TARGETS
  battery_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)
```

## Nodos en Python

### Equivalente en Python

```python
#!/usr/bin/env python3
# scripts/battery_monitor_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from rclpy.parameter import Parameter

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')
        
        # Declarar parámetros
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('critical_battery_threshold', 10.0)
        
        # Obtener parámetros
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self.critical_battery_threshold = self.get_parameter('critical_battery_threshold').value
        
        # Estado interno
        self.current_battery_level = 100.0
        self.low_battery_warned = False
        
        # Publisher
        self.low_battery_pub = self.create_publisher(Bool, 'low_battery_alert', 10)
        
        # Subscriber
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, 10)
        
        # Timer
        self.check_timer = self.create_timer(5.0, self.check_battery_level)
        
        self.get_logger().info('Battery Monitor Node initialized (Python)')
        self.get_logger().info(f'Low battery threshold: {self.low_battery_threshold}%')
        self.get_logger().info(f'Critical battery threshold: {self.critical_battery_threshold}%')
    
    def battery_callback(self, msg):
        """Callback para recibir estado de batería"""
        if msg.capacity > 0:
            self.current_battery_level = (msg.charge / msg.capacity) * 100.0
        else:
            self.current_battery_level = msg.percentage
        
        self.get_logger().debug(f'Battery level: {self.current_battery_level:.1f}%')
    
    def check_battery_level(self):
        """Verificar nivel de batería periódicamente"""
        alert_msg = Bool()
        
        if self.current_battery_level <= self.critical_battery_threshold:
            self.get_logger().error(
                f'CRITICAL BATTERY LEVEL: {self.current_battery_level:.1f}% - RETURN TO CHARGING STATION!')
            alert_msg.data = True
            self.low_battery_pub.publish(alert_msg)
            
        elif self.current_battery_level <= self.low_battery_threshold:
            if not self.low_battery_warned:
                self.get_logger().warn(
                    f'Low battery warning: {self.current_battery_level:.1f}% - Consider returning to charge')
                self.low_battery_warned = True
            alert_msg.data = True
            self.low_battery_pub.publish(alert_msg)
            
        else:
            # Reset warning cuando batería está bien
            self.low_battery_warned = False
            alert_msg.data = False
            self.low_battery_pub.publish(alert_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Configuración para Python

```python
# setup.py (agregar al final)
entry_points={
    'console_scripts': [
        'battery_monitor_node = tadeo_ecar_safety.battery_monitor_node:main',
    ],
},
```

## Lifecycle Nodes

### ¿Qué son los Lifecycle Nodes?

Los lifecycle nodes permiten control granular sobre el estado de los nodos. Útil para sistemas críticos como el eCar.

```
Estados del Lifecycle:
Unconfigured → Inactive → Active → Finalized
     ↑             ↑         ↑         ↓
     └─── Error ←──┴─────────┴─────────┘
```

### Implementación en el eCar

```cpp
// include/tadeo_ecar_control/wheel_controller_lifecycle.hpp
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

namespace tadeo_ecar_control
{
class WheelControllerLifecycle : public rclcpp_lifecycle::LifecycleNode
{
public:
    WheelControllerLifecycle();

protected:
    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state) override;
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state) override;
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state) override;
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
};
}  // namespace tadeo_ecar_control
```

### Implementación de Estados

```cpp
// src/wheel_controller_lifecycle.cpp

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WheelControllerLifecycle::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Configuring wheel controller...");
    
    // Configurar subscribers (inactivos)
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&WheelControllerLifecycle::cmdVelCallback, this, std::placeholders::_1));
    
    // Configurar publishers (inactivos)
    wheel_cmd_pub_ = this->create_lifecycle_publisher<std_msgs::msg::Float64MultiArray>(
        "wheel_commands", 10);
    
    // Verificar conexión con hardware
    if (!checkHardwareConnection()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to wheel hardware");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    
    RCLCPP_INFO(this->get_logger(), "Wheel controller configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WheelControllerLifecycle::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Activating wheel controller...");
    
    // Activar publishers
    wheel_cmd_pub_->on_activate();
    
    // Inicializar hardware
    if (!initializeHardware()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize wheel hardware");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    
    RCLCPP_INFO(this->get_logger(), "Wheel controller activated - Ready to receive commands");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WheelControllerLifecycle::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Deactivating wheel controller...");
    
    // Detener ruedas de forma segura
    stopWheelsSafely();
    
    // Desactivar publishers
    wheel_cmd_pub_->on_deactivate();
    
    RCLCPP_INFO(this->get_logger(), "Wheel controller deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
```

### Control Manual de Lifecycle

```bash
# Verificar estado del nodo
ros2 lifecycle get /wheel_controller_node
# State: unconfigured [1]

# Configurar nodo
ros2 lifecycle set /wheel_controller_node configure
# State: inactive [2]

# Activar nodo
ros2 lifecycle set /wheel_controller_node activate
# State: active [3]

# En emergencia, desactivar
ros2 lifecycle set /wheel_controller_node deactivate
# State: inactive [2]
```

## Comunicación entre Nodos

### Tópicos - Comunicación Asíncrona

Los tópicos son la forma principal de comunicación en el eCar:

```bash
# Flujo típico de datos en el eCar
Sensor LiDAR → /scan → Navigation Node → /cmd_vel → Wheel Controller
```

### Ejemplo: Conexión Sensor-Control

```cpp
// Nodo publicador (sensor)
class LidarNode : public rclcpp::Node
{
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    
    void publishScan() {
        auto scan_msg = sensor_msgs::msg::LaserScan();
        // ... llenar datos del LiDAR
        scan_pub_->publish(scan_msg);
    }
};

// Nodo suscriptor (navegación)
class NavigationNode : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Procesar datos LiDAR
        // Generar comando de velocidad
        publishCmdVel();
    }
};
```

### Servicios - Comunicación Síncrona

Para operaciones que requieren respuesta inmediata:

```cpp
// Servicio de calibración del eCar
class CalibrationService : public rclcpp::Node
{
    rclcpp::Service<tadeo_ecar_interfaces::srv::CalibrateWheels>::SharedPtr calibrate_service_;
    
    void calibrateCallback(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::CalibrateWheels::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::CalibrateWheels::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Starting wheel calibration...");
        
        // Realizar calibración
        bool success = performWheelCalibration();
        
        response->success = success;
        response->message = success ? "Calibration completed" : "Calibration failed";
    }
};
```

## Ejemplos del eCar

### Nodo de Monitoreo del Sistema

Un nodo que monitorea la salud general del eCar:

```cpp
// include/tadeo_ecar_safety/system_monitor.hpp
class SystemMonitor : public rclcpp::Node
{
public:
    SystemMonitor() : Node("system_monitor_node")
    {
        // Publishers para estado del sistema
        system_health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "system_health", 10);
        
        // Subscribers para monitorear componentes
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10,
            std::bind(&SystemMonitor::batteryCallback, this, std::placeholders::_1));
        
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&SystemMonitor::lidarCallback, this, std::placeholders::_1));
        
        // Timer para verificación periódica
        health_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SystemMonitor::publishSystemHealth, this));
    }

private:
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publishSystemHealth();
    
    // Component health tracking
    bool battery_healthy_;
    bool lidar_healthy_;
    rclcpp::Time last_battery_time_;
    rclcpp::Time last_lidar_time_;
    
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr system_health_pub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::TimerInterface::SharedPtr health_check_timer_;
};
```

### Nodo de Procesamiento de Cámara

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor_node')
        
        self.bridge = CvBridge()
        
        # Subscriber para imágenes de cámara
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # Publisher para obstáculos detectados
        self.obstacle_pub = self.create_publisher(Point, 'detected_obstacles', 10)
        
        # Publisher para imagen procesada
        self.processed_image_pub = self.create_publisher(Image, 'camera/processed_image', 10)
        
        self.get_logger().info('Camera Processor Node initialized')
    
    def image_callback(self, msg):
        try:
            # Convertir mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Procesar imagen (detección de obstáculos)
            obstacles = self.detect_obstacles(cv_image)
            
            # Publicar obstáculos detectados
            for obstacle in obstacles:
                obstacle_msg = Point()
                obstacle_msg.x = float(obstacle[0])
                obstacle_msg.y = float(obstacle[1])
                obstacle_msg.z = 0.0
                self.obstacle_pub.publish(obstacle_msg)
            
            # Crear imagen procesada
            processed_image = self.draw_obstacles(cv_image, obstacles)
            
            # Convertir de vuelta a mensaje ROS y publicar
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            self.processed_image_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def detect_obstacles(self, image):
        # Implementación simple de detección de obstáculos
        # En un sistema real usarías algoritmos más sofisticados
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        obstacles = []
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filtrar contornos pequeños
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    obstacles.append((cx, cy))
        
        return obstacles
    
    def draw_obstacles(self, image, obstacles):
        processed = image.copy()
        for obstacle in obstacles:
            cv2.circle(processed, obstacle, 10, (0, 0, 255), -1)
            cv2.putText(processed, 'Obstacle', (obstacle[0]-20, obstacle[1]-15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return processed
```

### Launch File para Múltiples Nodos

```python
# launch/ecar_sensors.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de monitor de batería
        Node(
            package='tadeo_ecar_safety',
            executable='battery_monitor_node',
            name='battery_monitor',
            parameters=[{
                'low_battery_threshold': 25.0,
                'critical_battery_threshold': 15.0
            }]
        ),
        
        # Nodo de procesamiento de cámara
        Node(
            package='tadeo_ecar_perception',
            executable='camera_processor_node',
            name='camera_processor',
            remappings=[
                ('camera/image_raw', '/ecar/camera/image_raw'),
                ('detected_obstacles', '/ecar/perception/obstacles')
            ]
        ),
        
        # Nodo de monitor del sistema
        Node(
            package='tadeo_ecar_safety',
            executable='system_monitor_node',
            name='system_monitor'
        )
    ])
```

## Ejercicios Prácticos

### Ejercicio 1: Crear un Nodo Simple

Crear un nodo que publique la velocidad actual del eCar:

```cpp
// Crear: src/speed_publisher_node.cpp
class SpeedPublisher : public rclcpp::Node
{
public:
    SpeedPublisher() : Node("speed_publisher_node")
    {
        // TODO: Implementar publisher de velocidad
        // TODO: Suscribirse a odometría
        // TODO: Calcular y publicar velocidad actual
    }
};
```

### Ejercicio 2: Nodo de Control de LED

Crear un nodo Python que controle LEDs basado en el estado del robot:

```python
class LEDController(Node):
    def __init__(self):
        super().__init__('led_controller_node')
        # TODO: Suscribirse a estado del robot
        # TODO: Controlar LEDs según estado
        # TODO: Verde = OK, Amarillo = Warning, Rojo = Error
```

### Ejercicio 3: Integración con el Sistema

Integrar tus nodos con el sistema eCar completo:

```bash
# 1. Compilar tus nodos
colcon build --packages-select tadeo_ecar_safety

# 2. Ejecutar sistema básico
ros2 launch tadeo_ecar_bringup minimal_system.launch.py

# 3. En otra terminal, ejecutar tu nodo
ros2 run tadeo_ecar_safety speed_publisher_node

# 4. Verificar comunicación
ros2 topic echo /current_speed
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Concepto de nodo**: Procesos independientes con responsabilidades específicas
2. **Anatomía del nodo**: Publishers, subscribers, timers, services
3. **Implementación en C++**: Nodos robustos para control en tiempo real
4. **Implementación en Python**: Nodos flexibles para procesamiento
5. **Lifecycle nodes**: Control granular del estado de nodos críticos
6. **Comunicación**: Tópicos, servicios y patrones de integración

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes qué es un nodo y su propósito
- [ ] Puedes crear nodos básicos en C++ y Python
- [ ] Comprendes el ciclo de vida de un nodo
- [ ] Sabes cuándo usar lifecycle nodes
- [ ] Puedes integrar nodos en el sistema eCar

### Próximo Capítulo

En el Capítulo 4 estudiaremos en profundidad:
- Comunicación mediante tópicos
- Publishers y subscribers
- Tipos de mensajes
- Quality of Service (QoS)

## Referencias

- [ROS2 Node Concepts](https://docs.ros.org/en/humble/Concepts/About-Nodes.html)
- [Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)
- [rclcpp API](https://docs.ros2.org/humble/api/rclcpp/)
- [rclpy API](https://docs.ros2.org/humble/api/rclpy/)