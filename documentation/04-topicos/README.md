# Capítulo 4: Tópicos en ROS2

## Tabla de Contenidos

1. [Concepto de Tópicos](#concepto-de-tópicos)
2. [Publisher y Subscriber](#publisher-y-subscriber)
3. [Tipos de Mensajes](#tipos-de-mensajes)
4. [Quality of Service (QoS)](#quality-of-service-qos)
5. [Implementación en C++](#implementación-en-c++)
6. [Implementación en Python](#implementación-en-python)
7. [Tópicos del eCar](#tópicos-del-ecar)
8. [Herramientas de Debugging](#herramientas-de-debugging)

## Concepto de Tópicos

### ¿Qué son los Tópicos?

Los tópicos son canales de comunicación nombrados donde los nodos pueden enviar y recibir mensajes. Es el mecanismo principal de comunicación asíncrona en ROS2.

```
    Publisher              Tópico                Subscriber
┌─────────────────┐    /cmd_vel topic     ┌─────────────────┐
│ Navigation Node │ ─────────────────────→ │ Control Node    │
│ (Publishes)     │                       │ (Subscribes)    │
└─────────────────┘                       └─────────────────┘
```

### Características de los Tópicos

**1. Comunicación Asíncrona**
```cpp
// Publisher no espera confirmación
publisher->publish(msg);
// Continúa inmediatamente

// Subscriber procesa cuando llega el mensaje
void callback(const Msg::SharedPtr msg) {
    // Procesar mensaje cuando llegue
}
```

**2. Muchos a Muchos**
```bash
# Un tópico puede tener múltiples publishers y subscribers
ros2 topic info /scan
# Publishers: lidar_node, simulation_node
# Subscribers: navigation_node, visualization_node, safety_node
```

**3. Tipado Fuerte**
```cpp
// Solo mensajes del tipo correcto
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
// No puede publicar sensor_msgs::msg::LaserScan aquí
```

### Flujo de Datos en el eCar

```
Sensores → Percepción → Planificación → Control → Actuadores

LiDAR ──→ /scan ──→ Navigation ──→ /cmd_vel ──→ Wheels
Camera ─→ /image ──→    Node    ──→ /status ──→ Monitor
IMU ────→ /imu ────→             ──→ /odom ────→ Localization
```

## Publisher y Subscriber

### Patrón Publisher-Subscriber

El patrón pub-sub desacopla el envío y recepción de datos:

```cpp
// Publisher - No sabe quién recibe
class SensorNode : public rclcpp::Node {
    void publishData() {
        auto msg = sensor_msgs::msg::LaserScan();
        // ... llenar datos
        publisher_->publish(msg);
        // No sabe si alguien está escuchando
    }
};

// Subscriber - No sabe quién envía
class ProcessorNode : public rclcpp::Node {
    void dataCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Procesar datos
        // No sabe de dónde vienen
    }
};
```

### Ventajas del Desacoplamiento

**1. Flexibilidad**
```bash
# Agregar nuevo subscriber sin modificar publisher
ros2 run tadeo_ecar_perception lidar_processor_node    # Publisher
ros2 run tadeo_ecar_navigation navigation_node         # Subscriber 1
ros2 run tadeo_ecar_safety collision_detector_node     # Subscriber 2 (nuevo)
```

**2. Escalabilidad**
```bash
# Un sensor puede alimentar múltiples sistemas
/scan topic:
├── Navigation system (path planning)
├── Safety system (collision detection)  
├── Mapping system (SLAM)
└── Visualization (RViz)
```

**3. Modularity**
```bash
# Cambiar implementación sin afectar otros nodos
# Cambiar LiDAR por cámara sin modificar navigation_node
```

### Ejemplo: Sensor LiDAR del eCar

```cpp
// Publisher: Nodo del sensor LiDAR
class LidarNode : public rclcpp::Node
{
public:
    LidarNode() : Node("lidar_node")
    {
        // Crear publisher
        scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan", 10);
        
        // Timer para publicar datos periódicamente
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz
            std::bind(&LidarNode::publishScan, this));
    }

private:
    void publishScan()
    {
        auto scan_msg = sensor_msgs::msg::LaserScan();
        
        // Configurar header
        scan_msg.header.stamp = this->get_clock()->now();
        scan_msg.header.frame_id = "laser_frame";
        
        // Configurar parámetros del LiDAR
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = M_PI / 180.0;  // 1 grado
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 30.0;
        
        // Simular datos (en implementación real leer del hardware)
        int num_readings = (scan_msg.angle_max - scan_msg.angle_min) / 
                          scan_msg.angle_increment;
        scan_msg.ranges.resize(num_readings);
        
        for(int i = 0; i < num_readings; ++i) {
            // Simular obstáculos aleatorios
            scan_msg.ranges[i] = 5.0 + sin(i * 0.1) * 2.0;
        }
        
        // Publicar
        scan_publisher_->publish(scan_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published LiDAR scan with %zu points", 
                     scan_msg.ranges.size());
    }
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    rclcpp::TimerInterface::SharedPtr timer_;
};
```

```cpp
// Subscriber: Nodo de navegación
class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_node")
    {
        // Crear subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&NavigationNode::scanCallback, this, std::placeholders::_1));
        
        // Publisher para comandos de velocidad
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received scan with %zu points", 
                     msg->ranges.size());
        
        // Procesar datos LiDAR
        processLidarData(msg);
        
        // Generar comando de velocidad
        auto cmd_msg = geometry_msgs::msg::Twist();
        
        // Algoritmo simple: avanzar si no hay obstáculos adelante
        float front_distance = getFrontDistance(msg);
        
        if (front_distance > 2.0) {
            cmd_msg.linear.x = 0.5;  // Avanzar
        } else if (front_distance > 1.0) {
            cmd_msg.linear.x = 0.2;  // Avanzar lento
            cmd_msg.angular.z = 0.3; // Girar un poco
        } else {
            cmd_msg.linear.x = 0.0;  // Parar
            cmd_msg.angular.z = 0.5; // Girar para buscar camino libre
        }
        
        cmd_vel_publisher_->publish(cmd_msg);
    }
    
    void processLidarData(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Procesar datos para detección de obstáculos
        // Filtrar ruido, agrupar puntos, detectar formas, etc.
    }
    
    float getFrontDistance(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Obtener distancia al frente (índice central del array)
        size_t front_index = scan->ranges.size() / 2;
        return scan->ranges[front_index];
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};
```

## Tipos de Mensajes

### Mensajes Estándar

ROS2 incluye tipos de mensajes comunes:

**1. Mensajes Básicos (std_msgs)**
```cpp
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

// Ejemplo de uso
auto msg = std_msgs::msg::String();
msg.data = "Estado del eCar: Navegando";
status_pub_->publish(msg);
```

**2. Mensajes de Geometría (geometry_msgs)**
```cpp
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

// Comando de velocidad para el eCar
auto cmd = geometry_msgs::msg::Twist();
cmd.linear.x = 1.0;   // m/s adelante
cmd.linear.y = 0.5;   // m/s lateral (4WD4WS permite movimiento lateral)
cmd.angular.z = 0.2;  // rad/s rotación
```

**3. Mensajes de Sensores (sensor_msgs)**
```cpp
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

// Estado de batería del eCar
auto battery = sensor_msgs::msg::BatteryState();
battery.voltage = 12.6;  // Voltios
battery.percentage = 85.0; // Porcentaje
battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
```

### Mensajes Personalizados para el eCar

Para crear mensajes específicos del eCar:

```bash
# En el paquete tadeo_ecar_msgs
mkdir -p msg
```

**Mensaje de Estado del eCar**
```
# msg/ECarStatus.msg
std_msgs/Header header

# Estado general
string robot_mode           # "manual", "autonomous", "emergency"
bool system_ready
float32 battery_voltage
float32 battery_percentage

# Estado de movimiento
geometry_msgs/Twist current_velocity
bool wheels_locked
bool steering_active

# Estado de sensores
bool lidar_active
bool camera_active
bool imu_active
bool gps_active

# Diagnósticos
string[] active_warnings
string[] active_errors
float32 cpu_usage
float32 memory_usage
```

**Configuración en CMakeLists.txt**
```cmake
# Agregar al CMakeLists.txt de tadeo_ecar_msgs
find_package(rosidl_default_generators REQUIRED)

# Declarar mensajes
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ECarStatus.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)
```

**Uso del mensaje personalizado**
```cpp
#include "tadeo_ecar_msgs/msg/e_car_status.hpp"

class SystemMonitor : public rclcpp::Node
{
public:
    SystemMonitor() : Node("system_monitor_node")
    {
        status_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::ECarStatus>(
            "ecar_status", 10);
    }

private:
    void publishStatus()
    {
        auto status = tadeo_ecar_msgs::msg::ECarStatus();
        
        // Llenar header
        status.header.stamp = this->get_clock()->now();
        status.header.frame_id = "base_link";
        
        // Estado del robot
        status.robot_mode = "autonomous";
        status.system_ready = true;
        status.battery_voltage = getCurrentBatteryVoltage();
        status.battery_percentage = getCurrentBatteryPercentage();
        
        // Estado de sensores
        status.lidar_active = isLidarActive();
        status.camera_active = isCameraActive();
        
        // Publicar
        status_pub_->publish(status);
    }
    
    rclcpp::Publisher<tadeo_ecar_msgs::msg::ECarStatus>::SharedPtr status_pub_;
};
```

## Quality of Service (QoS)

### ¿Qué es QoS?

Quality of Service define cómo se comporta la comunicación entre publishers y subscribers. Es crucial para sistemas robóticos críticos como el eCar.

### Perfiles de QoS

**1. Reliability (Confiabilidad)**
```cpp
// RELIABLE: Garantiza entrega (TCP-like)
auto qos_reliable = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

// BEST_EFFORT: Mejor esfuerzo, pero más rápido (UDP-like)
auto qos_best_effort = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
```

**2. Durability (Durabilidad)**
```cpp
// VOLATILE: Solo datos nuevos
auto qos_volatile = rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

// TRANSIENT_LOCAL: Mantiene último mensaje para nuevos subscribers
auto qos_transient = rclcpp::QoS(10).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
```

**3. History (Historial)**
```cpp
// KEEP_LAST: Mantener últimos N mensajes
auto qos_keep_last = rclcpp::QoS(10).history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

// KEEP_ALL: Mantener todos los mensajes
auto qos_keep_all = rclcpp::QoS(rclcpp::KeepAll()).history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
```

### QoS para el eCar

**Configuración según tipo de datos:**

```cpp
class ECarNode : public rclcpp::Node
{
public:
    ECarNode() : Node("ecar_node")
    {
        // QoS para datos de sensores (alta frecuencia, tolerante a pérdidas)
        auto sensor_qos = rclcpp::QoS(10)
            .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", sensor_qos, std::bind(&ECarNode::scanCallback, this, _1));
        
        // QoS para comandos de control (críticos, deben llegar)
        auto control_qos = rclcpp::QoS(1)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", control_qos);
        
        // QoS para estado del sistema (persistente para nuevos nodos)
        auto status_qos = rclcpp::QoS(1)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        
        status_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::ECarStatus>(
            "ecar_status", status_qos);
    }
};
```

### QoS Preconfigurado

```cpp
// Usar perfiles predefinidos
#include <rclcpp/qos.hpp>

// Para sensores
auto sensor_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());

// Para parámetros del sistema
auto param_pub = this->create_publisher<std_msgs::msg::String>(
    "robot_description", rclcpp::ParametersQoS());

// Para servicios
auto service_pub = this->create_publisher<std_msgs::msg::String>(
    "service_status", rclcpp::ServicesQoS());
```

## Implementación en C++

### Publisher Completo

```cpp
// include/tadeo_ecar_perception/camera_publisher.hpp
#ifndef TADEO_ECAR_PERCEPTION__CAMERA_PUBLISHER_HPP_
#define TADEO_ECAR_PERCEPTION__CAMERA_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace tadeo_ecar_perception
{
class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher();
    ~CameraPublisher();

private:
    void captureAndPublish();
    void publishCameraInfo();
    
    // OpenCV camera
    cv::VideoCapture cap_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    
    // Timer
    rclcpp::TimerInterface::SharedPtr timer_;
    
    // Parameters
    int camera_index_;
    int image_width_;
    int image_height_;
    double fps_;
    std::string frame_id_;
};
}  // namespace tadeo_ecar_perception

#endif  // TADEO_ECAR_PERCEPTION__CAMERA_PUBLISHER_HPP_
```

```cpp
// src/camera_publisher.cpp
#include "tadeo_ecar_perception/camera_publisher.hpp"

namespace tadeo_ecar_perception
{
CameraPublisher::CameraPublisher() : Node("camera_publisher_node")
{
    // Declarar parámetros
    this->declare_parameter("camera_index", 0);
    this->declare_parameter("image_width", 640);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("fps", 30.0);
    this->declare_parameter("frame_id", "camera_frame");
    
    // Obtener parámetros
    camera_index_ = this->get_parameter("camera_index").as_int();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    fps_ = this->get_parameter("fps").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    
    // Configurar cámara
    cap_.open(camera_index_);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open camera %d", camera_index_);
        return;
    }
    
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    
    // Crear publishers con QoS de sensor
    auto qos = rclcpp::SensorDataQoS();
    
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/image_raw", qos);
    
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera/camera_info", qos);
    
    // Timer para captura
    auto timer_period = std::chrono::duration<double>(1.0 / fps_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
        std::bind(&CameraPublisher::captureAndPublish, this));
    
    RCLCPP_INFO(this->get_logger(), "Camera publisher initialized");
    RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d @ %.1f FPS", 
                image_width_, image_height_, fps_);
}

CameraPublisher::~CameraPublisher()
{
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void CameraPublisher::captureAndPublish()
{
    cv::Mat frame;
    if (!cap_.read(frame)) {
        RCLCPP_WARN(this->get_logger(), "Failed to capture frame");
        return;
    }
    
    // Convertir OpenCV Mat a ROS message
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = frame_id_;
    
    // Publicar imagen
    image_pub_->publish(*msg);
    
    // Publicar info de cámara
    publishCameraInfo();
    
    RCLCPP_DEBUG(this->get_logger(), "Published image %dx%d", 
                 msg->width, msg->height);
}

void CameraPublisher::publishCameraInfo()
{
    auto camera_info = sensor_msgs::msg::CameraInfo();
    
    camera_info.header.stamp = this->get_clock()->now();
    camera_info.header.frame_id = frame_id_;
    
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    
    // Matriz de calibración de cámara (ejemplo)
    camera_info.k = {
        500.0, 0.0, 320.0,
        0.0, 500.0, 240.0,
        0.0, 0.0, 1.0
    };
    
    // Modelo de distorsión
    camera_info.distortion_model = "plumb_bob";
    camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    camera_info_pub_->publish(camera_info);
}
}  // namespace tadeo_ecar_perception

// Main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_perception::CameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Subscriber con Procesamiento

```cpp
// include/tadeo_ecar_perception/obstacle_detector.hpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace tadeo_ecar_perception
{
class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    std::vector<geometry_msgs::msg::Point> detectObstacles(
        const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void publishVisualization(const std::vector<geometry_msgs::msg::Point>& obstacles);
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Parameters
    double min_obstacle_distance_;
    double max_obstacle_distance_;
    double obstacle_clustering_distance_;
    int min_points_per_obstacle_;
};
}  // namespace tadeo_ecar_perception
```

```cpp
// src/obstacle_detector.cpp
#include "tadeo_ecar_perception/obstacle_detector.hpp"
#include <cmath>
#include <algorithm>

namespace tadeo_ecar_perception
{
ObstacleDetector::ObstacleDetector() : Node("obstacle_detector_node")
{
    // Parámetros
    this->declare_parameter("min_obstacle_distance", 0.1);
    this->declare_parameter("max_obstacle_distance", 10.0);
    this->declare_parameter("obstacle_clustering_distance", 0.3);
    this->declare_parameter("min_points_per_obstacle", 3);
    
    min_obstacle_distance_ = this->get_parameter("min_obstacle_distance").as_double();
    max_obstacle_distance_ = this->get_parameter("max_obstacle_distance").as_double();
    obstacle_clustering_distance_ = this->get_parameter("obstacle_clustering_distance").as_double();
    min_points_per_obstacle_ = this->get_parameter("min_points_per_obstacle").as_int();
    
    // Subscriber con QoS de sensor
    auto qos = rclcpp::SensorDataQoS();
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos,
        std::bind(&ObstacleDetector::scanCallback, this, std::placeholders::_1));
    
    // Publishers
    obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "detected_obstacles", 10);
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "obstacle_markers", 10);
    
    RCLCPP_INFO(this->get_logger(), "Obstacle detector initialized");
}

void ObstacleDetector::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Detectar obstáculos
    auto obstacles = detectObstacles(msg);
    
    // Publicar cada obstáculo
    for (size_t i = 0; i < obstacles.size(); ++i) {
        auto obstacle_msg = geometry_msgs::msg::PointStamped();
        obstacle_msg.header = msg->header;
        obstacle_msg.point = obstacles[i];
        
        obstacle_pub_->publish(obstacle_msg);
    }
    
    // Visualización
    publishVisualization(obstacles);
    
    RCLCPP_DEBUG(this->get_logger(), "Detected %zu obstacles", obstacles.size());
}

std::vector<geometry_msgs::msg::Point> ObstacleDetector::detectObstacles(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    std::vector<geometry_msgs::msg::Point> obstacles;
    std::vector<geometry_msgs::msg::Point> current_cluster;
    
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float range = scan->ranges[i];
        
        // Filtrar puntos válidos
        if (range < min_obstacle_distance_ || range > max_obstacle_distance_ ||
            std::isnan(range) || std::isinf(range)) {
            // Procesar cluster actual si existe
            if (current_cluster.size() >= static_cast<size_t>(min_points_per_obstacle_)) {
                // Calcular centroide del cluster
                geometry_msgs::msg::Point centroid;
                for (const auto& point : current_cluster) {
                    centroid.x += point.x;
                    centroid.y += point.y;
                }
                centroid.x /= current_cluster.size();
                centroid.y /= current_cluster.size();
                centroid.z = 0.0;
                
                obstacles.push_back(centroid);
            }
            current_cluster.clear();
            continue;
        }
        
        // Convertir coordenadas polares a cartesianas
        float angle = scan->angle_min + i * scan->angle_increment;
        geometry_msgs::msg::Point point;
        point.x = range * cos(angle);
        point.y = range * sin(angle);
        point.z = 0.0;
        
        // Agrupar puntos cercanos
        if (current_cluster.empty()) {
            current_cluster.push_back(point);
        } else {
            // Calcular distancia al último punto del cluster
            auto& last_point = current_cluster.back();
            double distance = sqrt(pow(point.x - last_point.x, 2) + 
                                 pow(point.y - last_point.y, 2));
            
            if (distance <= obstacle_clustering_distance_) {
                current_cluster.push_back(point);
            } else {
                // Finalizar cluster actual
                if (current_cluster.size() >= static_cast<size_t>(min_points_per_obstacle_)) {
                    geometry_msgs::msg::Point centroid;
                    for (const auto& cluster_point : current_cluster) {
                        centroid.x += cluster_point.x;
                        centroid.y += cluster_point.y;
                    }
                    centroid.x /= current_cluster.size();
                    centroid.y /= current_cluster.size();
                    centroid.z = 0.0;
                    
                    obstacles.push_back(centroid);
                }
                
                // Iniciar nuevo cluster
                current_cluster.clear();
                current_cluster.push_back(point);
            }
        }
    }
    
    // Procesar último cluster
    if (current_cluster.size() >= static_cast<size_t>(min_points_per_obstacle_)) {
        geometry_msgs::msg::Point centroid;
        for (const auto& point : current_cluster) {
            centroid.x += point.x;
            centroid.y += point.y;
        }
        centroid.x /= current_cluster.size();
        centroid.y /= current_cluster.size();
        centroid.z = 0.0;
        
        obstacles.push_back(centroid);
    }
    
    return obstacles;
}

void ObstacleDetector::publishVisualization(
    const std::vector<geometry_msgs::msg::Point>& obstacles)
{
    auto marker_array = visualization_msgs::msg::MarkerArray();
    
    for (size_t i = 0; i < obstacles.size(); ++i) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "laser_frame";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "obstacles";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position = obstacles[i];
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker_array.markers.push_back(marker);
    }
    
    marker_pub_->publish(marker_array);
}
}  // namespace tadeo_ecar_perception

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_perception::ObstacleDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Implementación en Python

### Publisher en Python

```python
#!/usr/bin/env python3
# scripts/battery_publisher_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header
import psutil
import time

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher_node')
        
        # Declarar parámetros
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('battery_capacity', 10000.0)  # mAh
        self.declare_parameter('initial_charge', 8500.0)  # mAh
        self.declare_parameter('discharge_rate', 50.0)  # mA
        
        # Obtener parámetros
        self.publish_rate = self.get_parameter('publish_rate').value
        self.battery_capacity = self.get_parameter('battery_capacity').value
        self.initial_charge = self.get_parameter('initial_charge').value
        self.discharge_rate = self.get_parameter('discharge_rate').value
        
        # Estado interno
        self.current_charge = self.initial_charge
        self.start_time = time.time()
        
        # QoS para datos de batería (críticos pero no de alta frecuencia)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', qos)
        
        # Timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_battery_state)
        
        self.get_logger().info('Battery publisher initialized')
        self.get_logger().info(f'Capacity: {self.battery_capacity:.0f} mAh')
        self.get_logger().info(f'Initial charge: {self.initial_charge:.0f} mAh')
    
    def publish_battery_state(self):
        """Publicar estado actual de la batería"""
        # Simular descarga de batería
        elapsed_time = time.time() - self.start_time
        consumed_charge = (self.discharge_rate / 1000.0) * (elapsed_time / 3600.0) * 1000.0  # mAh
        self.current_charge = max(0.0, self.initial_charge - consumed_charge)
        
        # Crear mensaje
        battery_msg = BatteryState()
        
        # Header
        battery_msg.header = Header()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.header.frame_id = 'base_link'
        
        # Datos de batería
        battery_msg.voltage = self.calculate_voltage()
        battery_msg.current = -self.discharge_rate / 1000.0  # A (negativo = descarga)
        battery_msg.charge = self.current_charge / 1000.0  # Ah
        battery_msg.capacity = self.battery_capacity / 1000.0  # Ah
        battery_msg.design_capacity = self.battery_capacity / 1000.0  # Ah
        battery_msg.percentage = (self.current_charge / self.battery_capacity) * 100.0
        
        # Estado de alimentación
        if battery_msg.percentage > 20.0:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif battery_msg.percentage > 5.0:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        
        # Salud de la batería
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        
        # Publicar
        self.battery_pub.publish(battery_msg)
        
        # Log periódico
        if int(elapsed_time) % 10 == 0:  # Cada 10 segundos
            self.get_logger().info(
                f'Battery: {battery_msg.percentage:.1f}% '
                f'({battery_msg.voltage:.2f}V, {battery_msg.current:.3f}A)'
            )
    
    def calculate_voltage(self):
        """Calcular voltaje basado en carga restante"""
        # Curva de descarga simplificada para batería Li-ion
        percentage = (self.current_charge / self.battery_capacity) * 100.0
        
        if percentage > 90:
            voltage = 12.6  # Vollá completa
        elif percentage > 70:
            voltage = 12.4
        elif percentage > 50:
            voltage = 12.2
        elif percentage > 30:
            voltage = 12.0
        elif percentage > 10:
            voltage = 11.8
        else:
            voltage = 11.5  # Batería baja
        
        return voltage

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    
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

### Subscriber en Python

```python
#!/usr/bin/env python3
# scripts/system_monitor_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import psutil
import json

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor_node')
        
        # Estado del sistema
        self.battery_status = None
        self.last_cmd_vel_time = None
        self.last_scan_time = None
        self.robot_moving = False
        self.system_warnings = []
        self.system_errors = []
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self.battery_callback, control_qos)
        
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, sensor_qos)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, control_qos)
        
        # Publishers
        self.system_status_pub = self.create_publisher(
            String, 'system_status', control_qos)
        
        self.emergency_pub = self.create_publisher(
            Bool, 'emergency_stop', control_qos)
        
        # Timer para monitoreo periódico
        self.monitor_timer = self.create_timer(2.0, self.monitor_system)
        
        self.get_logger().info('System monitor initialized')
    
    def battery_callback(self, msg):
        """Callback para estado de batería"""
        self.battery_status = msg
        
        # Verificar estado crítico de batería
        if msg.percentage < 10.0:
            self.add_error(f"Critical battery level: {msg.percentage:.1f}%")
            # Enviar señal de emergencia
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_pub.publish(emergency_msg)
            
        elif msg.percentage < 20.0:
            self.add_warning(f"Low battery level: {msg.percentage:.1f}%")
        else:
            self.remove_warning_type("Low battery")
            self.remove_error_type("Critical battery")
    
    def scan_callback(self, msg):
        """Callback para datos LiDAR"""
        self.last_scan_time = self.get_clock().now()
        
        # Verificar si hay obstáculos muy cercanos
        min_distance = min([r for r in msg.ranges if not (r < msg.range_min or r > msg.range_max)])
        
        if min_distance < 0.2:  # Obstáculo a menos de 20cm
            self.add_warning(f"Close obstacle detected: {min_distance:.2f}m")
        else:
            self.remove_warning_type("Close obstacle")
    
    def cmd_vel_callback(self, msg):
        """Callback para comandos de velocidad"""
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Verificar si el robot se está moviendo
        linear_speed = (msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2)**0.5
        angular_speed = abs(msg.angular.z)
        
        self.robot_moving = linear_speed > 0.01 or angular_speed > 0.01
    
    def monitor_system(self):
        """Monitoreo periódico del sistema"""
        current_time = self.get_clock().now()
        
        # Verificar timeouts de sensores
        if self.last_scan_time is not None:
            scan_timeout = (current_time - self.last_scan_time).nanoseconds / 1e9
            if scan_timeout > 2.0:  # 2 segundos sin datos LiDAR
                self.add_error("LiDAR timeout - no scan data received")
            else:
                self.remove_error_type("LiDAR timeout")
        
        # Verificar timeout de comandos (solo si el robot debería moverse)
        if self.last_cmd_vel_time is not None and self.robot_moving:
            cmd_timeout = (current_time - self.last_cmd_vel_time).nanoseconds / 1e9
            if cmd_timeout > 1.0:  # 1 segundo sin comandos
                self.add_warning("Command timeout - no velocity commands received")
            else:
                self.remove_warning_type("Command timeout")
        
        # Monitorear recursos del sistema
        self.monitor_system_resources()
        
        # Publicar estado del sistema
        self.publish_system_status()
    
    def monitor_system_resources(self):
        """Monitorear CPU, memoria y disco"""
        # CPU usage
        cpu_percent = psutil.cpu_percent(interval=None)
        if cpu_percent > 80.0:
            self.add_warning(f"High CPU usage: {cpu_percent:.1f}%")
        else:
            self.remove_warning_type("High CPU usage")
        
        # Memory usage
        memory = psutil.virtual_memory()
        if memory.percent > 85.0:
            self.add_warning(f"High memory usage: {memory.percent:.1f}%")
        else:
            self.remove_warning_type("High memory usage")
        
        # Disk usage
        disk = psutil.disk_usage('/')
        disk_percent = (disk.used / disk.total) * 100
        if disk_percent > 90.0:
            self.add_warning(f"High disk usage: {disk_percent:.1f}%")
        else:
            self.remove_warning_type("High disk usage")
    
    def publish_system_status(self):
        """Publicar estado del sistema"""
        status = {
            'timestamp': self.get_clock().now().to_msg(),
            'robot_moving': self.robot_moving,
            'warnings': self.system_warnings,
            'errors': self.system_errors,
            'warning_count': len(self.system_warnings),
            'error_count': len(self.system_errors)
        }
        
        # Agregar información de batería si está disponible
        if self.battery_status is not None:
            status['battery'] = {
                'percentage': self.battery_status.percentage,
                'voltage': self.battery_status.voltage,
                'current': self.battery_status.current
            }
        
        # Agregar recursos del sistema
        status['system_resources'] = {
            'cpu_percent': psutil.cpu_percent(interval=None),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_percent': (psutil.disk_usage('/').used / psutil.disk_usage('/').total) * 100
        }
        
        # Publicar como JSON string
        status_msg = String()
        status_msg.data = json.dumps(status, default=str)
        self.system_status_pub.publish(status_msg)
        
        # Log periódico
        if status['error_count'] > 0:
            self.get_logger().error(f"System has {status['error_count']} errors: {status['errors']}")
        elif status['warning_count'] > 0:
            self.get_logger().warn(f"System has {status['warning_count']} warnings: {status['warnings']}")
        else:
            self.get_logger().debug("System status: OK")
    
    def add_warning(self, warning):
        """Agregar warning si no existe"""
        if warning not in self.system_warnings:
            self.system_warnings.append(warning)
            self.get_logger().warn(f"System warning: {warning}")
    
    def add_error(self, error):
        """Agregar error si no existe"""
        if error not in self.system_errors:
            self.system_errors.append(error)
            self.get_logger().error(f"System error: {error}")
    
    def remove_warning_type(self, warning_type):
        """Remover warnings que contienen el tipo especificado"""
        self.system_warnings = [w for w in self.system_warnings if warning_type not in w]
    
    def remove_error_type(self, error_type):
        """Remover errors que contienen el tipo especificado"""
        self.system_errors = [e for e in self.system_errors if error_type not in e]

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    
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

## Tópicos del eCar

### Mapa de Tópicos del Sistema eCar

```bash
# Sensores
/scan                    # sensor_msgs/LaserScan - Datos LiDAR
/camera/image_raw        # sensor_msgs/Image - Imagen de cámara
/camera/camera_info      # sensor_msgs/CameraInfo - Calibración cámara
/imu/data               # sensor_msgs/Imu - Datos IMU
/gps/fix                # sensor_msgs/NavSatFix - GPS
/battery_state          # sensor_msgs/BatteryState - Estado batería

# Control
/cmd_vel                # geometry_msgs/Twist - Comandos de velocidad
/wheel_states           # sensor_msgs/JointState - Estado de ruedas
/steering_commands      # std_msgs/Float64MultiArray - Comandos dirección

# Navegación
/odom                   # nav_msgs/Odometry - Odometría
/global_path            # nav_msgs/Path - Ruta global
/local_path             # nav_msgs/Path - Ruta local
/goal_pose              # geometry_msgs/PoseStamped - Objetivo navegación
/map                    # nav_msgs/OccupancyGrid - Mapa ocupacional

# Estado del sistema
/system_status          # std_msgs/String - Estado general
/robot_state           # tadeo_ecar_msgs/ECarStatus - Estado eCar
/diagnostics           # diagnostic_msgs/DiagnosticArray - Diagnósticos
/emergency_stop        # std_msgs/Bool - Parada de emergencia

# Percepción
/detected_obstacles    # geometry_msgs/PointStamped - Obstáculos detectados
/traffic_signs         # tadeo_ecar_msgs/TrafficSign - Señales detectadas
/lane_detection        # tadeo_ecar_msgs/LaneInfo - Detección de carriles

# Comportamientos
/behavior_status       # std_msgs/String - Estado comportamiento actual
/mission_status        # tadeo_ecar_msgs/MissionStatus - Estado de misión
```

### Configuración de Tópicos

```yaml
# config/topic_config.yaml
topic_remapping:
  # Remapeo para evitar conflictos
  laser_scan: "/ecar/sensors/scan"
  camera_image: "/ecar/sensors/camera/image_raw"
  cmd_vel: "/ecar/control/cmd_vel"
  
topic_qos:
  # QoS específico por tópico
  "/scan":
    reliability: "best_effort"
    history: "keep_last"
    depth: 10
    
  "/cmd_vel":
    reliability: "reliable"
    history: "keep_last"
    depth: 1
    
  "/emergency_stop":
    reliability: "reliable"
    durability: "transient_local"
    history: "keep_last"
    depth: 1
```

## Herramientas de Debugging

### Herramientas CLI

```bash
# Ver tópicos activos
ros2 topic list

# Información detallada de un tópico
ros2 topic info /cmd_vel
# Type: geometry_msgs/msg/Twist
# Publisher count: 1
# Subscription count: 2

# Escuchar mensajes
ros2 topic echo /scan

# Ver frecuencia de publicación
ros2 topic hz /cmd_vel

# Ver ancho de banda
ros2 topic bw /scan

# Publicar mensaje de prueba
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'

# Ver tipo de mensaje
ros2 interface show geometry_msgs/msg/Twist
```

### Herramientas Gráficas

```bash
# Ver grafo de nodos y tópicos
rqt_graph

# Plotear datos en tiempo real
rqt_plot /battery_state/percentage

# Consola de logs
rqt_console

# Herramienta de topics
rqt_topic
```

### Debugging de QoS

```bash
# Ver incompatibilidades de QoS
ros2 topic info /cmd_vel --verbose

# Ejemplo de salida con problemas:
# Publishers:
#   * /navigation_node (1 connection)
#     QoS Profile: Reliability=RELIABLE, History=KEEP_LAST, Depth=1
# Subscriptions:
#   * /wheel_controller_node (0 connections)  # ← Sin conexión!
#     QoS Profile: Reliability=BEST_EFFORT, History=KEEP_LAST, Depth=10
#   Warning: Publisher and subscription are not compatible
```

### Script de Monitoreo

```bash
#!/bin/bash
# scripts/monitor_topics.sh

echo "=== eCar Topic Monitor ==="
echo "Press Ctrl+C to stop"

while true; do
    clear
    echo "=== Active Topics $(date) ==="
    
    echo "📡 Sensor Topics:"
    ros2 topic hz /scan --once 2>/dev/null | head -1 || echo "  /scan: No data"
    ros2 topic hz /camera/image_raw --once 2>/dev/null | head -1 || echo "  /camera/image_raw: No data"
    ros2 topic hz /imu/data --once 2>/dev/null | head -1 || echo "  /imu/data: No data"
    ros2 topic hz /battery_state --once 2>/dev/null | head -1 || echo "  /battery_state: No data"
    
    echo ""
    echo "🎮 Control Topics:"
    ros2 topic hz /cmd_vel --once 2>/dev/null | head -1 || echo "  /cmd_vel: No data"
    ros2 topic hz /odom --once 2>/dev/null | head -1 || echo "  /odom: No data"
    
    echo ""
    echo "🚨 Status Topics:"
    ros2 topic hz /system_status --once 2>/dev/null | head -1 || echo "  /system_status: No data"
    ros2 topic hz /emergency_stop --once 2>/dev/null | head -1 || echo "  /emergency_stop: No data"
    
    echo ""
    echo "📊 Topic Count: $(ros2 topic list | wc -l)"
    echo "🔗 Node Count: $(ros2 node list | wc -l)"
    
    sleep 2
done
```

## Ejercicios Prácticos

### Ejercicio 1: Publisher de Temperatura

Crear un nodo que publique temperatura del sistema:

```python
#!/usr/bin/env python3
# TODO: Implementar TemperaturePublisher
class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher_node')
        # TODO: Crear publisher para sensor_msgs/Temperature
        # TODO: Leer temperatura del sistema (/sys/class/thermal/)
        # TODO: Publicar cada segundo
```

### Ejercicio 2: Subscriber de Velocidad

Crear un nodo que monitoree velocidades del eCar:

```cpp
// TODO: Implementar VelocityMonitor
class VelocityMonitor : public rclcpp::Node
{
public:
    VelocityMonitor() : Node("velocity_monitor_node")
    {
        // TODO: Suscribirse a /cmd_vel y /odom
        // TODO: Calcular aceleración
        // TODO: Detectar frenado brusco
        // TODO: Publicar alertas si es necesario
    }
};
```

### Ejercicio 3: Integración con Sistema eCar

```bash
# 1. Compilar nodos
colcon build --packages-select tadeo_ecar_perception

# 2. Ejecutar sistema básico
ros2 launch tadeo_ecar_bringup minimal_system.launch.py

# 3. En terminales separadas, ejecutar tus nodos
ros2 run tadeo_ecar_perception temperature_publisher_node
ros2 run tadeo_ecar_perception velocity_monitor_node

# 4. Verificar comunicación
ros2 topic list | grep temperature
ros2 topic echo /velocity_alerts

# 5. Visualizar en rqt_graph
rqt_graph
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Concepto de tópicos**: Canales de comunicación asíncrona named
2. **Publisher/Subscriber**: Patrón desacoplado de comunicación
3. **Tipos de mensajes**: Estándar y personalizados para el eCar
4. **QoS**: Configuración de calidad de servicio según necesidades
5. **Implementación**: Ejemplos completos en C++ y Python
6. **Debugging**: Herramientas para monitorear y resolver problemas

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes el concepto de tópicos y su rol en ROS2
- [ ] Puedes crear publishers y subscribers básicos
- [ ] Comprendes los diferentes tipos de mensajes
- [ ] Sabes configurar QoS según el tipo de datos
- [ ] Puedes debuggear problemas de comunicación
- [ ] Has probado los ejemplos con el sistema eCar

### Próximo Capítulo

En el Capítulo 5 estudiaremos:
- Servicios en ROS2
- Comunicación síncrona cliente/servidor
- Definición de servicios personalizados
- Manejo de errores y timeouts
- Servicios del sistema eCar

## Referencias

- [ROS2 Topics Concepts](https://docs.ros.org/en/humble/Concepts/About-Topics.html)
- [Quality of Service](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [Message Types](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
- [rclcpp Publishers](https://docs.ros2.org/humble/api/rclcpp/classrclcpp_1_1Publisher.html)
- [rclpy Publishers](https://docs.ros2.org/humble/api/rclpy/api/publishers.html)