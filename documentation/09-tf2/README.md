# Capítulo 9: TF2 y Transformaciones en ROS2

## Tabla de Contenidos

1. [Concepto de TF2](#concepto-de-tf2)
2. [Sistema de Coordenadas](#sistema-de-coordenadas)
3. [Transformaciones Estáticas](#transformaciones-estáticas)
4. [Transformaciones Dinámicas](#transformaciones-dinámicas)
5. [Implementación en C++](#implementación-en-c++)
6. [Implementación en Python](#implementación-en-python)
7. [Frames del eCar](#frames-del-ecar)
8. [Herramientas TF2](#herramientas-tf2)

## Concepto de TF2

### ¿Qué es TF2?

TF2 (Transform Framework 2) es el sistema de transformaciones de coordenadas de ROS2 que mantiene las relaciones espaciales entre diferentes marcos de referencia (frames) a lo largo del tiempo.

```
                    Sistema TF2
                          |
    ┌─────────────────────┼────────────────────┐
    |                     |                    |
World Frame        Robot Frame         Sensor Frame
   /map               /base_link          /laser_frame
    │                     │                    │
    └─────────────────────┼────────────────────┘
                          │
                    TF Tree mantiene
                   todas las relaciones
```

### ¿Por qué TF2?

**1. Consistencia Espacial**
```cpp
// Sin TF2 - Manual y propenso a errores
point_in_base = point_in_laser - laser_offset_x - laser_offset_y;

// Con TF2 - Automático y preciso
tf_buffer.transform(point_in_laser, "base_link", "laser_frame");
```

**2. Transformaciones Temporales**
```cpp
// TF2 mantiene historial de transformaciones
auto transform = tf_buffer.lookupTransform(
    "base_link", "laser_frame", 
    tf2::TimePointZero  // Última transformación disponible
);
```

**3. Debugging Visual**
```bash
# Ver árbol completo de transformaciones
ros2 run tf2_tools view_frames
# Genera PDF con visualización del árbol TF
```

### Casos de Uso en el eCar

**Navegación:**
- Transformar objetivos del frame `/map` al frame `/base_link`
- Convertir obstáculos de `/laser_frame` a `/base_link`

**Control:**
- Calcular posición de ruedas respecto al centro del robot
- Transformar comandos de velocidad entre frames

**Percepción:**
- Fusionar datos de múltiples sensores en un frame común
- Proyectar detecciones de cámara al plano del suelo

## Sistema de Coordenadas

### Árbol TF del eCar 4WD4WS

```
                        /map
                          │
                        /odom
                          │
                      /base_link ── /base_footprint
                          │
           ┌──────────────┼──────────────┐
           │              │              │
      /laser_frame    /camera_frame   /imu_frame
                          │
                     /camera_optical_frame
           
    /base_link también conecta a:
           │
    ┌──────┼──────┬──────┬──────┐
    │      │      │      │      │
/wheel_fl /wheel_fr /wheel_rl /wheel_rr
    │      │      │      │      │
/steering_fl /steering_fr /steering_rl /steering_rr
```

### Convenciones de Frames

**Frame Principal: `/base_link`**
- Centro geométrico del robot
- Origen del sistema de coordenadas del robot
- X: hacia adelante, Y: hacia la izquierda, Z: hacia arriba

**Frame de Navegación: `/map`**
- Frame global fijo
- Referencia para navegación y localización
- Persiste entre sesiones

**Frame de Odometría: `/odom`**
- Frame local continuo
- Deriva con el tiempo
- Se resetea al reiniciar el sistema

### Convenciones REP-103

```
Convenciones ROS para marcos de coordenadas:

Robot Frame (/base_link):
  X: hacia adelante (rojo)
  Y: hacia la izquierda (verde) 
  Z: hacia arriba (azul)

Sensor Frames:
  X: hacia adelante desde el sensor
  Y: hacia la izquierda desde el sensor
  Z: hacia arriba desde el sensor

Camera Optical Frame:
  X: hacia la derecha en la imagen
  Y: hacia abajo en la imagen
  Z: hacia adelante desde la cámara
```

### Configuración de Frames del eCar

```yaml
# config/tf_configuration.yaml
tf_configuration:
  # Frame principal
  base_frame: "base_link"
  footprint_frame: "base_footprint"
  
  # Frames de navegación
  map_frame: "map"
  odom_frame: "odom"
  
  # Frames de sensores
  laser_frame: "laser_frame"
  camera_frame: "camera_frame"
  camera_optical_frame: "camera_optical_frame"
  imu_frame: "imu_frame"
  
  # Frames de ruedas (4WD4WS)
  wheel_frames:
    front_left: "wheel_fl"
    front_right: "wheel_fr"
    rear_left: "wheel_rl"
    rear_right: "wheel_rr"
    
  # Frames de dirección (4WS)
  steering_frames:
    front_left: "steering_fl"
    front_right: "steering_fr"
    rear_left: "steering_rl"
    rear_right: "steering_rr"
```

## Transformaciones Estáticas

### Publisher de Transformaciones Estáticas

```cpp
// include/tadeo_ecar_description/static_tf_publisher.hpp
#ifndef TADEO_ECAR_DESCRIPTION__STATIC_TF_PUBLISHER_HPP_
#define TADEO_ECAR_DESCRIPTION__STATIC_TF_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tadeo_ecar_description
{
class StaticTFPublisher : public rclcpp::Node
{
public:
    StaticTFPublisher();

private:
    void publishStaticTransforms();
    geometry_msgs::msg::TransformStamped createTransform(
        const std::string& parent_frame,
        const std::string& child_frame,
        double x, double y, double z,
        double roll, double pitch, double yaw);
    
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    
    // Parámetros del robot
    struct RobotDimensions {
        double wheel_base;      // Distancia entre ejes
        double track_width;     // Ancho de vía
        double wheel_radius;    // Radio de ruedas
        double sensor_height;   // Altura de sensores
    } robot_dims_;
};
}  // namespace tadeo_ecar_description

#endif  // TADEO_ECAR_DESCRIPTION__STATIC_TF_PUBLISHER_HPP_
```

```cpp
// src/static_tf_publisher.cpp
#include "tadeo_ecar_description/static_tf_publisher.hpp"
#include <cmath>

namespace tadeo_ecar_description
{
StaticTFPublisher::StaticTFPublisher() : Node("static_tf_publisher")
{
    // Declarar parámetros
    this->declare_parameter("robot.wheel_base", 1.2);
    this->declare_parameter("robot.track_width", 0.8);
    this->declare_parameter("robot.wheel_radius", 0.15);
    this->declare_parameter("robot.sensor_height", 0.3);
    
    // Posiciones de sensores
    this->declare_parameter("sensors.laser.x", 0.3);
    this->declare_parameter("sensors.laser.y", 0.0);
    this->declare_parameter("sensors.laser.z", 0.2);
    
    this->declare_parameter("sensors.camera.x", 0.4);
    this->declare_parameter("sensors.camera.y", 0.0);
    this->declare_parameter("sensors.camera.z", 0.3);
    
    this->declare_parameter("sensors.imu.x", 0.0);
    this->declare_parameter("sensors.imu.y", 0.0);
    this->declare_parameter("sensors.imu.z", 0.1);
    
    // Cargar parámetros
    robot_dims_.wheel_base = this->get_parameter("robot.wheel_base").as_double();
    robot_dims_.track_width = this->get_parameter("robot.track_width").as_double();
    robot_dims_.wheel_radius = this->get_parameter("robot.wheel_radius").as_double();
    robot_dims_.sensor_height = this->get_parameter("robot.sensor_height").as_double();
    
    // Crear broadcaster
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    
    // Publicar transformaciones estáticas
    publishStaticTransforms();
    
    RCLCPP_INFO(this->get_logger(), "Static TF publisher initialized");
    RCLCPP_INFO(this->get_logger(), "Robot dimensions: wheelbase=%.2fm, track=%.2fm", 
                robot_dims_.wheel_base, robot_dims_.track_width);
}

void StaticTFPublisher::publishStaticTransforms()
{
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    
    // 1. Base_footprint a base_link
    static_transforms.push_back(createTransform(
        "base_footprint", "base_link",
        0.0, 0.0, robot_dims_.wheel_radius,  // Elevado por radio de rueda
        0.0, 0.0, 0.0
    ));
    
    // 2. Transformaciones de sensores
    double laser_x = this->get_parameter("sensors.laser.x").as_double();
    double laser_y = this->get_parameter("sensors.laser.y").as_double();
    double laser_z = this->get_parameter("sensors.laser.z").as_double();
    
    static_transforms.push_back(createTransform(
        "base_link", "laser_frame",
        laser_x, laser_y, laser_z,
        0.0, 0.0, 0.0
    ));
    
    double camera_x = this->get_parameter("sensors.camera.x").as_double();
    double camera_y = this->get_parameter("sensors.camera.y").as_double();
    double camera_z = this->get_parameter("sensors.camera.z").as_double();
    
    static_transforms.push_back(createTransform(
        "base_link", "camera_frame",
        camera_x, camera_y, camera_z,
        0.0, 0.0, 0.0
    ));
    
    // Camera optical frame (rotado 90° para convención de cámara)
    static_transforms.push_back(createTransform(
        "camera_frame", "camera_optical_frame",
        0.0, 0.0, 0.0,
        -M_PI/2, 0.0, -M_PI/2  // Rotación para frame óptico
    ));
    
    double imu_x = this->get_parameter("sensors.imu.x").as_double();
    double imu_y = this->get_parameter("sensors.imu.y").as_double();
    double imu_z = this->get_parameter("sensors.imu.z").as_double();
    
    static_transforms.push_back(createTransform(
        "base_link", "imu_frame",
        imu_x, imu_y, imu_z,
        0.0, 0.0, 0.0
    ));
    
    // 3. Transformaciones de ruedas (4WD4WS)
    double wheel_offset_x = robot_dims_.wheel_base / 2.0;
    double wheel_offset_y = robot_dims_.track_width / 2.0;
    
    // Ruedas delanteras
    static_transforms.push_back(createTransform(
        "base_link", "wheel_fl",
        wheel_offset_x, wheel_offset_y, 0.0,
        0.0, 0.0, 0.0
    ));
    
    static_transforms.push_back(createTransform(
        "base_link", "wheel_fr", 
        wheel_offset_x, -wheel_offset_y, 0.0,
        0.0, 0.0, 0.0
    ));
    
    // Ruedas traseras
    static_transforms.push_back(createTransform(
        "base_link", "wheel_rl",
        -wheel_offset_x, wheel_offset_y, 0.0,
        0.0, 0.0, 0.0
    ));
    
    static_transforms.push_back(createTransform(
        "base_link", "wheel_rr",
        -wheel_offset_x, -wheel_offset_y, 0.0,
        0.0, 0.0, 0.0
    ));
    
    // 4. Frames de dirección (4WS) - inicialmente alineados con ruedas
    static_transforms.push_back(createTransform(
        "wheel_fl", "steering_fl",
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    ));
    
    static_transforms.push_back(createTransform(
        "wheel_fr", "steering_fr",
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    ));
    
    static_transforms.push_back(createTransform(
        "wheel_rl", "steering_rl",
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    ));
    
    static_transforms.push_back(createTransform(
        "wheel_rr", "steering_rr",
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    ));
    
    // Publicar todas las transformaciones estáticas
    static_broadcaster_->sendTransform(static_transforms);
    
    RCLCPP_INFO(this->get_logger(), 
                "Published %zu static transforms", static_transforms.size());
}

geometry_msgs::msg::TransformStamped StaticTFPublisher::createTransform(
    const std::string& parent_frame,
    const std::string& child_frame, 
    double x, double y, double z,
    double roll, double pitch, double yaw)
{
    geometry_msgs::msg::TransformStamped transform;
    
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    
    // Posición
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    
    // Orientación (convertir Euler a quaternion)
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform.transform.rotation = tf2::toMsg(q);
    
    return transform;
}
}  // namespace tadeo_ecar_description

// Main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_description::StaticTFPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Launch con TF Estático

```python
# launch/static_tf.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='tadeo_ecar'),
        
        # Publisher de transformaciones estáticas
        Node(
            package='tadeo_ecar_description',
            executable='static_tf_publisher_node',
            name='static_tf_publisher',
            namespace=LaunchConfiguration('robot_name'),
            parameters=[{
                'robot.wheel_base': 1.2,
                'robot.track_width': 0.8,
                'robot.wheel_radius': 0.15,
                'sensors.laser.x': 0.3,
                'sensors.laser.y': 0.0,
                'sensors.laser.z': 0.2,
                'sensors.camera.x': 0.4,
                'sensors.camera.y': 0.0,
                'sensors.camera.z': 0.3
            }],
            output='screen'
        ),
        
        # Alternativa: Usar static_transform_publisher directamente
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '0', '0', '0.15',  # x, y, z
                '0', '0', '0',     # roll, pitch, yaw
                'base_footprint', 'base_link'
            ]
        ),
        
        # Más transformaciones estáticas...
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=[
                '0.3', '0.0', '0.2',
                '0.0', '0.0', '0.0',
                'base_link', 'laser_frame'
            ]
        )
    ])
```

## Transformaciones Dinámicas

### Broadcaster de TF Dinámico

```cpp
// include/tadeo_ecar_localization/tf_broadcaster.hpp
#ifndef TADEO_ECAR_LOCALIZATION__TF_BROADCASTER_HPP_
#define TADEO_ECAR_LOCALIZATION__TF_BROADCASTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace tadeo_ecar_localization
{
class TFBroadcaster : public rclcpp::Node
{
public:
    TFBroadcaster();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void publishMapToOdom();
    
    // TF2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    
    // Timer
    rclcpp::TimerInterface::SharedPtr tf_timer_;
    
    // Estado
    geometry_msgs::msg::TransformStamped map_to_odom_transform_;
    geometry_msgs::msg::TransformStamped odom_to_base_transform_;
    bool have_map_to_odom_;
    bool have_odom_to_base_;
    
    // Parámetros
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    double tf_publish_rate_;
};
}  // namespace tadeo_ecar_localization

#endif  // TADEO_ECAR_LOCALIZATION__TF_BROADCASTER_HPP_
```

```cpp
// src/tf_broadcaster.cpp
#include "tadeo_ecar_localization/tf_broadcaster.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace tadeo_ecar_localization
{
TFBroadcaster::TFBroadcaster() 
    : Node("tf_broadcaster"),
      have_map_to_odom_(false),
      have_odom_to_base_(false)
{
    // Parámetros
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("tf_publish_rate", 50.0);
    
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    tf_publish_rate_ = this->get_parameter("tf_publish_rate").as_double();
    
    // TF2 setup
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&TFBroadcaster::odomCallback, this, std::placeholders::_1));
    
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", 10,
        std::bind(&TFBroadcaster::amclCallback, this, std::placeholders::_1));
    
    // Timer para publicar transformaciones
    tf_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / tf_publish_rate_),
        std::bind(&TFBroadcaster::publishMapToOdom, this));
    
    RCLCPP_INFO(this->get_logger(), "TF Broadcaster initialized");
    RCLCPP_INFO(this->get_logger(), "Frames: %s -> %s -> %s", 
                map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
}

void TFBroadcaster::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Crear transformación odom -> base_link
    odom_to_base_transform_.header = msg->header;
    odom_to_base_transform_.header.frame_id = odom_frame_;
    odom_to_base_transform_.child_frame_id = base_frame_;
    
    // Copiar pose
    odom_to_base_transform_.transform.translation.x = msg->pose.pose.position.x;
    odom_to_base_transform_.transform.translation.y = msg->pose.pose.position.y;
    odom_to_base_transform_.transform.translation.z = msg->pose.pose.position.z;
    odom_to_base_transform_.transform.rotation = msg->pose.pose.orientation;
    
    // Publicar transformación
    tf_broadcaster_->sendTransform(odom_to_base_transform_);
    
    have_odom_to_base_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Published odom->base_link: (%.3f, %.3f, %.3f)",
                 msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 tf2::getYaw(msg->pose.pose.orientation));
}

void TFBroadcaster::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!have_odom_to_base_) {
        RCLCPP_WARN(this->get_logger(), "No odom->base_link transform available");
        return;
    }
    
    try {
        // Obtener transformación actual odom->base_link
        auto odom_to_base = tf_buffer_->lookupTransform(
            odom_frame_, base_frame_, tf2::TimePointZero);
        
        // AMCL pose es map->base_link
        geometry_msgs::msg::TransformStamped map_to_base;
        map_to_base.header = msg->header;
        map_to_base.header.frame_id = map_frame_;
        map_to_base.child_frame_id = base_frame_;
        map_to_base.transform.translation.x = msg->pose.pose.position.x;
        map_to_base.transform.translation.y = msg->pose.pose.position.y;
        map_to_base.transform.translation.z = msg->pose.pose.position.z;
        map_to_base.transform.rotation = msg->pose.pose.orientation;
        
        // Calcular map->odom = map->base_link * (odom->base_link)^-1
        tf2::Transform tf_map_to_base;
        tf2::fromMsg(map_to_base.transform, tf_map_to_base);
        
        tf2::Transform tf_odom_to_base;
        tf2::fromMsg(odom_to_base.transform, tf_odom_to_base);
        
        tf2::Transform tf_map_to_odom = tf_map_to_base * tf_odom_to_base.inverse();
        
        // Crear mensaje de transformación
        map_to_odom_transform_.header.stamp = msg->header.stamp;
        map_to_odom_transform_.header.frame_id = map_frame_;
        map_to_odom_transform_.child_frame_id = odom_frame_;
        map_to_odom_transform_.transform = tf2::toMsg(tf_map_to_odom);
        
        have_map_to_odom_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "Updated map->odom transform from AMCL");
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not compute map->odom: %s", ex.what());
    }
}

void TFBroadcaster::publishMapToOdom()
{
    if (have_map_to_odom_) {
        // Actualizar timestamp
        map_to_odom_transform_.header.stamp = this->get_clock()->now();
        
        // Publicar transformación map->odom
        tf_broadcaster_->sendTransform(map_to_odom_transform_);
        
        RCLCPP_DEBUG(this->get_logger(), "Published map->odom transform");
    }
}
}  // namespace tadeo_ecar_localization

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_localization::TFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Implementación en C++

### Listener de TF2

```cpp
// include/tadeo_ecar_navigation/tf_listener.hpp
#ifndef TADEO_ECAR_NAVIGATION__TF_LISTENER_HPP_
#define TADEO_ECAR_NAVIGATION__TF_LISTENER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace tadeo_ecar_navigation
{
class TFListener : public rclcpp::Node
{
public:
    TFListener();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    // TF2 operations
    bool transformPoint(const geometry_msgs::msg::PointStamped& input,
                       const std::string& target_frame,
                       geometry_msgs::msg::PointStamped& output);
    
    bool transformPose(const geometry_msgs::msg::PoseStamped& input,
                      const std::string& target_frame,
                      geometry_msgs::msg::PoseStamped& output);
    
    geometry_msgs::msg::PoseStamped getCurrentRobotPose(const std::string& target_frame);
    
    void publishObstacleMarkers(const std::vector<geometry_msgs::msg::Point>& obstacles);
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformed_goal_pub_;
    
    // Timer
    rclcpp::TimerInterface::SharedPtr status_timer_;
    
    // Parámetros
    std::string base_frame_;
    std::string map_frame_;
    double transform_tolerance_;
};
}  // namespace tadeo_ecar_navigation

#endif  // TADEO_ECAR_NAVIGATION__TF_LISTENER_HPP_
```

```cpp
// src/tf_listener.cpp
#include "tadeo_ecar_navigation/tf_listener.hpp"
#include <tf2/utils.h>
#include <cmath>

namespace tadeo_ecar_navigation
{
TFListener::TFListener() : Node("tf_listener")
{
    // Parámetros
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("transform_tolerance", 0.1);
    
    base_frame_ = this->get_parameter("base_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    transform_tolerance_ = this->get_parameter("transform_tolerance").as_double();
    
    // TF2 setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&TFListener::scanCallback, this, std::placeholders::_1));
    
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        std::bind(&TFListener::goalCallback, this, std::placeholders::_1));
    
    // Publishers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "obstacle_markers", 10);
    
    transformed_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "goal_pose_base_link", 10);
    
    // Timer para estado
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            auto pose = getCurrentRobotPose(map_frame_);
            if (!pose.header.frame_id.empty()) {
                RCLCPP_DEBUG(this->get_logger(), 
                           "Robot pose in %s: (%.2f, %.2f, %.2f°)",
                           map_frame_.c_str(),
                           pose.pose.position.x,
                           pose.pose.position.y,
                           tf2::getYaw(pose.pose.orientation) * 180.0 / M_PI);
            }
        });
    
    RCLCPP_INFO(this->get_logger(), "TF Listener initialized");
    RCLCPP_INFO(this->get_logger(), "Base frame: %s, Map frame: %s", 
                base_frame_.c_str(), map_frame_.c_str());
}

void TFListener::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::vector<geometry_msgs::msg::Point> obstacles_in_map;
    
    // Procesar puntos del LiDAR y transformarlos al frame del mapa
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float range = msg->ranges[i];
        
        // Filtrar puntos válidos
        if (range < msg->range_min || range > msg->range_max || 
            std::isnan(range) || std::isinf(range)) {
            continue;
        }
        
        // Convertir a coordenadas cartesianas en frame del láser
        float angle = msg->angle_min + i * msg->angle_increment;
        
        geometry_msgs::msg::PointStamped point_laser;
        point_laser.header = msg->header;
        point_laser.point.x = range * cos(angle);
        point_laser.point.y = range * sin(angle);
        point_laser.point.z = 0.0;
        
        // Transformar al frame del mapa
        geometry_msgs::msg::PointStamped point_map;
        if (transformPoint(point_laser, map_frame_, point_map)) {
            obstacles_in_map.push_back(point_map.point);
        }
    }
    
    // Publicar marcadores de obstáculos
    publishObstacleMarkers(obstacles_in_map);
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Transformed %zu laser points to map frame", obstacles_in_map.size());
}

void TFListener::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Transformar objetivo al frame base_link
    geometry_msgs::msg::PoseStamped goal_in_base;
    if (transformPose(*msg, base_frame_, goal_in_base)) {
        transformed_goal_pub_->publish(goal_in_base);
        
        // Calcular distancia y ángulo al objetivo
        double distance = sqrt(pow(goal_in_base.pose.position.x, 2) + 
                              pow(goal_in_base.pose.position.y, 2));
        double angle = atan2(goal_in_base.pose.position.y, goal_in_base.pose.position.x);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Goal in base_link: distance=%.2fm, angle=%.1f°",
                    distance, angle * 180.0 / M_PI);
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to transform goal to base_link");
    }
}

bool TFListener::transformPoint(const geometry_msgs::msg::PointStamped& input,
                               const std::string& target_frame,
                               geometry_msgs::msg::PointStamped& output)
{
    try {
        tf_buffer_->transform(input, output, target_frame, 
                             tf2::durationFromSec(transform_tolerance_));
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), 
                    "Transform failed from %s to %s: %s",
                    input.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

bool TFListener::transformPose(const geometry_msgs::msg::PoseStamped& input,
                              const std::string& target_frame,
                              geometry_msgs::msg::PoseStamped& output)
{
    try {
        tf_buffer_->transform(input, output, target_frame,
                             tf2::durationFromSec(transform_tolerance_));
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Transform failed from %s to %s: %s", 
                    input.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

geometry_msgs::msg::PoseStamped TFListener::getCurrentRobotPose(const std::string& target_frame)
{
    geometry_msgs::msg::PoseStamped robot_pose;
    
    try {
        auto transform = tf_buffer_->lookupTransform(
            target_frame, base_frame_, tf2::TimePointZero);
        
        robot_pose.header.stamp = transform.header.stamp;
        robot_pose.header.frame_id = target_frame;
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG(this->get_logger(), 
                     "Could not get robot pose in %s: %s", target_frame.c_str(), ex.what());
        // Retornar pose vacía
        robot_pose.header.frame_id = "";
    }
    
    return robot_pose;
}

void TFListener::publishObstacleMarkers(const std::vector<geometry_msgs::msg::Point>& obstacles)
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (size_t i = 0; i < obstacles.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "obstacles";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position = obstacles[i];
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        
        marker_array.markers.push_back(marker);
    }
    
    marker_pub_->publish(marker_array);
}
}  // namespace tadeo_ecar_navigation

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_navigation::TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Implementación en Python

### TF2 en Python

```python
#!/usr/bin/env python3
# scripts/tf_monitor.py

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import tf2_py as tf2
import math

class TFMonitor(Node):
    def __init__(self):
        super().__init__('tf_monitor')
        
        # Parámetros
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('laser_frame', 'laser_frame')
        self.declare_parameter('monitor_rate', 2.0)
        
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.monitor_rate = self.get_parameter('monitor_rate').value
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'tf_visualization', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_pose_map', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Timers
        self.monitor_timer = self.create_timer(
            1.0 / self.monitor_rate, self.monitor_transforms)
        
        self.visualization_timer = self.create_timer(
            0.1, self.publish_tf_visualization)
        
        # Estado
        self.last_transforms = {}
        self.transform_stats = {}
        
        self.get_logger().info('TF Monitor initialized')
        self.get_logger().info(f'Monitoring frames: {self.map_frame} -> {self.odom_frame} -> {self.base_frame}')
    
    def monitor_transforms(self):
        """Monitorear estado de transformaciones"""
        
        transforms_to_check = [
            (self.map_frame, self.odom_frame),
            (self.odom_frame, self.base_frame),
            (self.base_frame, self.laser_frame),
            (self.map_frame, self.base_frame)
        ]
        
        for parent, child in transforms_to_check:
            try:
                # Obtener transformación más reciente
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
                
                # Calcular estadísticas
                transform_key = f"{parent}->{child}"
                current_time = self.get_clock().now()
                
                if transform_key in self.last_transforms:
                    # Calcular frecuencia
                    time_diff = (current_time - self.last_transforms[transform_key]).nanoseconds / 1e9
                    if time_diff > 0:
                        frequency = 1.0 / time_diff
                        
                        if transform_key not in self.transform_stats:
                            self.transform_stats[transform_key] = {'freq_samples': []}
                        
                        self.transform_stats[transform_key]['freq_samples'].append(frequency)
                        
                        # Mantener solo últimas 10 muestras
                        if len(self.transform_stats[transform_key]['freq_samples']) > 10:
                            self.transform_stats[transform_key]['freq_samples'].pop(0)
                
                self.last_transforms[transform_key] = current_time
                
                # Log información periódica
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z
                
                self.get_logger().debug(
                    f"Transform {parent}->{child}: ({x:.3f}, {y:.3f}, {z:.3f})")
                
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f"Transform {parent}->{child} not available: {e}")
        
        # Publicar estadísticas periódicamente
        self.log_transform_statistics()
    
    def log_transform_statistics(self):
        """Log estadísticas de transformaciones"""
        
        for transform_key, stats in self.transform_stats.items():
            if 'freq_samples' in stats and len(stats['freq_samples']) > 0:
                avg_freq = np.mean(stats['freq_samples'])
                min_freq = np.min(stats['freq_samples'])
                max_freq = np.max(stats['freq_samples'])
                
                self.get_logger().info(
                    f"Transform {transform_key}: "
                    f"avg_freq={avg_freq:.1f}Hz, range=[{min_freq:.1f}, {max_freq:.1f}]Hz")
    
    def scan_callback(self, msg):
        """Procesar scan y transformar puntos"""
        
        try:
            # Transformar algunos puntos de laser a map frame
            sample_indices = range(0, len(msg.ranges), len(msg.ranges) // 10)  # 10 puntos de muestra
            
            for i in sample_indices:
                if (msg.ranges[i] < msg.range_min or 
                    msg.ranges[i] > msg.range_max or
                    math.isnan(msg.ranges[i]) or 
                    math.isinf(msg.ranges[i])):
                    continue
                
                # Crear punto en frame laser
                angle = msg.angle_min + i * msg.angle_increment
                point_laser = PointStamped()
                point_laser.header = msg.header
                point_laser.point.x = msg.ranges[i] * math.cos(angle)
                point_laser.point.y = msg.ranges[i] * math.sin(angle)
                point_laser.point.z = 0.0
                
                # Transformar a map frame
                try:
                    point_map = self.tf_buffer.transform(point_laser, self.map_frame)
                    
                    # Log punto transformado (solo debug)
                    self.get_logger().debug(
                        f"Laser point transformed: "
                        f"({point_laser.point.x:.2f}, {point_laser.point.y:.2f}) -> "
                        f"({point_map.point.x:.2f}, {point_map.point.y:.2f})")
                    
                except tf2_ros.TransformException as e:
                    self.get_logger().debug(f"Could not transform laser point: {e}")
                    
        except Exception as e:
            self.get_logger().error(f"Error in scan callback: {e}")
    
    def publish_tf_visualization(self):
        """Publicar visualización de frames TF"""
        
        marker_array = MarkerArray()
        
        frames_to_visualize = [
            self.base_frame, self.laser_frame, 'camera_frame', 'imu_frame'
        ]
        
        for i, frame in enumerate(frames_to_visualize):
            try:
                # Obtener pose del frame en map
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, frame, rclpy.time.Time())
                
                # Crear marcador de flecha para el frame
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "tf_frames"
                marker.id = i
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                
                # Posición y orientación
                marker.pose.position.x = transform.transform.translation.x
                marker.pose.position.y = transform.transform.translation.y
                marker.pose.position.z = transform.transform.translation.z
                marker.pose.orientation = transform.transform.rotation
                
                # Escala
                marker.scale.x = 0.3  # Longitud de flecha
                marker.scale.y = 0.05  # Ancho de flecha
                marker.scale.z = 0.05  # Alto de flecha
                
                # Color según frame
                if frame == self.base_frame:
                    marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0  # Verde
                elif frame == self.laser_frame:
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0  # Rojo
                elif 'camera' in frame:
                    marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0  # Azul
                else:
                    marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0  # Amarillo
                
                marker.color.a = 0.8
                marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
                
                marker_array.markers.append(marker)
                
                # Crear marcador de texto para el nombre del frame
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.ns = "tf_frame_labels"
                text_marker.id = i + 100
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                text_marker.pose = marker.pose
                text_marker.pose.position.z += 0.2  # Elevar texto
                
                text_marker.scale.z = 0.1  # Tamaño de texto
                text_marker.color = marker.color
                text_marker.text = frame
                text_marker.lifetime = marker.lifetime
                
                marker_array.markers.append(text_marker)
                
            except tf2_ros.TransformException:
                # Frame no disponible, skip
                continue
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
        
        # Publicar pose actual del robot en map
        self.publish_robot_pose()
    
    def publish_robot_pose(self):
        """Publicar pose actual del robot en map frame"""
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
            
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.map_frame
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            
            self.pose_pub.publish(pose_msg)
            
        except tf2_ros.TransformException:
            # Transform no disponible
            pass
    
    def get_transform_info(self, parent_frame, child_frame):
        """Obtener información detallada de una transformación"""
        
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame, child_frame, rclpy.time.Time())
            
            # Extraer posición
            x = transform.transform.translation.x
            y = transform.transform.translation.y  
            z = transform.transform.translation.z
            
            # Extraer orientación (quaternion a euler)
            from tf_transformations import euler_from_quaternion
            quat = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            roll, pitch, yaw = euler_from_quaternion(quat)
            
            return {
                'translation': {'x': x, 'y': y, 'z': z},
                'rotation': {
                    'roll': roll, 'pitch': pitch, 'yaw': yaw,
                    'quaternion': quat
                },
                'timestamp': transform.header.stamp,
                'available': True
            }
            
        except tf2_ros.TransformException as e:
            return {
                'available': False,
                'error': str(e)
            }
    
    def check_tf_tree_health(self):
        """Verificar salud del árbol TF"""
        
        critical_transforms = [
            (self.map_frame, self.base_frame),
            (self.base_frame, self.laser_frame)
        ]
        
        issues = []
        
        for parent, child in critical_transforms:
            info = self.get_transform_info(parent, child)
            
            if not info['available']:
                issues.append(f"Missing transform: {parent} -> {child}")
            else:
                # Verificar si la transformación es muy antigua
                current_time = self.get_clock().now()
                transform_time = rclpy.time.Time.from_msg(info['timestamp'])
                age = (current_time - transform_time).nanoseconds / 1e9
                
                if age > 2.0:  # Más de 2 segundos
                    issues.append(f"Stale transform: {parent} -> {child} (age: {age:.1f}s)")
        
        if issues:
            self.get_logger().warn(f"TF Tree issues detected: {', '.join(issues)}")
            return False
        else:
            self.get_logger().debug("TF Tree health check passed")
            return True

def main(args=None):
    rclpy.init(args=args)
    
    monitor = TFMonitor()
    
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

## Frames del eCar

### Configuración Completa de Frames

```yaml
# config/tf_frames.yaml
# Configuración completa de frames TF para el eCar 4WD4WS

tf_configuration:
  # Frames base
  base_frame: "base_link"
  base_footprint_frame: "base_footprint"
  
  # Frames de navegación
  map_frame: "map"
  odom_frame: "odom"
  
  # Dimensiones del robot (metros)
  robot_dimensions:
    wheel_base: 1.2      # Distancia entre ejes
    track_width: 0.8     # Ancho de vía
    wheel_radius: 0.15   # Radio de ruedas
    robot_height: 0.6    # Altura total
    
  # Posiciones de sensores relativos a base_link
  sensor_positions:
    laser:
      frame: "laser_frame"
      x: 0.3
      y: 0.0
      z: 0.2
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
      
    camera:
      frame: "camera_frame"
      x: 0.4
      y: 0.0
      z: 0.3
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
      
    camera_optical:
      frame: "camera_optical_frame"
      parent: "camera_frame"
      x: 0.0
      y: 0.0
      z: 0.0
      roll: -1.5708   # -90°
      pitch: 0.0
      yaw: -1.5708    # -90°
      
    imu:
      frame: "imu_frame"
      x: 0.0
      y: 0.0
      z: 0.1
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
      
    gps:
      frame: "gps_frame"
      x: -0.2
      y: 0.0
      z: 0.4
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
  
  # Frames de ruedas (4WD4WS)
  wheel_frames:
    front_left:
      frame: "wheel_fl"
      x: 0.6   # wheel_base / 2
      y: 0.4   # track_width / 2
      z: 0.0
      
    front_right:
      frame: "wheel_fr" 
      x: 0.6
      y: -0.4
      z: 0.0
      
    rear_left:
      frame: "wheel_rl"
      x: -0.6
      y: 0.4
      z: 0.0
      
    rear_right:
      frame: "wheel_rr"
      x: -0.6
      y: -0.4
      z: 0.0
  
  # Frames de dirección (4WS)
  steering_frames:
    front_left:
      frame: "steering_fl"
      parent: "wheel_fl"
      x: 0.0
      y: 0.0
      z: 0.0
      # Orientación será dinámica
      
    front_right:
      frame: "steering_fr"
      parent: "wheel_fr"
      x: 0.0
      y: 0.0
      z: 0.0
      
    rear_left:
      frame: "steering_rl"
      parent: "wheel_rl"
      x: 0.0
      y: 0.0
      z: 0.0
      
    rear_right:
      frame: "steering_rr"
      parent: "wheel_rr"
      x: 0.0
      y: 0.0
      z: 0.0

# Configuración de publicación TF
tf_publishing:
  static_transforms:
    publish_rate: 100.0  # Hz
    
  dynamic_transforms:
    map_to_odom_rate: 50.0      # Hz
    odom_to_base_rate: 50.0     # Hz
    steering_angles_rate: 20.0   # Hz
    
  # Tolerancias
  transform_tolerance: 0.1      # segundos
  tf_lookup_timeout: 1.0        # segundos
```

### Launch File para TF del eCar

```python
# launch/tf_ecar.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Argumentos
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='tadeo_ecar')
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')
    
    # Configuración
    tf_config = PathJoinSubstitution([
        FindPackageShare('tadeo_ecar_config'),
        'config', 'tf_frames.yaml'
    ])
    
    return LaunchDescription([
        robot_name_arg,
        use_sim_arg,
        
        # Publisher de transformaciones estáticas
        Node(
            package='tadeo_ecar_description',
            executable='static_tf_publisher_node',
            name='static_tf_publisher',
            namespace=LaunchConfiguration('robot_name'),
            parameters=[tf_config],
            output='screen'
        ),
        
        # Broadcaster dinámico
        Node(
            package='tadeo_ecar_localization',
            executable='tf_broadcaster_node',
            name='tf_broadcaster',
            namespace=LaunchConfiguration('robot_name'),
            parameters=[{
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'tf_publish_rate': 50.0
            }],
            output='screen'
        ),
        
        # Monitor de TF
        Node(
            package='tadeo_ecar_utils',
            executable='tf_monitor.py',
            name='tf_monitor',
            namespace=LaunchConfiguration('robot_name'),
            parameters=[{
                'base_frame': 'base_link',
                'map_frame': 'map',
                'monitor_rate': 2.0
            }],
            output='screen'
        ),
        
        # Transformaciones estáticas adicionales usando static_transform_publisher
        
        # Base footprint a base link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=[
                '0', '0', '0.15',    # x, y, z
                '0', '0', '0',       # roll, pitch, yaw
                'base_footprint', 'base_link'
            ]
        ),
        
        # Sensor frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '0.3', '0.0', '0.2',
                '0.0', '0.0', '0.0',
                'base_link', 'laser_frame'
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '0.4', '0.0', '0.3',
                '0.0', '0.0', '0.0',
                'base_link', 'camera_frame'
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical_tf',
            arguments=[
                '0.0', '0.0', '0.0',
                '-1.5708', '0.0', '-1.5708',  # Rotación para frame óptico
                'camera_frame', 'camera_optical_frame'
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=[
                '0.0', '0.0', '0.1',
                '0.0', '0.0', '0.0',
                'base_link', 'imu_frame'
            ]
        )
    ])
```

## Herramientas TF2

### Herramientas de Línea de Comandos

```bash
# 1. Ver árbol TF completo
ros2 run tf2_tools view_frames
# Genera frames.pdf con visualización del árbol

# 2. Monitorear transformación específica
ros2 run tf2_ros tf2_echo map base_link
# Muestra transformación map -> base_link en tiempo real

# 3. Ver transformación en tiempo específico
ros2 run tf2_ros tf2_echo map base_link --time 1634567890.123

# 4. Listar todos los frames
ros2 topic echo /tf_static
ros2 topic echo /tf

# 5. Información de conectividad
ros2 run tf2_tools tf2_monitor map base_link
# Monitorea disponibilidad y frecuencia

# 6. Verificar cadena de transformaciones
ros2 service call /tf2_frames_as_yaml std_srvs/srv/Empty
```

### Script de Debugging TF

```bash
#!/bin/bash
# scripts/debug_tf.sh

echo "=== eCar TF Debugging Script ==="

# 1. Verificar nodos TF activos
echo "1. Active TF nodes:"
ros2 node list | grep -E "(tf|transform)" || echo "No TF nodes found"

# 2. Verificar tópicos TF
echo -e "\n2. TF topics:"
ros2 topic list | grep tf

# 3. Verificar frecuencias de TF
echo -e "\n3. TF frequencies:"
echo "Static TF:"
timeout 3s ros2 topic hz /tf_static || echo "No static TF"
echo "Dynamic TF:"
timeout 3s ros2 topic hz /tf || echo "No dynamic TF"

# 4. Verificar transformaciones críticas
echo -e "\n4. Critical transforms:"
critical_transforms=(
    "map odom"
    "odom base_link"
    "base_link laser_frame"
    "base_link camera_frame"
)

for transform in "${critical_transforms[@]}"; do
    echo "Checking $transform:"
    timeout 2s ros2 run tf2_ros tf2_echo $transform || echo "  ✗ Not available"
    echo ""
done

# 5. Generar diagrama del árbol TF
echo "5. Generating TF tree diagram..."
ros2 run tf2_tools view_frames
if [ -f frames.pdf ]; then
    echo "  ✓ TF tree saved to frames.pdf"
else
    echo "  ✗ Failed to generate TF tree"
fi

# 6. Verificar frames esperados
echo -e "\n6. Expected frames check:"
expected_frames=(
    "map"
    "odom" 
    "base_link"
    "base_footprint"
    "laser_frame"
    "camera_frame"
    "imu_frame"
    "wheel_fl"
    "wheel_fr"
    "wheel_rl"
    "wheel_rr"
)

for frame in "${expected_frames[@]}"; do
    # Verificar si el frame existe en TF
    if timeout 1s ros2 run tf2_ros tf2_echo map $frame >/dev/null 2>&1; then
        echo "  ✓ $frame"
    else
        echo "  ✗ $frame (missing)"
    fi
done

echo -e "\n=== TF Debugging Complete ==="
```

### Monitor de TF en Tiempo Real

```python
#!/usr/bin/env python3
# scripts/tf_realtime_monitor.py

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_msgs.msg import TFMessage
import time
import curses

class TFRealtimeMonitor(Node):
    def __init__(self):
        super().__init__('tf_realtime_monitor')
        
        # Estado
        self.tf_data = {}
        self.tf_frequencies = {}
        self.last_update_times = {}
        
        # Subscribers
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 100)
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback, 100)
        
        self.get_logger().info('TF Realtime Monitor started')
    
    def tf_callback(self, msg):
        """Callback para transformaciones dinámicas"""
        for transform in msg.transforms:
            frame_pair = f"{transform.header.frame_id} -> {transform.child_frame_id}"
            current_time = time.time()
            
            # Actualizar datos
            self.tf_data[frame_pair] = {
                'type': 'dynamic',
                'transform': transform,
                'last_seen': current_time
            }
            
            # Calcular frecuencia
            if frame_pair in self.last_update_times:
                dt = current_time - self.last_update_times[frame_pair]
                if dt > 0:
                    freq = 1.0 / dt
                    if frame_pair not in self.tf_frequencies:
                        self.tf_frequencies[frame_pair] = []
                    
                    self.tf_frequencies[frame_pair].append(freq)
                    # Mantener últimas 10 muestras
                    if len(self.tf_frequencies[frame_pair]) > 10:
                        self.tf_frequencies[frame_pair].pop(0)
            
            self.last_update_times[frame_pair] = current_time
    
    def tf_static_callback(self, msg):
        """Callback para transformaciones estáticas"""
        for transform in msg.transforms:
            frame_pair = f"{transform.header.frame_id} -> {transform.child_frame_id}"
            
            self.tf_data[frame_pair] = {
                'type': 'static',
                'transform': transform,
                'last_seen': time.time()
            }
    
    def get_display_data(self):
        """Obtener datos para mostrar"""
        display_data = []
        current_time = time.time()
        
        for frame_pair, data in self.tf_data.items():
            transform = data['transform']
            age = current_time - data['last_seen']
            
            # Calcular frecuencia promedio
            avg_freq = 0.0
            if frame_pair in self.tf_frequencies and self.tf_frequencies[frame_pair]:
                avg_freq = sum(self.tf_frequencies[frame_pair]) / len(self.tf_frequencies[frame_pair])
            
            # Posición
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Orientación (simplificada)
            qw = transform.transform.rotation.w
            qz = transform.transform.rotation.z
            yaw = 2.0 * math.atan2(qz, qw) * 180.0 / math.pi
            
            display_data.append({
                'frame_pair': frame_pair,
                'type': data['type'],
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'age': age,
                'frequency': avg_freq,
                'status': 'OK' if age < 2.0 else 'STALE'
            })
        
        return sorted(display_data, key=lambda x: x['frame_pair'])

def display_tf_monitor(stdscr, monitor):
    """Mostrar monitor TF usando curses"""
    
    # Configurar curses
    curses.curs_set(0)  # Ocultar cursor
    stdscr.nodelay(1)   # No bloquear en getch()
    stdscr.timeout(100) # Timeout de 100ms
    
    # Colores
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)  # OK
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK) # Warning
    curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)    # Error
    curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)   # Header
    
    while True:
        # Limpiar pantalla
        stdscr.clear()
        
        # Header
        stdscr.addstr(0, 0, "eCar TF Monitor - Press 'q' to quit", curses.color_pair(4) | curses.A_BOLD)
        stdscr.addstr(1, 0, "=" * 80, curses.color_pair(4))
        
        # Headers de columnas
        header = f"{'Frame Pair':<25} {'Type':<8} {'X':<8} {'Y':<8} {'Z':<8} {'Yaw':<8} {'Freq':<8} {'Age':<6} {'Status':<8}"
        stdscr.addstr(2, 0, header, curses.color_pair(4) | curses.A_BOLD)
        stdscr.addstr(3, 0, "-" * 80, curses.color_pair(4))
        
        # Datos
        display_data = monitor.get_display_data()
        row = 4
        
        for data in display_data[:15]:  # Mostrar máximo 15 filas
            # Elegir color según estado
            if data['status'] == 'OK':
                color = curses.color_pair(1)
            elif data['status'] == 'STALE':
                color = curses.color_pair(2)
            else:
                color = curses.color_pair(3)
            
            # Formatear fila
            line = f"{data['frame_pair']:<25} {data['type']:<8} {data['x']:<8.2f} {data['y']:<8.2f} {data['z']:<8.2f} {data['yaw']:<8.1f} {data['frequency']:<8.1f} {data['age']:<6.1f} {data['status']:<8}"
            
            try:
                stdscr.addstr(row, 0, line, color)
                row += 1
            except curses.error:
                break  # Pantalla llena
        
        # Información adicional
        info_row = row + 2
        stdscr.addstr(info_row, 0, f"Total transforms: {len(display_data)}", curses.color_pair(4))
        stdscr.addstr(info_row + 1, 0, f"Time: {time.strftime('%H:%M:%S')}", curses.color_pair(4))
        
        # Refresh
        stdscr.refresh()
        
        # Verificar input
        key = stdscr.getch()
        if key == ord('q') or key == ord('Q'):
            break
        
        # Procesar callbacks de ROS
        rclpy.spin_once(monitor, timeout_sec=0.1)

def main():
    rclpy.init()
    monitor = TFRealtimeMonitor()
    
    try:
        # Ejecutar interfaz curses
        curses.wrapper(display_tf_monitor, monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import math
    main()
```

## Ejercicios Prácticos

### Ejercicio 1: Transformador de Puntos

Crear un nodo que transforme puntos entre diferentes frames:

```cpp
// TODO: Implementar PointTransformer
class PointTransformer : public rclcpp::Node
{
public:
    PointTransformer() : Node("point_transformer")
    {
        // TODO: Configurar TF2 buffer y listener
        // TODO: Subscriber para puntos de entrada
        // TODO: Publisher para puntos transformados
        // TODO: Transformar entre laser_frame, base_link y map
    }
};
```

### Ejercicio 2: Calibración de Sensor

```python
# TODO: Implementar sensor calibration
class SensorCalibrator(Node):
    def __init__(self):
        super().__init__('sensor_calibrator')
        # TODO: Recopilar datos de transformación
        # TODO: Calcular offset de sensor automáticamente
        # TODO: Publicar transformación calibrada
        # TODO: Guardar calibración en archivo
```

### Ejercicio 3: Verificador de TF

```bash
# scripts/verify_ecar_tf.sh
# TODO: Script de verificación completa del árbol TF del eCar
# - Verificar que todos los frames esperados existen
# - Comprobar frecuencias de publicación
# - Validar transformaciones críticas
# - Generar reporte de salud TF
```

### Comandos de Testing

```bash
# 1. Verificar configuración completa
ros2 launch tadeo_ecar_description tf_ecar.launch.py

# 2. Monitorear transformaciones en tiempo real
ros2 run tadeo_ecar_utils tf_realtime_monitor.py

# 3. Verificar transformación específica
ros2 run tf2_ros tf2_echo map base_link

# 4. Generar diagrama del árbol TF
ros2 run tf2_tools view_frames

# 5. Debug problemas de TF
./scripts/debug_tf.sh
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Concepto de TF2**: Sistema de transformaciones espaciales con historial temporal
2. **Sistema de Coordenadas**: Árbol TF del eCar 4WD4WS con todos los frames
3. **Transformaciones Estáticas**: Configuración de sensores y componentes fijos
4. **Transformaciones Dinámicas**: Odometría, localización y movimiento
5. **Implementación**: Broadcasters y listeners en C++ y Python
6. **Frames del eCar**: Configuración completa para robot 4WD4WS
7. **Herramientas**: Debugging, monitoreo y visualización de TF
8. **Best Practices**: Convenciones y configuración robusta

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes el concepto y propósito de TF2
- [ ] Puedes configurar el árbol TF completo del eCar
- [ ] Sabes implementar broadcasters y listeners de TF
- [ ] Comprendes las convenciones de frames y coordenadas
- [ ] Puedes debuggear problemas de transformaciones
- [ ] Has configurado todos los frames del sistema eCar
- [ ] Sabes usar herramientas de monitoreo y visualización

### Próximo Capítulo

En el Capítulo 10 estudiaremos:
- Sistema de navegación autónoma en ROS2
- Stack Nav2 completo
- Costmaps y planificadores
- Behavior trees de navegación
- Implementación en el robot eCar 4WD4WS

## Referencias

- [TF2 Concepts](https://docs.ros.org/en/humble/Concepts/About-Tf2.html)
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [REP-103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [tf2_geometry_msgs](https://docs.ros.org/en/humble/p/tf2_geometry_msgs/)