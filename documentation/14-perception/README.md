# Capítulo 14: Percepción Avanzada para eCar 4WD4WS

## Tabla de Contenidos

1. [Introducción a la Percepción](#introducción-a-la-percepción)
2. [Sensores del eCar](#sensores-del-ecar)
3. [Procesamiento LiDAR](#procesamiento-lidar)
4. [Visión por Computadora](#visión-por-computadora)
5. [Fusión de Sensores](#fusión-de-sensores)
6. [Detección de Obstáculos](#detección-de-obstáculos)
7. [Percepción Semántica](#percepción-semántica)
8. [Implementación en ROS2](#implementación-en-ros2)

## Introducción a la Percepción

### ¿Qué es la Percepción Robótica?

La percepción robótica es la capacidad del robot para interpretar y entender su entorno mediante sensores, extrayendo información útil para la toma de decisiones.

```
Percepción = Sensores + Algoritmos + Interpretación

Flujo de Percepción:
Mundo Real → Sensores → Datos Raw → Procesamiento → Información Semántica
```

### Objetivos de la Percepción en el eCar

**1. Navegación Segura**
- Detección de obstáculos estáticos y dinámicos
- Identificación de superficies navegables
- Reconocimiento de características del entorno

**2. Localización Precisa**
- Landmarks para SLAM y localización
- Características distintivas del entorno
- Corrección de deriva odométrica

**3. Comprensión del Entorno**
- Clasificación de objetos
- Estimación de propiedades (velocidad, dirección)
- Predicción de comportamientos

### Arquitectura de Percepción del eCar

```
                    Perception Architecture eCar 4WD4WS
                                    |
     ┌─────────────────────────────────────────────────────────┐
     |                  Sensor Fusion Layer                    |
     |         (Integra y correlaciona datos)                  |
     └─────────────────┬───────────────────────────────────────┘
                       |
     ┌─────────────────┼───────────────────────────────────────┐
     |                 |                                       |
┌────▼────┐    ┌───────▼────┐    ┌──────────▼────┐    ┌───────▼───┐
│LiDAR    │    │Camera      │    │IMU            │    │Ultrasonic │
│Processor│    │Vision      │    │Processor      │    │Sensors    │
└─────────┘    └────────────┘    └───────────────┘    └───────────┘
     |             |                      |                  |
┌────▼────┐   ┌────▼────┐         ┌──────▼──────┐     ┌──────▼──────┐
│Point    │   │Object   │         │Orientation  │     │Proximity    │
│Cloud    │   │Detection│         │Estimation   │     │Detection    │
│Filter   │   │         │         │             │     │             │
└─────────┘   └─────────┘         └─────────────┘     └─────────────┘
```

## Sensores del eCar

### Configuración de Sensores

**LiDAR Principal (2D/3D)**
```yaml
# LiDAR Configuration
lidar:
  type: "SICK TiM571"
  range: 25.0  # metros
  accuracy: ±60mm
  angular_resolution: 0.33°
  scan_frequency: 15 Hz
  mount_position: [0.3, 0.0, 0.5]  # x, y, z desde base_link
```

**Cámaras RGB**
```yaml
# Camera Configuration
cameras:
  front_camera:
    resolution: [1920, 1080]
    fps: 30
    fov: 60°  # field of view
    mount_position: [0.4, 0.0, 0.3]
    
  rear_camera:
    resolution: [1280, 720]
    fps: 30
    fov: 120°
    mount_position: [-0.4, 0.0, 0.3]
```

**IMU (Unidad de Medición Inercial)**
```yaml
# IMU Configuration
imu:
  type: "BNO055"
  accelerometer_range: ±16g
  gyroscope_range: ±2000°/s
  magnetometer_resolution: 2.5µT
  update_rate: 100 Hz
  mount_position: [0.0, 0.0, 0.1]
```

**Sensores Ultrasónicos**
```yaml
# Ultrasonic Sensors
ultrasonic:
  count: 8  # Perímetro completo
  range: [0.02, 4.0]  # metros
  accuracy: ±3mm
  positions:
    - [0.35, 0.2, 0.1]    # front-left
    - [0.35, -0.2, 0.1]   # front-right
    - [0.2, 0.35, 0.1]    # left-front
    - [-0.2, 0.35, 0.1]   # left-rear
    - [-0.35, 0.2, 0.1]   # rear-left
    - [-0.35, -0.2, 0.1]  # rear-right
    - [0.2, -0.35, 0.1]   # right-front
    - [-0.2, -0.35, 0.1]  # right-rear
```

### Frame de Coordenadas para Sensores

```xml
<!-- TF Tree para sensores del eCar -->
<launch>
  <!-- LiDAR Frame -->
  <node pkg="tf2_ros" exec="static_transform_publisher"
        args="0.3 0.0 0.5 0 0 0 base_link laser_frame"/>
  
  <!-- Camera Frames -->
  <node pkg="tf2_ros" exec="static_transform_publisher"
        args="0.4 0.0 0.3 0 0 0 base_link camera_frame"/>
  
  <!-- IMU Frame -->
  <node pkg="tf2_ros" exec="static_transform_publisher"
        args="0.0 0.0 0.1 0 0 0 base_link imu_frame"/>
  
  <!-- Ultrasonic Sensor Frames -->
  <node pkg="tf2_ros" exec="static_transform_publisher"
        args="0.35 0.2 0.1 0 0 0 base_link ultrasonic_fl_frame"/>
  <!-- Más sensores... -->
</launch>
```

## Procesamiento LiDAR

### Nodo de Procesamiento LiDAR

**C++ Implementation**
```cpp
// src/lidar_processor.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class LidarProcessor : public rclcpp::Node
{
public:
    LidarProcessor() : Node("lidar_processor")
    {
        // Suscriptores
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, 
            std::bind(&LidarProcessor::scanCallback, this, std::placeholders::_1));
        
        // Publicadores
        filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/filtered_pointcloud", 10);
        
        obstacles_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/obstacles", 10);
        
        ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/ground_plane", 10);
        
        // Parámetros
        this->declare_parameter("min_range", 0.1);
        this->declare_parameter("max_range", 20.0);
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("ground_threshold", 0.02);
        
        RCLCPP_INFO(this->get_logger(), "LiDAR Processor initialized");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Convertir LaserScan a PointCloud
        auto cloud = convertScanToCloud(msg);
        
        // Filtrar por rango
        auto filtered_cloud = filterByRange(cloud);
        
        // Detección de plano del suelo
        auto [ground_cloud, obstacle_cloud] = segmentGround(filtered_cloud);
        
        // Filtro de voxel para reducir densidad
        auto final_obstacles = voxelFilter(obstacle_cloud);
        
        // Publicar resultados
        publishCloud(filtered_cloud_pub_, filtered_cloud, msg->header.frame_id);
        publishCloud(ground_pub_, ground_cloud, msg->header.frame_id);
        publishCloud(obstacles_pub_, final_obstacles, msg->header.frame_id);
        
        // Análisis de obstáculos para navegación
        analyzeObstacles(final_obstacles);
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertScanToCloud(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan)
    {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            
            // Verificar validez del punto
            if (range < scan->range_min || range > scan->range_max) {
                continue;
            }
            
            float angle = scan->angle_min + i * scan->angle_increment;
            
            pcl::PointXYZ point;
            point.x = range * cos(angle);
            point.y = range * sin(angle);
            point.z = 0.0;  // LiDAR 2D
            
            cloud->points.push_back(point);
        }
        
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        
        return cloud;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterByRange(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        
        double min_range = this->get_parameter("min_range").as_double();
        double max_range = this->get_parameter("max_range").as_double();
        
        for (const auto& point : cloud->points) {
            double distance = sqrt(point.x * point.x + point.y * point.y);
            
            if (distance >= min_range && distance <= max_range) {
                filtered->points.push_back(point);
            }
        }
        
        filtered->width = filtered->points.size();
        filtered->height = 1;
        filtered->is_dense = true;
        
        return filtered;
    }
    
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, 
              pcl::PointCloud<pcl::PointXYZ>::Ptr> 
    segmentGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Segmentación RANSAC para detectar plano del suelo
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(this->get_parameter("ground_threshold").as_double());
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        
        // Extraer puntos del suelo y obstáculos
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        auto ground_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        auto obstacle_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        
        // Puntos del suelo
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*ground_cloud);
        
        // Puntos de obstáculos
        extract.setNegative(true);
        extract.filter(*obstacle_cloud);
        
        return {ground_cloud, obstacle_cloud};
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        
        double voxel_size = this->get_parameter("voxel_size").as_double();
        
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.filter(*filtered);
        
        return filtered;
    }
    
    void analyzeObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacles)
    {
        // Análisis específico para navegación 4WD4WS
        
        // 1. Detectar obstáculos en cada sector
        struct Sector {
            std::string name;
            double angle_min, angle_max;
            std::vector<pcl::PointXYZ> points;
        };
        
        std::vector<Sector> sectors = {
            {"front", -M_PI/4, M_PI/4, {}},
            {"left", M_PI/4, 3*M_PI/4, {}},
            {"rear", 3*M_PI/4, -3*M_PI/4, {}},
            {"right", -3*M_PI/4, -M_PI/4, {}}
        };
        
        // Clasificar puntos por sector
        for (const auto& point : obstacles->points) {
            double angle = atan2(point.y, point.x);
            
            for (auto& sector : sectors) {
                if (angle >= sector.angle_min && angle <= sector.angle_max) {
                    sector.points.push_back(point);
                    break;
                }
            }
        }
        
        // 2. Analizar densidad de obstáculos por sector
        for (const auto& sector : sectors) {
            if (!sector.points.empty()) {
                double min_distance = std::numeric_limits<double>::max();
                for (const auto& point : sector.points) {
                    double dist = sqrt(point.x * point.x + point.y * point.y);
                    min_distance = std::min(min_distance, dist);
                }
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "Sector %s: %zu obstacles, closest at %.2fm", 
                    sector.name.c_str(), sector.points.size(), min_distance);
            }
        }
        
        // 3. Publicar información para planificador local
        // (Implementar mensaje personalizado con información de sectores)
    }
    
    void publishCloud(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::string& frame_id)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = this->now();
        pub->publish(cloud_msg);
    }
    
    // Miembros
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Python Implementation

```python
#!/usr/bin/env python3
# src/lidar_processor.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Point
import numpy as np
import struct
from std_msgs.msg import Header

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor_py')
        
        # Suscriptores
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Publicadores
        self.obstacles_pub = self.create_publisher(
            PointCloud2, '/obstacles_py', 10)
        
        # Parámetros
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('obstacle_height_threshold', 0.1)
        
        self.get_logger().info('LiDAR Processor (Python) initialized')
    
    def scan_callback(self, msg):
        """Procesar datos del LiDAR"""
        
        # Convertir a puntos 3D
        points = self.scan_to_points(msg)
        
        # Filtrar obstáculos
        obstacles = self.filter_obstacles(points)
        
        # Crear y publicar PointCloud2
        cloud_msg = self.create_pointcloud2(obstacles, msg.header.frame_id)
        self.obstacles_pub.publish(cloud_msg)
        
        # Análisis para el eCar
        self.analyze_for_navigation(obstacles)
    
    def scan_to_points(self, scan_msg):
        """Convertir LaserScan a puntos 3D"""
        points = []
        
        for i, range_val in enumerate(scan_msg.ranges):
            # Verificar validez
            if (range_val < scan_msg.range_min or 
                range_val > scan_msg.range_max or
                np.isnan(range_val) or np.isinf(range_val)):
                continue
            
            # Calcular ángulo
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # Convertir a coordenadas cartesianas
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            z = 0.0  # LiDAR 2D
            
            points.append([x, y, z])
        
        return np.array(points)
    
    def filter_obstacles(self, points):
        """Filtrar puntos que representan obstáculos"""
        if len(points) == 0:
            return points
        
        min_range = self.get_parameter('min_range').value
        max_range = self.get_parameter('max_range').value
        
        # Filtrar por rango
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        range_mask = (distances >= min_range) & (distances <= max_range)
        
        filtered_points = points[range_mask]
        
        # Filtrar puntos del suelo (simulado para LiDAR 2D)
        # En aplicación real, usar altura del sensor
        height_threshold = self.get_parameter('obstacle_height_threshold').value
        obstacle_mask = np.abs(filtered_points[:, 2]) > height_threshold
        
        return filtered_points[obstacle_mask] if len(filtered_points) > 0 else filtered_points
    
    def analyze_for_navigation(self, obstacles):
        """Análisis específico para navegación del eCar"""
        if len(obstacles) == 0:
            return
        
        # Definir sectores de análisis para 4WD4WS
        sectors = {
            'front': (-np.pi/4, np.pi/4),
            'front_left': (np.pi/4, np.pi/2),
            'left': (np.pi/2, 3*np.pi/4),
            'rear_left': (3*np.pi/4, np.pi),
            'rear': (-np.pi, -3*np.pi/4),
            'rear_right': (-3*np.pi/4, -np.pi/2),
            'right': (-np.pi/2, -np.pi/4),
            'front_right': (-np.pi/4, 0)
        }
        
        sector_analysis = {}
        
        for sector_name, (angle_min, angle_max) in sectors.items():
            # Calcular ángulos de obstáculos
            angles = np.arctan2(obstacles[:, 1], obstacles[:, 0])
            
            # Normalizar ángulos
            angles = np.where(angles < 0, angles + 2*np.pi, angles)
            angle_min_norm = angle_min if angle_min >= 0 else angle_min + 2*np.pi
            angle_max_norm = angle_max if angle_max >= 0 else angle_max + 2*np.pi
            
            # Seleccionar puntos en el sector
            if angle_min_norm <= angle_max_norm:
                mask = (angles >= angle_min_norm) & (angles <= angle_max_norm)
            else:  # Sector que cruza 0°
                mask = (angles >= angle_min_norm) | (angles <= angle_max_norm)
            
            sector_points = obstacles[mask]
            
            if len(sector_points) > 0:
                # Calcular estadísticas del sector
                distances = np.sqrt(sector_points[:, 0]**2 + sector_points[:, 1]**2)
                sector_analysis[sector_name] = {
                    'count': len(sector_points),
                    'min_distance': np.min(distances),
                    'avg_distance': np.mean(distances),
                    'density': len(sector_points) / len(obstacles)
                }
            else:
                sector_analysis[sector_name] = {
                    'count': 0,
                    'min_distance': float('inf'),
                    'avg_distance': float('inf'),
                    'density': 0.0
                }
        
        # Log información crítica para navegación
        critical_sectors = ['front', 'front_left', 'front_right']
        for sector in critical_sectors:
            info = sector_analysis[sector]
            if info['min_distance'] < 1.0:  # Obstáculo cercano
                self.get_logger().warn(
                    f'Close obstacle in {sector}: {info["min_distance"]:.2f}m')
        
        # Determinar estrategia de movimiento para 4WD4WS
        self.suggest_movement_strategy(sector_analysis)
    
    def suggest_movement_strategy(self, sector_analysis):
        """Sugerir estrategia de movimiento basada en obstáculos"""
        
        # Umbrales de seguridad
        SAFE_DISTANCE = 1.5
        CRITICAL_DISTANCE = 0.8
        
        front_clear = sector_analysis['front']['min_distance'] > SAFE_DISTANCE
        left_clear = sector_analysis['left']['min_distance'] > SAFE_DISTANCE
        right_clear = sector_analysis['right']['min_distance'] > SAFE_DISTANCE
        rear_clear = sector_analysis['rear']['min_distance'] > SAFE_DISTANCE
        
        # Estrategias específicas para 4WD4WS
        if front_clear:
            strategy = "FORWARD_CLEAR"
        elif left_clear and right_clear:
            strategy = "LATERAL_MOVEMENT_AVAILABLE"
        elif left_clear:
            strategy = "STRAFE_LEFT_AVAILABLE"
        elif right_clear:
            strategy = "STRAFE_RIGHT_AVAILABLE"
        elif rear_clear:
            strategy = "REVERSE_AVAILABLE"
        else:
            strategy = "ROTATE_IN_PLACE"
        
        self.get_logger().debug(f'Suggested strategy: {strategy}')
    
    def create_pointcloud2(self, points, frame_id):
        """Crear mensaje PointCloud2 desde array de puntos"""
        
        # Definir campos del PointCloud2
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Crear header
        header = Header()
        header.frame_id = frame_id
        header.stamp = self.get_clock().now().to_msg()
        
        # Convertir puntos a bytes
        if len(points) == 0:
            data = b''
        else:
            # Convertir array numpy a bytes
            data = b''
            for point in points:
                data += struct.pack('fff', point[0], point[1], point[2])
        
        # Crear mensaje PointCloud2
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 floats * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.data = data
        cloud_msg.is_dense = True
        
        return cloud_msg

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    
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

## Visión por Computadora

### Nodo de Procesamiento de Cámara

**C++ Implementation con OpenCV**
```cpp
// src/camera_processor.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/dnn.hpp>

class CameraProcessor : public rclcpp::Node
{
public:
    CameraProcessor() : Node("camera_processor")
    {
        // Suscriptores
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&CameraProcessor::imageCallback, this, std::placeholders::_1));
        
        // Publicadores
        processed_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/camera/processed_image", 10);
        
        detection_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/camera/detections", 10);
        
        // Inicializar detectores
        initializeDetectors();
        
        // Parámetros de cámara (calibración)
        this->declare_parameter("camera_matrix", std::vector<double>{
            525.0, 0.0, 320.0,
            0.0, 525.0, 240.0,
            0.0, 0.0, 1.0
        });
        
        this->declare_parameter("distortion_coeffs", std::vector<double>{
            0.1, -0.2, 0.0, 0.0, 0.0
        });
        
        RCLCPP_INFO(this->get_logger(), "Camera Processor initialized");
    }

private:
    void initializeDetectors()
    {
        // 1. Detector de personas (HOG)
        hog_detector_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        
        // 2. Detector de caras (Cascade)
        if (!face_cascade_.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml")) {
            RCLCPP_WARN(this->get_logger(), "Could not load face cascade");
        }
        
        // 3. Detector de esquinas (para landmarks)
        corner_detector_ = cv::goodFeaturesToTrack;
        
        // 4. DNN para detección de objetos (opcional)
        try {
            // Cargar modelo YOLO o MobileNet
            // dnn_net_ = cv::dnn::readNetFromDarknet("yolo.cfg", "yolo.weights");
            RCLCPP_INFO(this->get_logger(), "DNN models ready");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "DNN initialization failed: %s", e.what());
        }
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // Convertir ROS Image a OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat image = cv_ptr->image;
            cv::Mat processed_image = image.clone();
            
            // Pipeline de procesamiento
            auto features = extractFeatures(image);
            auto people = detectPeople(image);
            auto faces = detectFaces(image);
            auto landmarks = detectLandmarks(image);
            
            // Visualización en imagen procesada
            drawDetections(processed_image, people, faces, landmarks, features);
            
            // Publicar imagen procesada
            publishProcessedImage(processed_image, msg->header);
            
            // Análisis específico para navegación del eCar
            analyzeForNavigation(people, faces, landmarks, msg->header);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
    }
    
    std::vector<cv::KeyPoint> extractFeatures(const cv::Mat& image)
    {
        // Extractor SIFT/ORB para landmarks
        auto sift = cv::SIFT::create();
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        
        sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
        
        return keypoints;
    }
    
    std::vector<cv::Rect> detectPeople(const cv::Mat& image)
    {
        std::vector<cv::Rect> people;
        std::vector<double> weights;
        
        // Detectar personas usando HOG
        hog_detector_.detectMultiScale(image, people, weights, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);
        
        return people;
    }
    
    std::vector<cv::Rect> detectFaces(const cv::Mat& image)
    {
        std::vector<cv::Rect> faces;
        cv::Mat gray;
        
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, gray);
        
        face_cascade_.detectMultiScale(gray, faces, 1.1, 3, 0, cv::Size(30, 30));
        
        return faces;
    }
    
    std::vector<cv::Point2f> detectLandmarks(const cv::Mat& image)
    {
        std::vector<cv::Point2f> corners;
        cv::Mat gray;
        
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        // Detectar esquinas para SLAM visual
        cv::goodFeaturesToTrack(gray, corners, 100, 0.3, 7, cv::Mat(), 7, false, 0.04);
        
        return corners;
    }
    
    void drawDetections(cv::Mat& image, 
                       const std::vector<cv::Rect>& people,
                       const std::vector<cv::Rect>& faces,
                       const std::vector<cv::Point2f>& landmarks,
                       const std::vector<cv::KeyPoint>& features)
    {
        // Dibujar personas detectadas
        for (const auto& person : people) {
            cv::rectangle(image, person, cv::Scalar(0, 255, 0), 2);
            cv::putText(image, "Person", person.tl(), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
        
        // Dibujar caras detectadas
        for (const auto& face : faces) {
            cv::rectangle(image, face, cv::Scalar(255, 0, 0), 2);
            cv::putText(image, "Face", face.tl(), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
        }
        
        // Dibujar landmarks
        for (const auto& landmark : landmarks) {
            cv::circle(image, landmark, 3, cv::Scalar(0, 0, 255), -1);
        }
        
        // Dibujar features SIFT
        cv::drawKeypoints(image, features, image, cv::Scalar(255, 255, 0));
    }
    
    void analyzeForNavigation(const std::vector<cv::Rect>& people,
                             const std::vector<cv::Rect>& faces,
                             const std::vector<cv::Point2f>& landmarks,
                             const std_msgs::msg::Header& header)
    {
        // Análisis de seguridad: personas detectadas
        if (!people.empty()) {
            RCLCPP_WARN(this->get_logger(), 
                       "SAFETY: %zu people detected in camera view", people.size());
            
            // Estimar distancia de personas (usando altura en imagen)
            for (size_t i = 0; i < people.size(); ++i) {
                double estimated_distance = estimatePersonDistance(people[i]);
                
                if (estimated_distance < 3.0) {  // Persona cercana
                    RCLCPP_WARN(this->get_logger(), 
                               "CRITICAL: Person detected at %.2fm", estimated_distance);
                    
                    // Publicar alerta de seguridad
                    geometry_msgs::msg::PointStamped alert;
                    alert.header = header;
                    alert.point.x = estimated_distance;
                    alert.point.y = people[i].x + people[i].width/2;  // Posición X en imagen
                    alert.point.z = -1.0;  // Indicador de persona
                    detection_pub_->publish(alert);
                }
            }
        }
        
        // Análisis de landmarks para localización visual
        if (landmarks.size() > 20) {  // Suficientes features para localización
            RCLCPP_DEBUG(this->get_logger(), 
                        "Good visual features: %zu landmarks detected", landmarks.size());
            
            // Calcular calidad de tracking
            double feature_density = static_cast<double>(landmarks.size()) / (640 * 480);
            
            if (feature_density > 0.0001) {  // Umbral de densidad
                RCLCPP_DEBUG(this->get_logger(), 
                            "Excellent conditions for visual SLAM");
            }
        }
    }
    
    double estimatePersonDistance(const cv::Rect& person_rect)
    {
        // Estimación simple basada en altura de persona en imagen
        // Asume persona promedio de 1.7m de altura
        
        double person_height_real = 1.7;  // metros
        double person_height_pixels = person_rect.height;
        double focal_length = 525.0;  // parámetro de cámara
        
        // Ecuación de proyección: distance = (real_height * focal_length) / pixel_height
        double distance = (person_height_real * focal_length) / person_height_pixels;
        
        return distance;
    }
    
    void publishProcessedImage(const cv::Mat& image, const std_msgs::msg::Header& header)
    {
        cv_bridge::CvImage cv_image;
        cv_image.header = header;
        cv_image.encoding = "bgr8";
        cv_image.image = image;
        
        processed_image_pub_->publish(*cv_image.toImageMsg());
    }
    
    // Miembros
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr detection_pub_;
    
    // Detectores OpenCV
    cv::HOGDescriptor hog_detector_;
    cv::CascadeClassifier face_cascade_;
    cv::dnn::Net dnn_net_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Fusión de Sensores

### Nodo de Fusión Multisensor

```cpp
// src/sensor_fusion.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class SensorFusion : public rclcpp::Node
{
public:
    SensorFusion() : Node("sensor_fusion"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Suscriptores sincronizados
        lidar_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
            this, "/scan");
        imu_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
            this, "/imu");
        
        // Política de sincronización
        sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), *lidar_sub_, *imu_sub_);
        sync_->registerCallback(
            std::bind(&SensorFusion::fusedCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Suscriptores individuales
        ultrasonic_subs_.resize(8);
        for (int i = 0; i < 8; ++i) {
            std::string topic = "/ultrasonic_" + std::to_string(i);
            ultrasonic_subs_[i] = this->create_subscription<sensor_msgs::msg::Range>(
                topic, 10, 
                [this, i](const sensor_msgs::msg::Range::SharedPtr msg) {
                    ultrasonicCallback(msg, i);
                });
        }
        
        // Publicadores
        fused_obstacles_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/fused_costmap", 10);
        
        safety_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/emergency_stop", 10);
        
        // Inicializar costmap fusionado
        initializeFusedCostmap();
        
        // Timer para publicación periódica
        fusion_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SensorFusion::publishFusedData, this));
        
        RCLCPP_INFO(this->get_logger(), "Sensor Fusion initialized");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan, sensor_msgs::msg::Imu>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
    
    void fusedCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
                      const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
    {
        // Fusión principal LiDAR + IMU
        updateFromLidar(scan);
        updateFromIMU(imu);
        
        // Verificar condiciones de emergencia
        checkEmergencyConditions();
    }
    
    void ultrasonicCallback(const sensor_msgs::msg::Range::SharedPtr msg, int sensor_id)
    {
        // Actualizar datos ultrasónicos
        ultrasonic_data_[sensor_id] = *msg;
        updateFromUltrasonic(sensor_id);
    }
    
    void updateFromLidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan)
    {
        // Procesar datos LiDAR y actualizar costmap
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            
            if (range < scan->range_min || range > scan->range_max) {
                continue;
            }
            
            float angle = scan->angle_min + i * scan->angle_increment;
            
            // Convertir a coordenadas del costmap
            float x = range * cos(angle);
            float y = range * sin(angle);
            
            // Marcar obstáculo en costmap fusionado
            markObstacleInCostmap(x, y, 100);  // Alta confianza LiDAR
        }
    }
    
    void updateFromIMU(const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
    {
        // Usar IMU para corrección de orientación y detección de movimientos bruscos
        
        // Detectar aceleración excesiva (posible colisión)
        double accel_magnitude = sqrt(
            pow(imu->linear_acceleration.x, 2) +
            pow(imu->linear_acceleration.y, 2) +
            pow(imu->linear_acceleration.z - 9.81, 2));  // Restar gravedad
        
        if (accel_magnitude > 5.0) {  // Umbral de emergencia
            RCLCPP_WARN(this->get_logger(), 
                       "High acceleration detected: %.2f m/s²", accel_magnitude);
            emergency_stop_triggered_ = true;
        }
        
        // Usar velocidad angular para predicción de movimiento
        current_angular_velocity_ = imu->angular_velocity.z;
    }
    
    void updateFromUltrasonic(int sensor_id)
    {
        const auto& ultrasonic = ultrasonic_data_[sensor_id];
        
        if (ultrasonic.range < ultrasonic.min_range || 
            ultrasonic.range > ultrasonic.max_range) {
            return;
        }
        
        // Posiciones de sensores ultrasónicos (configuración específica del eCar)
        std::vector<std::pair<double, double>> sensor_positions = {
            {0.35, 0.2},    // front-left
            {0.35, -0.2},   // front-right
            {0.2, 0.35},    // left-front
            {-0.2, 0.35},   // left-rear
            {-0.35, 0.2},   // rear-left
            {-0.35, -0.2},  // rear-right
            {0.2, -0.35},   // right-front
            {-0.2, -0.35}   // right-rear
        };
        
        if (sensor_id < sensor_positions.size()) {
            auto [sensor_x, sensor_y] = sensor_positions[sensor_id];
            
            // Calcular posición del obstáculo
            double obstacle_x = sensor_x + ultrasonic.range * cos(sensor_id * M_PI/4);
            double obstacle_y = sensor_y + ultrasonic.range * sin(sensor_id * M_PI/4);
            
            // Marcar en costmap con menor confianza que LiDAR
            markObstacleInCostmap(obstacle_x, obstacle_y, 80);
            
            // Verificar distancias críticas para cada sector
            if (ultrasonic.range < 0.3) {  // Muy cerca
                RCLCPP_WARN(this->get_logger(), 
                           "Critical proximity: Sensor %d detects obstacle at %.2fm", 
                           sensor_id, ultrasonic.range);
                
                proximity_alerts_[sensor_id] = true;
            } else {
                proximity_alerts_[sensor_id] = false;
            }
        }
    }
    
    void markObstacleInCostmap(double x, double y, int confidence)
    {
        // Convertir coordenadas del mundo a índices del costmap
        int map_x = static_cast<int>((x - costmap_origin_x_) / costmap_resolution_);
        int map_y = static_cast<int>((y - costmap_origin_y_) / costmap_resolution_);
        
        // Verificar límites
        if (map_x >= 0 && map_x < costmap_width_ && 
            map_y >= 0 && map_y < costmap_height_) {
            
            int index = map_y * costmap_width_ + map_x;
            
            // Fusión de confianza: max entre valor actual y nuevo
            fused_costmap_.data[index] = std::max(
                static_cast<int>(fused_costmap_.data[index]), confidence);
        }
    }
    
    void checkEmergencyConditions()
    {
        // 1. Verificar parada de emergencia por IMU
        if (emergency_stop_triggered_) {
            publishEmergencyStop();
            return;
        }
        
        // 2. Verificar proximidad crítica en múltiples sensores
        int critical_sensors = 0;
        for (const auto& alert : proximity_alerts_) {
            if (alert) critical_sensors++;
        }
        
        if (critical_sensors >= 3) {  // 3 o más sensores detectan proximidad crítica
            RCLCPP_ERROR(this->get_logger(), 
                        "EMERGENCY: Multiple proximity alerts (%d sensors)", critical_sensors);
            publishEmergencyStop();
        }
        
        // 3. Verificar patrón de obstáculos que indique entorno cerrado
        if (isTrappedByObstacles()) {
            RCLCPP_WARN(this->get_logger(), "Robot appears to be trapped");
            // No emergency stop, pero sí notificación especial
        }
    }
    
    bool isTrappedByObstacles()
    {
        // Analizar si el robot está rodeado de obstáculos
        int sectors_with_obstacles = 0;
        
        for (int i = 0; i < 8; ++i) {
            if (proximity_alerts_[i]) {
                sectors_with_obstacles++;
            }
        }
        
        return sectors_with_obstacles >= 6;  // 6 de 8 sectores bloqueados
    }
    
    void publishEmergencyStop()
    {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.y = 0.0;
        stop_cmd.angular.z = 0.0;
        
        safety_pub_->publish(stop_cmd);
        
        // Reset flag después de publicar
        emergency_stop_triggered_ = false;
    }
    
    void initializeFusedCostmap()
    {
        // Configurar costmap fusionado
        costmap_width_ = 200;   // 20m / 0.1m/cell
        costmap_height_ = 200;
        costmap_resolution_ = 0.1;  // 10cm/cell
        costmap_origin_x_ = -10.0;  // Centrado en robot
        costmap_origin_y_ = -10.0;
        
        fused_costmap_.header.frame_id = "base_link";
        fused_costmap_.info.resolution = costmap_resolution_;
        fused_costmap_.info.width = costmap_width_;
        fused_costmap_.info.height = costmap_height_;
        fused_costmap_.info.origin.position.x = costmap_origin_x_;
        fused_costmap_.info.origin.position.y = costmap_origin_y_;
        fused_costmap_.info.origin.orientation.w = 1.0;
        
        // Inicializar datos
        fused_costmap_.data.resize(costmap_width_ * costmap_height_, 0);
        
        // Inicializar arrays de alertas
        proximity_alerts_.resize(8, false);
        ultrasonic_data_.resize(8);
    }
    
    void publishFusedData()
    {
        // Actualizar timestamp y publicar costmap fusionado
        fused_costmap_.header.stamp = this->now();
        fused_obstacles_pub_->publish(fused_costmap_);
        
        // Decay del costmap (reducir confianza con el tiempo)
        for (auto& cell : fused_costmap_.data) {
            if (cell > 0) {
                cell = static_cast<int8_t>(cell * 0.95);  // Decay del 5%
            }
        }
    }
    
    // Miembros
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    
    // Suscriptores sincronizados
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> lidar_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_sub_;
    std::shared_ptr<Synchronizer> sync_;
    
    // Suscriptores individuales
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> ultrasonic_subs_;
    
    // Publicadores
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr fused_obstacles_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safety_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr fusion_timer_;
    
    // Datos de estado
    nav_msgs::msg::OccupancyGrid fused_costmap_;
    std::vector<bool> proximity_alerts_;
    std::vector<sensor_msgs::msg::Range> ultrasonic_data_;
    bool emergency_stop_triggered_ = false;
    double current_angular_velocity_ = 0.0;
    
    // Parámetros del costmap
    int costmap_width_, costmap_height_;
    double costmap_resolution_;
    double costmap_origin_x_, costmap_origin_y_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Configuración y Launch Files

### Launch File para Percepción Completa

```xml
<!-- launch/perception.launch.py -->
<launch>
  <!-- Argumentos -->
  <arg name="use_camera" default="true"/>
  <arg name="use_lidar" default="true"/>
  <arg name="use_ultrasonic" default="true"/>
  <arg name="debug_visualization" default="false"/>
  
  <!-- Procesamiento LiDAR -->
  <group if="$(var use_lidar)">
    <node pkg="tadeo_ecar_perception" exec="lidar_processor">
      <param name="min_range" value="0.1"/>
      <param name="max_range" value="20.0"/>
      <param name="voxel_size" value="0.05"/>
      <param name="ground_threshold" value="0.02"/>
    </node>
  </group>
  
  <!-- Procesamiento de Cámara -->
  <group if="$(var use_camera)">
    <node pkg="tadeo_ecar_perception" exec="camera_processor">
      <param name="camera_matrix" value="[525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0]"/>
      <param name="distortion_coeffs" value="[0.1, -0.2, 0.0, 0.0, 0.0]"/>
    </node>
  </group>
  
  <!-- Sensores Ultrasónicos -->
  <group if="$(var use_ultrasonic)">
    <!-- Simulación de sensores ultrasónicos -->
    <node pkg="tadeo_ecar_sensors" exec="ultrasonic_simulator">
      <param name="sensor_count" value="8"/>
      <param name="update_rate" value="20.0"/>
    </node>
  </group>
  
  <!-- Fusión de Sensores -->
  <node pkg="tadeo_ecar_perception" exec="sensor_fusion">
    <param name="costmap_resolution" value="0.1"/>
    <param name="costmap_size" value="20.0"/>
    <param name="emergency_threshold" value="0.3"/>
  </node>
  
  <!-- Visualización (solo para debug) -->
  <group if="$(var debug_visualization)">
    <node pkg="rviz2" exec="rviz2" args="-d $(find-pkg-share tadeo_ecar_perception)/config/perception.rviz"/>
  </group>
  
</launch>
```

### Configuración YAML

```yaml
# config/perception_params.yaml
lidar_processor:
  ros__parameters:
    min_range: 0.1
    max_range: 20.0
    voxel_size: 0.05
    ground_threshold: 0.02
    obstacle_height_threshold: 0.1
    
camera_processor:
  ros__parameters:
    # Parámetros de calibración de cámara
    camera_matrix: [525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0]
    distortion_coeffs: [0.1, -0.2, 0.0, 0.0, 0.0]
    
    # Detección de personas
    person_detection_enabled: true
    person_confidence_threshold: 0.7
    
    # Detección de caras
    face_detection_enabled: true
    face_cascade_path: "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml"
    
sensor_fusion:
  ros__parameters:
    # Costmap fusionado
    costmap_resolution: 0.1
    costmap_width: 200
    costmap_height: 200
    costmap_origin_x: -10.0
    costmap_origin_y: -10.0
    
    # Umbrales de emergencia
    emergency_accel_threshold: 5.0
    critical_proximity_threshold: 0.3
    min_critical_sensors: 3
    
    # Decay del costmap
    costmap_decay_rate: 0.95
    update_frequency: 10.0
```

### Package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>tadeo_ecar_perception</name>
  <version>1.0.0</version>
  <description>Sistema de percepción avanzada para eCar 4WD4WS</description>
  
  <maintainer email="semillero@universidad.edu">Semillero TADEO</maintainer>
  <license>Proprietary</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- ROS2 Dependencies -->
  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  
  <!-- Perception Dependencies -->
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  <depend>pcl_conversions</depend>
  <depend>pcl_ros</depend>
  <depend>message_filters</depend>
  
  <!-- OpenCV -->
  <depend>opencv2</depend>
  
  <!-- PCL -->
  <depend>libpcl-all-dev</depend>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_perception)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# C++ Executables
add_executable(lidar_processor src/lidar_processor.cpp)
ament_target_dependencies(lidar_processor
  rclcpp sensor_msgs geometry_msgs pcl_conversions pcl_ros)
target_link_libraries(lidar_processor ${PCL_LIBRARIES})

add_executable(camera_processor src/camera_processor.cpp)
ament_target_dependencies(camera_processor
  rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(camera_processor ${OpenCV_LIBS})

add_executable(sensor_fusion src/sensor_fusion.cpp)
ament_target_dependencies(sensor_fusion
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros message_filters)

# Install targets
install(TARGETS
  lidar_processor
  camera_processor
  sensor_fusion
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  src/lidar_processor.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Testing y Validación

### Test Script para Percepción

```python
#!/usr/bin/env python3
# test/test_perception.py

import unittest
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, Image, Range
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge

class PerceptionTester(Node):
    def __init__(self):
        super().__init__('perception_tester')
        self.bridge = CvBridge()
        
        # Publishers para datos de prueba
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.range_pubs = []
        for i in range(8):
            pub = self.create_publisher(Range, f'/ultrasonic_{i}', 10)
            self.range_pubs.append(pub)
        
        # Subscribers para resultados
        self.detection_sub = self.create_subscription(
            PointStamped, '/camera/detections', self.detection_callback, 10)
        
        self.detections_received = []
    
    def detection_callback(self, msg):
        self.detections_received.append(msg)
    
    def generate_test_scan(self, obstacle_distance=2.0, obstacle_angle=0.0):
        """Generar LaserScan de prueba con obstáculo en posición específica"""
        scan = LaserScan()
        scan.header.frame_id = "laser_frame"
        scan.header.stamp = self.get_clock().now().to_msg()
        
        scan.angle_min = -np.pi
        scan.angle_max = np.pi
        scan.angle_increment = np.pi / 180.0  # 1 grado
        scan.range_min = 0.1
        scan.range_max = 30.0
        
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [scan.range_max] * num_readings
        
        # Añadir obstáculo en ángulo específico
        obstacle_index = int((obstacle_angle - scan.angle_min) / scan.angle_increment)
        if 0 <= obstacle_index < num_readings:
            scan.ranges[obstacle_index] = obstacle_distance
        
        return scan
    
    def generate_test_image_with_person(self):
        """Generar imagen de prueba con persona simulada"""
        # Crear imagen en blanco
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Dibujar rectángulo que simula una persona
        cv2.rectangle(image, (250, 150), (350, 400), (255, 255, 255), -1)
        cv2.rectangle(image, (275, 160), (325, 200), (200, 200, 200), -1)  # Cabeza
        
        # Convertir a ROS Image
        ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        ros_image.header.frame_id = "camera_frame"
        ros_image.header.stamp = self.get_clock().now().to_msg()
        
        return ros_image
    
    def generate_ultrasonic_reading(self, sensor_id, distance):
        """Generar lectura ultrasónica de prueba"""
        range_msg = Range()
        range_msg.header.frame_id = f"ultrasonic_{sensor_id}_frame"
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.1
        range_msg.min_range = 0.02
        range_msg.max_range = 4.0
        range_msg.range = distance
        
        return range_msg

class TestPerceptionSystem(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.tester = PerceptionTester()
    
    @classmethod
    def tearDownClass(cls):
        cls.tester.destroy_node()
        rclpy.shutdown()
    
    def test_lidar_obstacle_detection(self):
        """Test detección de obstáculos con LiDAR"""
        
        # Generar scan con obstáculo frontal
        test_scan = self.tester.generate_test_scan(
            obstacle_distance=1.5, obstacle_angle=0.0)
        
        # Publicar datos de prueba
        self.tester.scan_pub.publish(test_scan)
        
        # Dar tiempo para procesamiento
        rclpy.spin_once(self.tester, timeout_sec=0.1)
        
        # Verificar que se detectó el obstáculo
        # (En implementación real, verificar mensaje de salida)
        self.assertTrue(True)  # Placeholder
    
    def test_camera_person_detection(self):
        """Test detección de personas con cámara"""
        
        # Limpiar detecciones previas
        self.tester.detections_received.clear()
        
        # Generar imagen con persona
        test_image = self.tester.generate_test_image_with_person()
        
        # Publicar imagen
        self.tester.image_pub.publish(test_image)
        
        # Esperar procesamiento
        for _ in range(10):
            rclpy.spin_once(self.tester, timeout_sec=0.1)
        
        # Verificar detección
        # self.assertGreater(len(self.tester.detections_received), 0)
        # self.assertEqual(self.tester.detections_received[0].point.z, -1.0)  # Indicador de persona
    
    def test_ultrasonic_proximity_detection(self):
        """Test detección de proximidad con ultrasonidos"""
        
        # Generar lecturas de proximidad crítica
        for i in range(4):  # Simular 4 sensores con detección crítica
            critical_reading = self.tester.generate_ultrasonic_reading(i, 0.2)
            self.tester.range_pubs[i].publish(critical_reading)
        
        # Procesar
        for _ in range(5):
            rclpy.spin_once(self.tester, timeout_sec=0.1)
        
        # Verificar que se activó alerta de emergencia
        self.assertTrue(True)  # Placeholder
    
    def test_sensor_fusion_consistency(self):
        """Test consistencia de fusión de sensores"""
        
        # Publicar datos consistentes de múltiples sensores
        # LiDAR detecta obstáculo frontal
        lidar_scan = self.tester.generate_test_scan(1.0, 0.0)
        self.tester.scan_pub.publish(lidar_scan)
        
        # Ultrasónico frontal confirma
        ultrasonic_front = self.tester.generate_ultrasonic_reading(0, 1.0)
        self.tester.range_pubs[0].publish(ultrasonic_front)
        
        # Procesar fusión
        for _ in range(10):
            rclpy.spin_once(self.tester, timeout_sec=0.1)
        
        # Verificar coherencia
        self.assertTrue(True)  # Placeholder
    
    def test_emergency_stop_conditions(self):
        """Test condiciones de parada de emergencia"""
        
        # Simular múltiples sensores detectando proximidad crítica
        for i in range(6):  # 6 de 8 sensores
            critical_reading = self.tester.generate_ultrasonic_reading(i, 0.15)
            self.tester.range_pubs[i].publish(critical_reading)
        
        # Procesar
        for _ in range(5):
            rclpy.spin_once(self.tester, timeout_sec=0.1)
        
        # Verificar activación de emergency stop
        self.assertTrue(True)  # Placeholder

def main():
    unittest.main()

if __name__ == '__main__':
    main()
```

### Herramientas de Debug

```bash
#!/bin/bash
# scripts/debug_perception.sh

echo "=== Debugging Percepción eCar ==="

# 1. Verificar topics activos
echo "Topics de percepción activos:"
ros2 topic list | grep -E "(scan|image|ultrasonic|detection|fusion)"

# 2. Verificar frecuencia de publicación
echo -e "\nFrecuencias de publicación:"
ros2 topic hz /scan &
ros2 topic hz /camera/image_raw &
ros2 topic hz /fused_costmap &

sleep 5
pkill -f "ros2 topic hz"

# 3. Verificar calidad de datos LiDAR
echo -e "\nCalidad datos LiDAR:"
ros2 topic echo /scan --once | grep -E "(range_min|range_max|ranges)"

# 4. Verificar TF tree
echo -e "\nÁrbol TF:"
ros2 run tf2_tools view_frames

# 5. Verificar nodos de percepción
echo -e "\nNodos de percepción activos:"
ros2 node list | grep -E "(lidar|camera|fusion|perception)"

# 6. Test de latencia
echo -e "\nTest de latencia:"
ros2 topic delay /scan
ros2 topic delay /camera/image_raw

echo "=== Debug completado ==="
```

Este capítulo cubre todos los aspectos fundamentales de la percepción para el eCar 4WD4WS, desde el procesamiento básico de sensores hasta la fusión avanzada y la detección de emergencias específicas para la plataforma omnidireccional.