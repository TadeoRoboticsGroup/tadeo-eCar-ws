# Capítulo 11: SLAM en ROS2

## Tabla de Contenidos

1. [Introducción al SLAM](#introducción-al-slam)
2. [Algoritmos de SLAM](#algoritmos-de-slam)
3. [SLAM Toolbox](#slam-toolbox)
4. [Cartographer](#cartographer)
5. [Configuración para eCar](#configuración-para-ecar)
6. [Mapeo en Tiempo Real](#mapeo-en-tiempo-real)
7. [Localización y Re-localización](#localización-y-re-localización)
8. [Optimización y Tuning](#optimización-y-tuning)

## Introducción al SLAM

### ¿Qué es SLAM?

SLAM (Simultaneous Localization and Mapping) es el proceso de construir un mapa del entorno mientras simultáneamente se determina la ubicación del robot dentro de ese mapa.

```
SLAM = Localización + Mapeo (Simultáneamente)

Entrada: Datos de sensores (LiDAR, IMU, odometría)
Salida: Mapa del entorno + Pose del robot

Problema: Sin mapa no hay localización, sin localización no hay mapa
Solución: Resolver ambos problemas simultáneamente
```

### El Problema del Huevo y la Gallina

```
Sin SLAM:
┌─ Localización ─┐    ┌─ Mapeo ─┐
│ Necesita mapa  │ ←→ │ Necesita │
│ para funcionar │    │ posición │
└────────────────┘    └──────────┘

Con SLAM:
┌─ Localización ─┐    ┌─ Mapeo ─┐
│ Estima posición│ ←→ │ Construye│
│ usando mapa    │    │ mapa     │
└─ parcial      ─┘    └─ gradual ─┘
```

### Ventajas de SLAM para el eCar

**1. Autonomía Completa**
- No requiere mapas pre-existentes
- Puede operar en entornos desconocidos
- Adaptable a cambios en el entorno

**2. Precisión Mejorada**
- Corrige errores de odometría
- Usa características del entorno
- Mantiene consistencia global

**3. Mapeo Específico del Dominio**
- Mapas optimizados para tareas específicas
- Información semántica del entorno
- Actualizaciones en tiempo real

### Tipos de SLAM

**SLAM 2D (para eCar)**
```bash
# Usa LiDAR 2D para crear mapas de ocupación
# Ideal para navegación indoor/outdoor plana
# Menor costo computacional
```

**SLAM 3D**
```bash
# Usa LiDAR 3D o cámaras estéreo
# Mapas volumétricos completos
# Mayor costo computacional
```

**Visual SLAM**
```bash
# Usa cámaras para SLAM
# Información rica en textura
# Problemas con iluminación variable
```

## Algoritmos de SLAM

### Clasificación de Algoritmos SLAM

**1. Basados en Filtros**
```
Extended Kalman Filter (EKF-SLAM)
Particle Filter (FastSLAM)
Información Filter

Ventajas: Tiempo real, menor memoria
Desventajas: Linealización, drift acumulativo
```

**2. Basados en Grafos**
```
Graph-based SLAM (g2o, GTSAM)
Pose Graph SLAM
Bundle Adjustment

Ventajas: Optimización global, precisión
Desventajas: Mayor costo computacional
```

**3. Algoritmos Híbridos**
```
SLAM Toolbox (ROS2)
Cartographer (Google)
ORB-SLAM (Visual)

Ventajas: Combinan lo mejor de ambos mundos
```

### Comparación para eCar 4WD4WS

| Algoritmo | Tiempo Real | Precisión | CPU | Memoria | eCar Compatibility |
|-----------|-------------|-----------|-----|---------|-------------------|
| SLAM Toolbox | ✓✓✓ | ✓✓✓ | ✓✓ | ✓✓ | ✓✓✓ |
| Cartographer | ✓✓ | ✓✓✓ | ✓ | ✓ | ✓✓ |
| Hector SLAM | ✓✓✓ | ✓✓ | ✓✓✓ | ✓✓✓ | ✓✓ |
| GMapping | ✓✓ | ✓✓ | ✓✓ | ✓✓ | ✓ |

## SLAM Toolbox

### Introducción a SLAM Toolbox

SLAM Toolbox es el algoritmo SLAM oficial de ROS2, desarrollado por Steve Macenski. Es ideal para robots como el eCar por su equilibrio entre precisión y eficiencia.

### Características Principales

**1. Modos de Operación**
```yaml
# Online SLAM - Tiempo real
mapping_mode: "mapping"

# Offline SLAM - Post-procesamiento
mapping_mode: "localization"

# Lifelong SLAM - Mapeo continuo
mapping_mode: "lifelong"
```

**2. Loop Closure Detection**
```cpp
// Detección automática de bucles cerrados
// Corrige errores acumulativos
// Mejora consistencia global del mapa
```

**3. Serialización de Mapas**
```bash
# Guardar mapas en formato serializado
# Cargar mapas para localización
# Continuar mapeo desde mapas existentes
```

### Configuración Básica para eCar

```yaml
# config/slam_toolbox_params.yaml
slam_toolbox:
  ros__parameters:
    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping  # mapping, localization, lifelong
    
    # General Parameters
    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 20.0  # Ajustado para eCar LiDAR
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    
    # Correlation Parameters
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1  
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    
    # Correlation Parameters - Fine
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Loop Closure Parameters  
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03
    
    # Scan Matcher Parameters
    distance_variance_penalty: 0.5      
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

### Implementación del Nodo SLAM

```cpp
// include/tadeo_ecar_slam/slam_node.hpp
#ifndef TADEO_ECAR_SLAM__SLAM_NODE_HPP_
#define TADEO_ECAR_SLAM__SLAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/string.hpp>

namespace tadeo_ecar_slam
{
class SlamNode : public rclcpp::Node
{
public:
  SlamNode();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  
  void publishSlamStatus();
  void savemap();
  void loadMap(const std::string& map_file);
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slam_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  
  // Subscribers  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  
  // TF2
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Timer
  rclcpp::TimerInterface::SharedPtr status_timer_;
  
  // State
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
  
  // Parameters
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool save_map_on_shutdown_;
  std::string map_save_directory_;
};
}  // namespace tadeo_ecar_slam

#endif  // TADEO_ECAR_SLAM__SLAM_NODE_HPP_
```

```cpp
// src/slam_node.cpp
#include "tadeo_ecar_slam/slam_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <filesystem>

namespace tadeo_ecar_slam
{
SlamNode::SlamNode() : Node("slam_node")
{
  // Declare parameters
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("save_map_on_shutdown", true);
  this->declare_parameter("map_save_directory", std::string(getenv("HOME")) + "/ecar_maps");
  
  // Get parameters
  map_frame_ = this->get_parameter("map_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  save_map_on_shutdown_ = this->get_parameter("save_map_on_shutdown").as_bool();
  map_save_directory_ = this->get_parameter("map_save_directory").as_string();
  
  // Create map save directory
  std::filesystem::create_directories(map_save_directory_);
  
  // TF2 setup
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Publishers
  slam_status_pub_ = this->create_publisher<std_msgs::msg::String>("slam_status", 10);
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_raw", 10);
  
  // Subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&SlamNode::scanCallback, this, std::placeholders::_1));
  
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", 10,
    std::bind(&SlamNode::mapCallback, this, std::placeholders::_1));
  
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose", 10,
    std::bind(&SlamNode::poseCallback, this, std::placeholders::_1));
  
  // Timer for status publishing
  status_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SlamNode::publishSlamStatus, this));
  
  RCLCPP_INFO(this->get_logger(), "eCar SLAM Node initialized");
  RCLCPP_INFO(this->get_logger(), "Map save directory: %s", map_save_directory_.c_str());
}

void SlamNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Process laser scan for SLAM
  // In a real implementation, this would interface with SLAM Toolbox
  
  RCLCPP_DEBUG(this->get_logger(), 
               "Received laser scan with %zu points", msg->ranges.size());
  
  // Example: Basic scan quality check for eCar
  size_t valid_points = 0;
  for (const auto& range : msg->ranges) {
    if (range >= msg->range_min && range <= msg->range_max && 
        !std::isnan(range) && !std::isinf(range)) {
      valid_points++;
    }
  }
  
  double scan_quality = static_cast<double>(valid_points) / msg->ranges.size();
  
  if (scan_quality < 0.5) {
    RCLCPP_WARN(this->get_logger(), 
                "Low scan quality: %.1f%% valid points", scan_quality * 100);
  }
}

void SlamNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  
  // Republish for monitoring
  map_pub_->publish(*msg);
  
  RCLCPP_DEBUG(this->get_logger(), 
               "Received map update: %dx%d, resolution=%.3f",
               msg->info.width, msg->info.height, msg->info.resolution);
}

void SlamNode::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg;
  
  RCLCPP_DEBUG(this->get_logger(),
               "Received pose update: (%.2f, %.2f)",
               msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void SlamNode::publishSlamStatus()
{
  auto status_msg = std_msgs::msg::String();
  
  // Create status report
  std::string status = "SLAM Status: ";
  
  if (current_map_ && current_pose_) {
    // Calculate map metrics
    int occupied_cells = 0;
    int free_cells = 0;
    int unknown_cells = 0;
    
    for (const auto& cell : current_map_->data) {
      if (cell == 100) occupied_cells++;
      else if (cell == 0) free_cells++;
      else unknown_cells++;
    }
    
    double map_coverage = static_cast<double>(free_cells + occupied_cells) / 
                         current_map_->data.size();
    
    status += "ACTIVE, ";
    status += "Map: " + std::to_string(current_map_->info.width) + "x" + 
              std::to_string(current_map_->info.height) + ", ";
    status += "Coverage: " + std::to_string(static_cast<int>(map_coverage * 100)) + "%, ";
    status += "Pose: (" + std::to_string(current_pose_->pose.pose.position.x) + ", " +
              std::to_string(current_pose_->pose.pose.position.y) + ")";
  } else {
    status += "INITIALIZING";
  }
  
  status_msg.data = status;
  slam_status_pub_->publish(status_msg);
}

void SlamNode::savemap()
{
  if (!current_map_) {
    RCLCPP_WARN(this->get_logger(), "No map available to save");
    return;
  }
  
  // Generate timestamp for unique filename
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto tm = *std::localtime(&time_t);
  
  char timestamp[100];
  std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm);
  
  std::string map_filename = map_save_directory_ + "/ecar_map_" + timestamp;
  
  // In a real implementation, you would call map_saver service
  RCLCPP_INFO(this->get_logger(), "Saving map to: %s", map_filename.c_str());
  
  // Example map save command (would be done via service call)
  std::string cmd = "ros2 run nav2_map_server map_saver_cli -f " + map_filename;
  system(cmd.c_str());
}

void SlamNode::loadMap(const std::string& map_file)
{
  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_file.c_str());
  
  // In a real implementation, you would use map_server to load the map
  // This is a placeholder for the integration
}
}  // namespace tadeo_ecar_slam

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tadeo_ecar_slam::SlamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### Launch File para SLAM Toolbox

```python
# launch/slam_toolbox.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get directories
    pkg_share = FindPackageShare('tadeo_ecar_slam')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map_file')
    mode = LaunchConfiguration('mode')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_params.yaml']),
        description='Full path to the ROS2 parameters file for SLAM Toolbox')
    
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Map file for localization mode')
    
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        choices=['mapping', 'localization', 'lifelong'],
        description='SLAM mode: mapping, localization, or lifelong')
    
    # SLAM Toolbox node (mapping mode)
    slam_toolbox_mapping_node = Node(
        condition=IfCondition(LaunchConfiguration('mode', 'mapping')),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # SLAM Toolbox node (localization mode)
    slam_toolbox_localization_node = Node(
        condition=IfCondition(LaunchConfiguration('mode', 'localization')),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time,
             'map_file_name': map_file}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # SLAM Toolbox node (lifelong mode)
    slam_toolbox_lifelong_node = Node(
        condition=IfCondition(LaunchConfiguration('mode', 'lifelong')),
        package='slam_toolbox',
        executable='lifelong_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )
    
    # eCar SLAM monitoring node
    ecar_slam_node = Node(
        package='tadeo_ecar_slam',
        executable='slam_node',
        name='ecar_slam_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'map_frame': 'map',
             'odom_frame': 'odom',
             'base_frame': 'base_link'}
        ]
    )
    
    # Map server (for localization mode)
    map_server_node = Node(
        condition=IfCondition(LaunchConfiguration('mode', 'localization')),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'yaml_filename': map_file}
        ]
    )
    
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        declare_map_file_cmd,
        declare_mode_cmd,
        
        # Log info
        LogInfo(msg=['Launching SLAM in ', mode, ' mode']),
        
        # Nodes
        slam_toolbox_mapping_node,
        slam_toolbox_localization_node,
        slam_toolbox_lifelong_node,
        ecar_slam_node,
        map_server_node
    ])
```

## Cartographer

### Introducción a Cartographer

Cartographer es el algoritmo SLAM desarrollado por Google, conocido por su alta precisión en mapeo 2D y 3D. Aunque es más pesado computacionalmente, ofrece excelente calidad de mapas.

### Configuración para eCar

```lua
-- config/ecar_2d.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- Configuración específica para eCar 4WD4WS
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- Trajectory Builder 2D para eCar
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 20.  -- Rango del LiDAR del eCar
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 40.

-- Ceres Scan Matcher para mayor precisión
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 2

-- Motion Filter para eCar (más sensible debido a 4WS)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- Pose Graph Optimization
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
```

### Launch File para Cartographer

```python
# launch/cartographer.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Directories
    cartographer_config_dir = PathJoinSubstitution([
        FindPackageShare('tadeo_ecar_slam'), 'config'
    ])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_basename = LaunchConfiguration('configuration_basename')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation time')
    
    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='ecar_2d.lua',
        description='Cartographer configuration file')
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('echoes', 'horizontal_laser_2d'),
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/imu', '/imu/data')
        ]
    )
    
    # Cartographer occupancy grid node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_configuration_basename_cmd,
        cartographer_node,
        cartographer_occupancy_grid_node
    ])
```

## Configuración para eCar

### Parámetros Específicos del eCar

```yaml
# config/ecar_slam_config.yaml
ecar_slam:
  ros__parameters:
    # Robot-specific parameters
    robot_type: "4wd4ws"
    wheel_base: 1.2
    track_width: 0.8
    max_linear_velocity: 2.0
    max_angular_velocity: 1.0
    
    # SLAM-specific parameters
    slam_algorithm: "slam_toolbox"  # slam_toolbox, cartographer, hector
    update_rate: 20.0
    map_resolution: 0.05
    map_size: 2048  # 2048x2048 cells = ~100x100m at 0.05m resolution
    
    # Sensor configuration
    lidar_topic: "/scan"
    imu_topic: "/imu/data"
    odom_topic: "/odom"
    
    # Quality parameters
    min_scan_range: 0.3
    max_scan_range: 20.0
    min_scan_quality: 0.5  # Minimum percentage of valid scan points
    
    # Performance parameters
    cpu_limit_percent: 70.0
    memory_limit_mb: 2048
    map_update_interval: 1.0
    
    # File management
    auto_save_map: true
    map_save_interval: 300.0  # Save every 5 minutes
    map_directory: "~/ecar_maps"
    map_prefix: "ecar_slam"
```

### Integración con Sistema de Control

```cpp
// include/tadeo_ecar_slam/slam_control_interface.hpp
#ifndef TADEO_ECAR_SLAM__SLAM_CONTROL_INTERFACE_HPP_
#define TADEO_ECAR_SLAM__SLAM_CONTROL_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace tadeo_ecar_slam
{
class SlamControlInterface : public rclcpp::Node
{
public:
  SlamControlInterface();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  
  void optimizeMovementForSLAM();
  void adjustScanRateBasedOnMovement();
  void checkSLAMQuality();
  
  bool isMovingTooFastForSLAM();
  bool isScanQualityGood();
  double calculateMovementSpeed();
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr filtered_cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr slam_quality_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slam_recommendations_pub_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  
  // State
  geometry_msgs::msg::Twist::SharedPtr last_cmd_vel_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  
  // Parameters
  double max_slam_linear_velocity_;
  double max_slam_angular_velocity_;
  double min_scan_quality_threshold_;
  double scan_rate_adjustment_factor_;
  
  // Timers
  rclcpp::TimerInterface::SharedPtr quality_check_timer_;
};
}  // namespace tadeo_ecar_slam

#endif  // TADEO_ECAR_SLAM__SLAM_CONTROL_INTERFACE_HPP_
```

```cpp
// src/slam_control_interface.cpp
#include "tadeo_ecar_slam/slam_control_interface.hpp"
#include <algorithm>
#include <cmath>

namespace tadeo_ecar_slam
{
SlamControlInterface::SlamControlInterface() : Node("slam_control_interface")
{
  // Declare parameters
  this->declare_parameter("max_slam_linear_velocity", 1.0);
  this->declare_parameter("max_slam_angular_velocity", 0.5);
  this->declare_parameter("min_scan_quality_threshold", 0.6);
  this->declare_parameter("scan_rate_adjustment_factor", 0.8);
  
  // Get parameters
  max_slam_linear_velocity_ = this->get_parameter("max_slam_linear_velocity").as_double();
  max_slam_angular_velocity_ = this->get_parameter("max_slam_angular_velocity").as_double();
  min_scan_quality_threshold_ = this->get_parameter("min_scan_quality_threshold").as_double();
  scan_rate_adjustment_factor_ = this->get_parameter("scan_rate_adjustment_factor").as_double();
  
  // Publishers
  filtered_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel_filtered", 10);
  slam_quality_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "slam_quality_good", 10);
  slam_recommendations_pub_ = this->create_publisher<std_msgs::msg::String>(
    "slam_recommendations", 10);
  
  // Subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_raw", 10,
    std::bind(&SlamControlInterface::cmdVelCallback, this, std::placeholders::_1));
  
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&SlamControlInterface::scanCallback, this, std::placeholders::_1));
  
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", 10,
    std::bind(&SlamControlInterface::mapCallback, this, std::placeholders::_1));
  
  // Timer for quality checks
  quality_check_timer_ = this->create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&SlamControlInterface::checkSLAMQuality, this));
  
  RCLCPP_INFO(this->get_logger(), "SLAM Control Interface initialized");
}

void SlamControlInterface::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel_ = msg;
  
  // Create filtered command
  auto filtered_cmd = *msg;
  
  // Limit velocities for better SLAM performance
  if (isMovingTooFastForSLAM()) {
    filtered_cmd.linear.x = std::clamp(
      filtered_cmd.linear.x, 
      -max_slam_linear_velocity_, 
      max_slam_linear_velocity_);
    
    filtered_cmd.linear.y = std::clamp(
      filtered_cmd.linear.y,
      -max_slam_linear_velocity_,
      max_slam_linear_velocity_);
    
    filtered_cmd.angular.z = std::clamp(
      filtered_cmd.angular.z,
      -max_slam_angular_velocity_,
      max_slam_angular_velocity_);
    
    RCLCPP_DEBUG(this->get_logger(), "Velocity limited for SLAM optimization");
  }
  
  filtered_cmd_vel_pub_->publish(filtered_cmd);
}

void SlamControlInterface::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
  
  // Adjust scan processing rate based on movement
  adjustScanRateBasedOnMovement();
}

void SlamControlInterface::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
}

bool SlamControlInterface::isMovingTooFastForSLAM()
{
  if (!last_cmd_vel_) {
    return false;
  }
  
  double linear_speed = std::sqrt(
    std::pow(last_cmd_vel_->linear.x, 2) + 
    std::pow(last_cmd_vel_->linear.y, 2));
  
  double angular_speed = std::abs(last_cmd_vel_->angular.z);
  
  return (linear_speed > max_slam_linear_velocity_) || 
         (angular_speed > max_slam_angular_velocity_);
}

bool SlamControlInterface::isScanQualityGood()
{
  if (!last_scan_) {
    return false;
  }
  
  size_t valid_points = 0;
  for (const auto& range : last_scan_->ranges) {
    if (range >= last_scan_->range_min && range <= last_scan_->range_max &&
        !std::isnan(range) && !std::isinf(range)) {
      valid_points++;
    }
  }
  
  double quality = static_cast<double>(valid_points) / last_scan_->ranges.size();
  return quality >= min_scan_quality_threshold_;
}

void SlamControlInterface::checkSLAMQuality()
{
  bool quality_good = isScanQualityGood();
  
  auto quality_msg = std_msgs::msg::Bool();
  quality_msg.data = quality_good;
  slam_quality_pub_->publish(quality_msg);
  
  // Generate recommendations
  auto recommendation_msg = std_msgs::msg::String();
  
  if (!quality_good) {
    if (isMovingTooFastForSLAM()) {
      recommendation_msg.data = "SLOW_DOWN: Reduce velocity for better SLAM performance";
    } else {
      recommendation_msg.data = "ENVIRONMENT: Low scan quality, check for dust or obstacles";
    }
  } else {
    recommendation_msg.data = "OPTIMAL: SLAM quality is good";
  }
  
  slam_recommendations_pub_->publish(recommendation_msg);
}

void SlamControlInterface::adjustScanRateBasedOnMovement()
{
  // This would interface with scan rate adjustment if supported by LiDAR
  double movement_speed = calculateMovementSpeed();
  
  if (movement_speed > max_slam_linear_velocity_) {
    RCLCPP_DEBUG(this->get_logger(), 
                 "High movement speed detected, recommend increasing scan rate");
  }
}

double SlamControlInterface::calculateMovementSpeed()
{
  if (!last_cmd_vel_) {
    return 0.0;
  }
  
  return std::sqrt(
    std::pow(last_cmd_vel_->linear.x, 2) + 
    std::pow(last_cmd_vel_->linear.y, 2));
}
}  // namespace tadeo_ecar_slam

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tadeo_ecar_slam::SlamControlInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## Mapeo en Tiempo Real

### Monitor de Performance de SLAM

```python
#!/usr/bin/env python3
# scripts/slam_performance_monitor.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Float32
import time
import psutil
import numpy as np

class SlamPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('slam_performance_monitor')
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'pose', self.pose_callback, 10)
        
        # Publishers
        self.performance_pub = self.create_publisher(
            String, 'slam_performance', 10)
        self.cpu_usage_pub = self.create_publisher(
            Float32, 'slam_cpu_usage', 10)
        self.memory_usage_pub = self.create_publisher(
            Float32, 'slam_memory_usage', 10)
        self.map_quality_pub = self.create_publisher(
            Float32, 'map_quality_score', 10)
        
        # Timers
        self.performance_timer = self.create_timer(1.0, self.publish_performance)
        self.system_timer = self.create_timer(5.0, self.monitor_system_resources)
        
        # State variables
        self.last_map_time = None
        self.last_scan_time = None
        self.last_pose_time = None
        self.map_update_count = 0
        self.scan_count = 0
        self.pose_update_count = 0
        self.start_time = time.time()
        
        # Performance metrics
        self.map_update_rate = 0.0
        self.scan_rate = 0.0
        self.pose_update_rate = 0.0
        self.current_map = None
        
        self.get_logger().info('SLAM Performance Monitor initialized')
    
    def map_callback(self, msg):
        current_time = time.time()
        
        if self.last_map_time is not None:
            dt = current_time - self.last_map_time
            self.map_update_rate = 1.0 / dt if dt > 0 else 0.0
        
        self.last_map_time = current_time
        self.map_update_count += 1
        self.current_map = msg
        
        # Calculate map quality
        quality_score = self.calculate_map_quality(msg)
        quality_msg = Float32()
        quality_msg.data = quality_score
        self.map_quality_pub.publish(quality_msg)
    
    def scan_callback(self, msg):
        current_time = time.time()
        
        if self.last_scan_time is not None:
            dt = current_time - self.last_scan_time
            self.scan_rate = 1.0 / dt if dt > 0 else 0.0
        
        self.last_scan_time = current_time
        self.scan_count += 1
    
    def pose_callback(self, msg):
        current_time = time.time()
        
        if self.last_pose_time is not None:
            dt = current_time - self.last_pose_time
            self.pose_update_rate = 1.0 / dt if dt > 0 else 0.0
        
        self.last_pose_time = current_time
        self.pose_update_count += 1
    
    def calculate_map_quality(self, map_msg):
        """Calculate a quality score for the map"""
        if not map_msg.data:
            return 0.0
        
        # Convert to numpy array
        map_array = np.array(map_msg.data).reshape(
            (map_msg.info.height, map_msg.info.width))
        
        # Calculate metrics
        total_cells = map_array.size
        unknown_cells = np.sum(map_array == -1)
        free_cells = np.sum(map_array == 0)
        occupied_cells = np.sum(map_array == 100)
        
        # Quality score based on exploration and clarity
        exploration_ratio = (free_cells + occupied_cells) / total_cells
        clarity_ratio = occupied_cells / (free_cells + occupied_cells + 1)  # +1 to avoid division by zero
        
        # Edge detection for map detail
        edges = 0
        for i in range(1, map_msg.info.height - 1):
            for j in range(1, map_msg.info.width - 1):
                if map_array[i, j] != -1:  # Known cell
                    neighbors = [
                        map_array[i-1, j], map_array[i+1, j],
                        map_array[i, j-1], map_array[i, j+1]
                    ]
                    if any(n != map_array[i, j] and n != -1 for n in neighbors):
                        edges += 1
        
        edge_density = edges / (map_msg.info.width * map_msg.info.height)
        
        # Combined quality score (0-100)
        quality_score = (exploration_ratio * 50 + 
                        clarity_ratio * 30 + 
                        min(edge_density * 1000, 20))  # Cap edge contribution
        
        return min(quality_score, 100.0)
    
    def publish_performance(self):
        """Publish performance metrics"""
        current_time = time.time()
        uptime = current_time - self.start_time
        
        # Create performance report
        performance_data = {
            'uptime': uptime,
            'map_updates': self.map_update_count,
            'scan_count': self.scan_count,
            'pose_updates': self.pose_update_count,
            'map_update_rate': self.map_update_rate,
            'scan_rate': self.scan_rate,
            'pose_update_rate': self.pose_update_rate
        }
        
        # Average rates
        avg_map_rate = self.map_update_count / uptime if uptime > 0 else 0
        avg_scan_rate = self.scan_count / uptime if uptime > 0 else 0
        avg_pose_rate = self.pose_update_count / uptime if uptime > 0 else 0
        
        performance_report = (
            f"SLAM Performance Report:\n"
            f"Uptime: {uptime:.1f}s\n"
            f"Map Updates: {self.map_update_count} (avg: {avg_map_rate:.2f} Hz)\n"
            f"Scan Rate: {self.scan_rate:.2f} Hz (avg: {avg_scan_rate:.2f} Hz)\n"
            f"Pose Rate: {self.pose_update_rate:.2f} Hz (avg: {avg_pose_rate:.2f} Hz)\n"
        )
        
        # Check for performance issues
        issues = []
        if self.scan_rate < 5.0:
            issues.append("Low scan rate")
        if self.map_update_rate < 0.5:
            issues.append("Slow map updates")
        if self.pose_update_rate < 1.0:
            issues.append("Infrequent pose updates")
        
        if issues:
            performance_report += f"Issues: {', '.join(issues)}"
        else:
            performance_report += "Status: OPTIMAL"
        
        performance_msg = String()
        performance_msg.data = performance_report
        self.performance_pub.publish(performance_msg)
    
    def monitor_system_resources(self):
        """Monitor CPU and memory usage"""
        # Get process info for SLAM nodes
        slam_processes = []
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent', 'memory_percent']):
            try:
                if any(slam_name in proc.info['name'].lower() 
                       for slam_name in ['slam', 'cartographer']):
                    slam_processes.append(proc.info)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        # Calculate total usage
        total_cpu = sum(p['cpu_percent'] or 0 for p in slam_processes)
        total_memory = sum(p['memory_percent'] or 0 for p in slam_processes)
        
        # Publish metrics
        cpu_msg = Float32()
        cpu_msg.data = total_cpu
        self.cpu_usage_pub.publish(cpu_msg)
        
        memory_msg = Float32()
        memory_msg.data = total_memory
        self.memory_usage_pub.publish(memory_msg)
        
        # Log if usage is high
        if total_cpu > 80.0:
            self.get_logger().warn(f'High SLAM CPU usage: {total_cpu:.1f}%')
        if total_memory > 70.0:
            self.get_logger().warn(f'High SLAM memory usage: {total_memory:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    monitor = SlamPerformanceMonitor()
    
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

## Localización y Re-localización

### Sistema de Re-localización

```cpp
// include/tadeo_ecar_slam/relocalization_system.hpp
#ifndef TADEO_ECAR_SLAM__RELOCALIZATION_SYSTEM_HPP_
#define TADEO_ECAR_SLAM__RELOCALIZATION_SYSTEM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace tadeo_ecar_slam
{
class RelocalizationSystem : public rclcpp::Node
{
public:
  RelocalizationSystem();

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  
  void checkLocalizationHealth();
  void performRelocalization();
  bool isLocalizationLost();
  
  // Service callbacks
  void forceRelocalizationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  std::vector<geometry_msgs::msg::Pose> generateRelocalizationHypotheses();
  double calculateScanMatchScore(
    const geometry_msgs::msg::Pose& pose,
    const sensor_msgs::msg::LaserScan& scan);
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr hypotheses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr localization_status_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  
  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr force_relocalization_service_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Timers
  rclcpp::TimerInterface::SharedPtr health_check_timer_;
  
  // State
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_good_pose_;
  rclcpp::Time last_good_pose_time_;
  
  // Parameters
  double max_pose_error_threshold_;
  double min_scan_match_score_;
  double relocalization_timeout_;
  int max_relocalization_attempts_;
  bool auto_relocalization_enabled_;
  
  // State tracking
  bool localization_lost_;
  int relocalization_attempts_;
  rclcpp::Time relocalization_start_time_;
};
}  // namespace tadeo_ecar_slam

#endif  // TADEO_ECAR_SLAM__RELOCALIZATION_SYSTEM_HPP_
```

### Script de Gestión de Mapas

```bash
#!/bin/bash
# scripts/map_manager.sh

MAPS_DIR="$HOME/ecar_maps"
CURRENT_MAP_LINK="$MAPS_DIR/current_map"

echo "=== eCar Map Manager ==="

# Create maps directory if it doesn't exist
mkdir -p "$MAPS_DIR"

case "$1" in
    "save")
        echo "Saving current map..."
        TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
        MAP_NAME="${2:-ecar_map_$TIMESTAMP}"
        
        # Call map saver service
        ros2 run nav2_map_server map_saver_cli -f "$MAPS_DIR/$MAP_NAME"
        
        if [ $? -eq 0 ]; then
            echo "✓ Map saved as: $MAP_NAME"
            
            # Update current map link
            ln -sf "$MAPS_DIR/$MAP_NAME.yaml" "$CURRENT_MAP_LINK.yaml"
            ln -sf "$MAPS_DIR/$MAP_NAME.pgm" "$CURRENT_MAP_LINK.pgm"
            echo "✓ Updated current map link"
        else
            echo "✗ Failed to save map"
            exit 1
        fi
        ;;
        
    "load")
        MAP_NAME="$2"
        if [ -z "$MAP_NAME" ]; then
            echo "Usage: $0 load <map_name>"
            exit 1
        fi
        
        MAP_FILE="$MAPS_DIR/$MAP_NAME.yaml"
        if [ ! -f "$MAP_FILE" ]; then
            echo "✗ Map file not found: $MAP_FILE"
            exit 1
        fi
        
        echo "Loading map: $MAP_NAME"
        
        # Launch SLAM in localization mode
        ros2 launch tadeo_ecar_slam slam_toolbox.launch.py \
            mode:=localization \
            map_file:="$MAP_FILE" &
        
        SLAM_PID=$!
        echo "✓ SLAM launched in localization mode (PID: $SLAM_PID)"
        ;;
        
    "list")
        echo "Available maps:"
        if ls "$MAPS_DIR"/*.yaml 1> /dev/null 2>&1; then
            for map_file in "$MAPS_DIR"/*.yaml; do
                map_name=$(basename "$map_file" .yaml)
                map_size=$(stat -c%s "$map_file")
                map_date=$(stat -c%y "$map_file" | cut -d' ' -f1)
                
                current_marker=""
                if [ -L "$CURRENT_MAP_LINK.yaml" ] && [ "$(readlink "$CURRENT_MAP_LINK.yaml")" = "$map_file" ]; then
                    current_marker=" (current)"
                fi
                
                printf "  %-30s %8s bytes %12s%s\n" "$map_name" "$map_size" "$map_date" "$current_marker"
            done
        else
            echo "  No maps found in $MAPS_DIR"
        fi
        ;;
        
    "delete")
        MAP_NAME="$2"
        if [ -z "$MAP_NAME" ]; then
            echo "Usage: $0 delete <map_name>"
            exit 1
        fi
        
        MAP_YAML="$MAPS_DIR/$MAP_NAME.yaml"
        MAP_PGM="$MAPS_DIR/$MAP_NAME.pgm"
        
        if [ ! -f "$MAP_YAML" ]; then
            echo "✗ Map not found: $MAP_NAME"
            exit 1
        fi
        
        echo "Are you sure you want to delete map '$MAP_NAME'? (y/N)"
        read -r confirmation
        
        if [ "$confirmation" = "y" ] || [ "$confirmation" = "Y" ]; then
            rm -f "$MAP_YAML" "$MAP_PGM"
            echo "✓ Map '$MAP_NAME' deleted"
            
            # Remove current link if it pointed to deleted map
            if [ -L "$CURRENT_MAP_LINK.yaml" ] && [ "$(readlink "$CURRENT_MAP_LINK.yaml")" = "$MAP_YAML" ]; then
                rm -f "$CURRENT_MAP_LINK.yaml" "$CURRENT_MAP_LINK.pgm"
                echo "✓ Removed current map link"
            fi
        else
            echo "Delete cancelled"
        fi
        ;;
        
    "current")
        if [ -L "$CURRENT_MAP_LINK.yaml" ]; then
            current_map=$(readlink "$CURRENT_MAP_LINK.yaml")
            current_name=$(basename "$current_map" .yaml)
            echo "Current map: $current_name"
            echo "Path: $current_map"
        else
            echo "No current map set"
        fi
        ;;
        
    "info")
        MAP_NAME="$2"
        if [ -z "$MAP_NAME" ]; then
            if [ -L "$CURRENT_MAP_LINK.yaml" ]; then
                MAP_FILE=$(readlink "$CURRENT_MAP_LINK.yaml")
                MAP_NAME=$(basename "$MAP_FILE" .yaml)
            else
                echo "Usage: $0 info <map_name>"
                exit 1
            fi
        else
            MAP_FILE="$MAPS_DIR/$MAP_NAME.yaml"
        fi
        
        if [ ! -f "$MAP_FILE" ]; then
            echo "✗ Map file not found: $MAP_FILE"
            exit 1
        fi
        
        echo "Map Information: $MAP_NAME"
        echo "================================="
        
        # Parse YAML file for map info
        if command -v yq >/dev/null 2>&1; then
            echo "Resolution: $(yq '.resolution' "$MAP_FILE") m/pixel"
            echo "Origin: [$(yq '.origin[0]' "$MAP_FILE"), $(yq '.origin[1]' "$MAP_FILE"), $(yq '.origin[2]' "$MAP_FILE")]"
            echo "Image: $(yq '.image' "$MAP_FILE")"
        else
            echo "Map file: $MAP_FILE"
            echo "Note: Install 'yq' for detailed map information"
        fi
        
        # File information
        map_size=$(stat -c%s "$MAP_FILE")
        map_date=$(stat -c%y "$MAP_FILE")
        echo "File size: $map_size bytes"
        echo "Created: $map_date"
        ;;
        
    "start-mapping")
        echo "Starting SLAM mapping mode..."
        ros2 launch tadeo_ecar_slam slam_toolbox.launch.py mode:=mapping
        ;;
        
    *)
        echo "Usage: $0 {save|load|list|delete|current|info|start-mapping} [map_name]"
        echo ""
        echo "Commands:"
        echo "  save [name]     - Save current map (auto-generate name if not provided)"
        echo "  load <name>     - Load map and start localization"
        echo "  list            - List all available maps"
        echo "  delete <name>   - Delete a map"
        echo "  current         - Show current active map"
        echo "  info [name]     - Show map information (current map if name not provided)"
        echo "  start-mapping   - Start SLAM in mapping mode"
        exit 1
        ;;
esac
```

## Optimización y Tuning

### Configuración de Rendimiento por Entorno

```yaml
# config/slam_performance_profiles.yaml
slam_profiles:
  
  # Indoor environment (high precision, moderate speed)
  indoor:
    slam_toolbox:
      resolution: 0.03
      correlation_search_space_dimension: 0.3
      correlation_search_space_resolution: 0.005
      minimum_travel_distance: 0.2
      minimum_travel_heading: 0.2
      scan_buffer_size: 15
      loop_search_maximum_distance: 4.0
      
  # Outdoor environment (moderate precision, higher speed)  
  outdoor:
    slam_toolbox:
      resolution: 0.05
      correlation_search_space_dimension: 0.5
      correlation_search_space_resolution: 0.01
      minimum_travel_distance: 0.5
      minimum_travel_heading: 0.3
      scan_buffer_size: 10
      loop_search_maximum_distance: 6.0
      
  # Warehouse environment (balanced for efficiency)
  warehouse:
    slam_toolbox:
      resolution: 0.04
      correlation_search_space_dimension: 0.4
      correlation_search_space_resolution: 0.008
      minimum_travel_distance: 0.3
      minimum_travel_heading: 0.25
      scan_buffer_size: 12
      loop_search_maximum_distance: 5.0
      
  # High-precision mode (for detailed mapping)
  precision:
    slam_toolbox:
      resolution: 0.02
      correlation_search_space_dimension: 0.2
      correlation_search_space_resolution: 0.003
      minimum_travel_distance: 0.1
      minimum_travel_heading: 0.1
      scan_buffer_size: 20
      loop_search_maximum_distance: 3.0
      
  # Fast mapping mode (for quick exploration)
  fast:
    slam_toolbox:
      resolution: 0.08
      correlation_search_space_dimension: 0.6
      correlation_search_space_resolution: 0.02
      minimum_travel_distance: 0.8
      minimum_travel_heading: 0.5
      scan_buffer_size: 8
      loop_search_maximum_distance: 8.0
```

### Herramientas de Análisis

```python
#!/usr/bin/env python3
# scripts/slam_analysis_tools.py

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import cv2
from scipy.spatial.distance import pdist
import yaml

class SlamAnalysisTools(Node):
    def __init__(self):
        super().__init__('slam_analysis_tools')
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        self.path_sub = self.create_subscription(
            Path, 'path', self.path_callback, 10)
        
        # State
        self.current_map = None
        self.robot_path = []
        
        self.get_logger().info('SLAM Analysis Tools initialized')
    
    def map_callback(self, msg):
        self.current_map = msg
    
    def path_callback(self, msg):
        """Store robot path for analysis"""
        if msg.poses:
            self.robot_path = [(pose.pose.position.x, pose.pose.position.y) 
                              for pose in msg.poses]
    
    def analyze_map_quality(self):
        """Comprehensive map quality analysis"""
        if not self.current_map:
            self.get_logger().warn('No map available for analysis')
            return None
        
        # Convert to numpy array
        map_data = np.array(self.current_map.data).reshape(
            (self.current_map.info.height, self.current_map.info.width))
        
        results = {}
        
        # Basic statistics
        total_cells = map_data.size
        unknown_cells = np.sum(map_data == -1)
        free_cells = np.sum(map_data == 0)
        occupied_cells = np.sum(map_data == 100)
        
        results['total_cells'] = total_cells
        results['unknown_ratio'] = unknown_cells / total_cells
        results['free_ratio'] = free_cells / total_cells
        results['occupied_ratio'] = occupied_cells / total_cells
        results['explored_ratio'] = (free_cells + occupied_cells) / total_cells
        
        # Map connectivity analysis
        results['connectivity'] = self.analyze_connectivity(map_data)
        
        # Edge detection for detail analysis
        results['edge_density'] = self.calculate_edge_density(map_data)
        
        # Noise analysis
        results['noise_level'] = self.calculate_noise_level(map_data)
        
        # Loop closure quality
        if len(self.robot_path) > 10:
            results['path_consistency'] = self.analyze_path_consistency()
        
        return results
    
    def analyze_connectivity(self, map_data):
        """Analyze map connectivity using flood fill"""
        free_cells = (map_data == 0).astype(np.uint8)
        
        # Find connected components
        num_labels, labels = cv2.connectedComponents(free_cells)
        
        if num_labels <= 1:
            return 1.0  # Perfect connectivity
        
        # Calculate largest component ratio
        component_sizes = [np.sum(labels == i) for i in range(1, num_labels)]
        largest_component = max(component_sizes) if component_sizes else 0
        total_free = np.sum(free_cells)
        
        connectivity_ratio = largest_component / total_free if total_free > 0 else 0
        return connectivity_ratio
    
    def calculate_edge_density(self, map_data):
        """Calculate edge density as a measure of map detail"""
        # Apply edge detection
        edges = cv2.Canny(
            ((map_data + 1) * 127).astype(np.uint8), 50, 150)
        
        edge_pixels = np.sum(edges > 0)
        total_known = np.sum(map_data != -1)
        
        return edge_pixels / total_known if total_known > 0 else 0
    
    def calculate_noise_level(self, map_data):
        """Calculate noise level using isolated pixels"""
        known_mask = (map_data != -1)
        occupied_mask = (map_data == 100)
        
        noise_count = 0
        total_occupied = np.sum(occupied_mask)
        
        # Check for isolated occupied pixels
        for i in range(1, map_data.shape[0] - 1):
            for j in range(1, map_data.shape[1] - 1):
                if occupied_mask[i, j]:
                    # Check 8-connected neighbors
                    neighbors = map_data[i-1:i+2, j-1:j+2]
                    neighbor_occupied = np.sum(neighbors == 100) - 1  # Exclude center
                    
                    if neighbor_occupied == 0:  # Isolated pixel
                        noise_count += 1
        
        return noise_count / total_occupied if total_occupied > 0 else 0
    
    def analyze_path_consistency(self):
        """Analyze path consistency for loop closure quality"""
        if len(self.robot_path) < 10:
            return 0.0
        
        path_array = np.array(self.robot_path)
        
        # Calculate path smoothness
        velocities = np.diff(path_array, axis=0)
        accelerations = np.diff(velocities, axis=0)
        
        # RMS acceleration as smoothness measure
        rms_acceleration = np.sqrt(np.mean(np.sum(accelerations**2, axis=1)))
        
        # Normalize (lower is better, but we want higher scores for better quality)
        consistency_score = 1.0 / (1.0 + rms_acceleration)
        
        return consistency_score
    
    def generate_analysis_report(self):
        """Generate comprehensive analysis report"""
        results = self.analyze_map_quality()
        
        if not results:
            return "No map data available for analysis"
        
        report = "=== SLAM Quality Analysis Report ===\n\n"
        
        # Basic statistics
        report += f"Map Coverage:\n"
        report += f"  Explored: {results['explored_ratio']:.1%}\n"
        report += f"  Free space: {results['free_ratio']:.1%}\n"
        report += f"  Obstacles: {results['occupied_ratio']:.1%}\n"
        report += f"  Unknown: {results['unknown_ratio']:.1%}\n\n"
        
        # Quality metrics
        report += f"Quality Metrics:\n"
        report += f"  Connectivity: {results['connectivity']:.3f}\n"
        report += f"  Detail level: {results['edge_density']:.4f}\n"
        report += f"  Noise level: {results['noise_level']:.4f}\n"
        
        if 'path_consistency' in results:
            report += f"  Path consistency: {results['path_consistency']:.3f}\n"
        
        # Overall quality score
        quality_score = (
            results['explored_ratio'] * 0.3 +
            results['connectivity'] * 0.3 +
            min(results['edge_density'] * 100, 1.0) * 0.2 +
            (1.0 - results['noise_level']) * 0.2
        )
        
        report += f"\nOverall Quality Score: {quality_score:.2f}/1.00\n"
        
        # Recommendations
        report += "\nRecommendations:\n"
        if results['explored_ratio'] < 0.7:
            report += "  - Continue exploration to improve coverage\n"
        if results['connectivity'] < 0.8:
            report += "  - Check for mapping artifacts or obstacles\n"
        if results['noise_level'] > 0.1:
            report += "  - Consider tuning SLAM parameters to reduce noise\n"
        if results['edge_density'] < 0.01:
            report += "  - Environment may lack features, consider adding landmarks\n"
        
        return report
    
    def save_analysis_to_file(self, filename):
        """Save analysis results to YAML file"""
        results = self.analyze_map_quality()
        
        if not results:
            return False
        
        # Add metadata
        results['timestamp'] = self.get_clock().now().to_msg()
        results['map_info'] = {
            'width': self.current_map.info.width,
            'height': self.current_map.info.height,
            'resolution': self.current_map.info.resolution
        }
        
        try:
            with open(filename, 'w') as f:
                yaml.dump(results, f, default_flow_style=False)
            self.get_logger().info(f'Analysis saved to {filename}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save analysis: {e}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    analyzer = SlamAnalysisTools()
    
    # Example usage
    try:
        rclpy.spin_once(analyzer, timeout_sec=5.0)  # Let it collect some data
        
        report = analyzer.generate_analysis_report()
        print(report)
        
        # Save to file
        analyzer.save_analysis_to_file('/tmp/slam_analysis.yaml')
        
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Ejercicios Prácticos

### Ejercicio 1: Configurar SLAM para Diferentes Entornos

```bash
# 1. Mapear entorno interior
ros2 launch tadeo_ecar_slam slam_toolbox.launch.py \
    slam_params_file:=config/indoor_slam_params.yaml

# 2. Guardar mapa interior
./scripts/map_manager.sh save indoor_map

# 3. Mapear entorno exterior  
ros2 launch tadeo_ecar_slam slam_toolbox.launch.py \
    slam_params_file:=config/outdoor_slam_params.yaml

# 4. Comparar calidad de mapas
python3 scripts/slam_analysis_tools.py
```

### Ejercicio 2: Optimizar Parámetros SLAM

```yaml
# TODO: Crear config/optimized_slam_params.yaml
# - Ajustar resolution para el entorno
# - Configurar search space parameters
# - Optimizar loop closure detection
# - Probar diferentes configuraciones
```

### Ejercicio 3: Implementar Re-localización Robusta

```cpp
// TODO: Mejorar sistema de re-localización
// - Implementar múltiples hipótesis
// - Usar scan matching para validación
// - Crear recovery behaviors
// - Integrar con sistema de navegación
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Fundamentos SLAM**: Localización y mapeo simultáneos
2. **Algoritmos**: SLAM Toolbox, Cartographer, comparaciones
3. **Configuración**: Parámetros específicos para eCar 4WD4WS
4. **Tiempo Real**: Monitoreo y optimización de performance
5. **Re-localización**: Sistemas de recuperación robustos
6. **Análisis**: Herramientas de evaluación de calidad
7. **Gestión**: Scripts para manejo de mapas
8. **Optimización**: Tuning por entorno y aplicación

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes el problema y soluciones SLAM
- [ ] Puedes configurar SLAM Toolbox para eCar
- [ ] Sabes optimizar parámetros por entorno
- [ ] Comprendes sistemas de re-localización
- [ ] Puedes analizar calidad de mapas
- [ ] Has implementado monitoring de SLAM
- [ ] Sabes gestionar bibliotecas de mapas

### Próximo Capítulo

En el Capítulo 12 estudiaremos:
- Behavior Trees en profundidad
- Diseño de comportamientos complejos
- Estado finito vs árbol de comportamiento
- Implementación para eCar
- Debugging y visualización

## Referencias

- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Cartographer](https://google-cartographer.readthedocs.io/)
- [ROS2 Navigation](https://navigation.ros.org/)
- [SLAM Algorithms Survey](https://arxiv.org/abs/1606.05830)
- [Graph-based SLAM](https://openslam-org.github.io/)