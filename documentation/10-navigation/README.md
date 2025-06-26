# Capítulo 10: Navegación Autónoma en ROS2

## Tabla de Contenidos

1. [Introducción a Nav2](#introducción-a-nav2)
2. [Arquitectura de Navegación](#arquitectura-de-navegación)
3. [Costmaps](#costmaps)
4. [Planificadores](#planificadores)
5. [Controladores](#controladores)
6. [Behavior Trees](#behavior-trees)
7. [Implementación en el eCar](#implementación-en-el-ecar)
8. [Configuración Avanzada](#configuración-avanzada)

## Introducción a Nav2

### ¿Qué es Nav2?

Nav2 (Navigation2) es el stack de navegación autónoma de ROS2, diseñado para robots móviles que necesitan navegar desde un punto A hasta un punto B evitando obstáculos.

```
Nav2 = Sistema Completo de Navegación Autónoma

Entrada: Goal Pose (destino deseado)
Salida: cmd_vel (comandos de velocidad)

Incluye: Planificación, Control, Evitación, Recovery
```

### Características Principales

**1. Planificación Global**
```bash
# Encuentra la ruta óptima desde posición actual hasta objetivo
Global Planner: A*, Dijkstra, Hybrid A*, etc.
```

**2. Planificación Local**
```bash
# Navega localmente evitando obstáculos dinámicos
Local Planner: DWB, TEB, MPPI, etc.
```

**3. Costmaps**
```bash
# Representación del mundo para planificación
Global Costmap: Mapa estático + obstáculos detectados
Local Costmap: Ventana local con obstáculos dinámicos
```

**4. Recovery Behaviors**
```bash
# Acciones cuando el robot se "atasca"
- Clear costmap
- Backup
- Spin recovery
- Wait
```

### Ventajas de Nav2 para el eCar

**Navegación 4WD4WS Específica:**
- Soporte para cinemática omnidireccional
- Planificación que aprovecha movimiento lateral
- Control de velocidad en 3 DOF (x, y, θ)

**Robustez:**
- Múltiples planificadores y controladores
- Recovery behaviors para situaciones complejas
- Monitoreo de progreso y timeouts

### Flujo Típico de Navegación

```
1. Recibir Goal → 2. Planificación Global → 3. Planificación Local → 4. Control → 5. Monitoreo
      ↓                    ↓                      ↓                ↓           ↓
  PoseStamped        Global Path           Local Path         cmd_vel    Progress Check
     |                    |                      |                |           |
     └── nav2_bt_navigator → nav2_planner → nav2_controller → nav2_controller → nav2_bt_navigator
```

## Arquitectura de Navegación

### Componentes Principales de Nav2

```
                    Nav2 Stack Architecture
                           |
    ┌─────────────────────────────────────────────────────────┐
    |                BT Navigator                             |
    |  (Coordina todo el proceso de navegación)               |
    └─────────────────┬───────────────────────────────────────┘
                      |
    ┌─────────────────┼───────────────────────────────────────┐
    |                 |                                       |
┌───▼────┐    ┌───────▼────┐    ┌──────────▼────┐    ┌───────▼───┐
│Global  │    │Local       │    │Controller     │    │Recovery   │
│Planner │    │Planner     │    │Server         │    │Server     │
└────────┘    └────────────┘    └───────────────┘    └───────────┘
     |             |                      |                  |
┌────▼────┐   ┌────▼────┐         ┌──────▼──────┐     ┌──────▼──────┐
│Global   │   │Local    │         │Controller   │     │Recovery     │
│Costmap  │   │Costmap  │         │Plugins      │     │Behaviors    │
└─────────┘   └─────────┘         └─────────────┘     └─────────────┘
```

### Nav2 Servers

**1. BT Navigator**
```cpp
// Coordina el proceso completo usando Behavior Trees
nav2_bt_navigator_node
// - Ejecuta árbol de comportamiento
// - Maneja goals y cancellations
// - Coordina recovery behaviors
```

**2. Planner Server**
```cpp
// Planificación global de rutas
nav2_planner_node
// - Calcula ruta desde inicio a objetivo
// - Usa global costmap
// - Plugins: NavFn, GlobalPlanner, SmacPlanner
```

**3. Controller Server**
```cpp
// Control local y seguimiento de trayectoria
nav2_controller_node
// - Sigue global path
// - Evita obstáculos locales
// - Plugins: DWB, TEB, MPPI, PurePursuit
```

**4. Recovery Server**
```cpp
// Comportamientos de recuperación
nav2_recoveries_node
// - Clear costmap
// - Backup
// - Spin
// - Wait
```

### Costmap 2D

**Global Costmap**
```yaml
# Configuración típica global costmap
global_costmap:
  ros__parameters:
    update_frequency: 1.0
    publish_frequency: 1.0
    global_frame: map
    robot_base_frame: base_link
    resolution: 0.05
    width: 100
    height: 100
```

**Local Costmap**
```yaml
# Configuración típica local costmap
local_costmap:
  ros__parameters:
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_link
    resolution: 0.05
    width: 5.0
    height: 5.0
    rolling_window: true
```

## Costmaps

### Concepto de Costmap

Un costmap es una grilla 2D donde cada celda tiene un costo asociado que representa qué tan "difícil" o "peligroso" es para el robot ocupar esa posición.

```
Valores de Costo:
  0   = Espacio libre (LETHAL_OBSTACLE = 254)
  100 = Obstáculo desconocido
  254 = Obstáculo letal  
  255 = Obstáculo no mapeado
```

### Layers del Costmap

**1. Static Layer**
```yaml
# Mapa estático base
static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: True
  subscribe_to_updates: True
```

**2. Obstacle Layer**
```yaml
# Obstáculos detectados por sensores
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: True
    marking: True
```

**3. Inflation Layer**
```yaml
# Infla obstáculos para seguridad
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.55
```

### Configuración del eCar Costmap

```yaml
# config/costmap_common.yaml
footprint: "[[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]]"
robot_radius: 0.7  # Para aproximación circular

plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

static_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_subscribe_transient_local: True

obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan"
    raytrace_max_range: 10.0
    obstacle_max_range: 8.0
    raytrace_min_range: 0.0
    obstacle_min_range: 0.0

inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.55
```

### Costmap Personalizado para eCar

```cpp
// include/tadeo_ecar_navigation/ecar_costmap_plugin.hpp
#ifndef TADEO_ECAR_NAVIGATION__ECAR_COSTMAP_PLUGIN_HPP_
#define TADEO_ECAR_NAVIGATION__ECAR_COSTMAP_PLUGIN_HPP_

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace tadeo_ecar_navigation
{
class ECarCostmapPlugin : public nav2_costmap_2d::Layer
{
public:
  ECarCostmapPlugin();

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override;
  virtual void onFootprintChanged() override;

private:
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void processLaserData();
  
  // Algoritmo específico para eCar 4WD4WS
  void addObstacle(nav2_costmap_2d::Costmap2D & costmap, 
                   double x, double y, double inflation_radius);
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  
  double obstacle_inflation_radius_;
  double max_obstacle_distance_;
  bool use_ecar_specific_inflation_;
};
}  // namespace tadeo_ecar_navigation

#endif  // TADEO_ECAR_NAVIGATION__ECAR_COSTMAP_PLUGIN_HPP_
```

```cpp
// src/ecar_costmap_plugin.cpp
#include "tadeo_ecar_navigation/ecar_costmap_plugin.hpp"
#include <nav2_costmap_2d/costmap_math.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace tadeo_ecar_navigation
{
ECarCostmapPlugin::ECarCostmapPlugin() : last_scan_(nullptr)
{
}

void ECarCostmapPlugin::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }
  
  // Declarar parámetros específicos del eCar
  declareParameter("obstacle_inflation_radius", rclcpp::ParameterValue(0.3));
  declareParameter("max_obstacle_distance", rclcpp::ParameterValue(5.0));
  declareParameter("use_ecar_specific_inflation", rclcpp::ParameterValue(true));
  
  // Obtener parámetros
  node->get_parameter(name_ + ".obstacle_inflation_radius", obstacle_inflation_radius_);
  node->get_parameter(name_ + ".max_obstacle_distance", max_obstacle_distance_);
  node->get_parameter(name_ + ".use_ecar_specific_inflation", use_ecar_specific_inflation_);
  
  // Suscribirse a LiDAR
  laser_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&ECarCostmapPlugin::laserCallback, this, std::placeholders::_1));
  
  current_ = true;
  enabled_ = true;
  
  RCLCPP_INFO(node->get_logger(), "eCar Costmap Plugin initialized");
}

void ECarCostmapPlugin::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
}

void ECarCostmapPlugin::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!last_scan_) {
    return;
  }
  
  // Calcular bounds basado en datos del LiDAR
  double max_range = std::min(last_scan_->range_max, max_obstacle_distance_);
  
  *min_x = std::min(*min_x, robot_x - max_range);
  *min_y = std::min(*min_y, robot_y - max_range);
  *max_x = std::max(*max_x, robot_x + max_range);
  *max_y = std::max(*max_y, robot_y + max_range);
}

void ECarCostmapPlugin::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!last_scan_) {
    return;
  }
  
  // Procesar cada punto del LiDAR
  for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
    float range = last_scan_->ranges[i];
    
    // Filtrar lecturas válidas
    if (range < last_scan_->range_min || range > last_scan_->range_max ||
        std::isnan(range) || std::isinf(range)) {
      continue;
    }
    
    // Limitar distancia máxima
    if (range > max_obstacle_distance_) {
      continue;
    }
    
    // Calcular posición del obstáculo
    float angle = last_scan_->angle_min + i * last_scan_->angle_increment;
    double obstacle_x = range * cos(angle);
    double obstacle_y = range * sin(angle);
    
    // Convertir a coordenadas globales (simplificado)
    // En implementación real usar TF2
    
    // Agregar obstáculo al costmap
    if (use_ecar_specific_inflation_) {
      addObstacle(master_grid, obstacle_x, obstacle_y, obstacle_inflation_radius_);
    } else {
      unsigned int mx, my;
      if (master_grid.worldToMap(obstacle_x, obstacle_y, mx, my)) {
        master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}

void ECarCostmapPlugin::addObstacle(
  nav2_costmap_2d::Costmap2D & costmap,
  double x, double y, double inflation_radius)
{
  // Implementación específica para eCar 4WD4WS
  // Considera la geometría del robot y su capacidad de movimiento lateral
  
  unsigned int mx, my;
  if (!costmap.worldToMap(x, y, mx, my)) {
    return;
  }
  
  // Inflar en forma que considera movimiento omnidireccional
  unsigned int inflation_cells = static_cast<unsigned int>(
    inflation_radius / costmap.getResolution());
  
  for (unsigned int i = mx - inflation_cells; i <= mx + inflation_cells; ++i) {
    for (unsigned int j = my - inflation_cells; j <= my + inflation_cells; ++j) {
      if (i >= costmap.getSizeInCellsX() || j >= costmap.getSizeInCellsY()) {
        continue;
      }
      
      double distance = std::sqrt(
        std::pow(static_cast<double>(i - mx), 2) + 
        std::pow(static_cast<double>(j - my), 2)
      ) * costmap.getResolution();
      
      if (distance <= inflation_radius) {
        // Costo gradual basado en distancia
        unsigned char cost = static_cast<unsigned char>(
          nav2_costmap_2d::LETHAL_OBSTACLE * (1.0 - distance / inflation_radius));
        costmap.setCost(i, j, std::max(costmap.getCost(i, j), cost));
      }
    }
  }
}

void ECarCostmapPlugin::reset()
{
  last_scan_.reset();
}

void ECarCostmapPlugin::onFootprintChanged()
{
  // Actualizar parámetros si cambia footprint del robot
}
}  // namespace tadeo_ecar_navigation

// Register plugin
PLUGINLIB_EXPORT_CLASS(tadeo_ecar_navigation::ECarCostmapPlugin, nav2_costmap_2d::Layer)
```

## Planificadores

### Global Planners

**1. NavFn Planner**
```yaml
# Planificador Dijkstra clásico
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

**2. Smac Planner**
```yaml
# Planificador híbrido A* (mejor para eCar)
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      tolerance: 0.5
      downsample_costmap: false
      downsampling_factor: 1
      motion_model_for_search: "DUBIN"  # Para eCar usar "OMNI"
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
```

### Local Planners/Controllers

**1. DWB Controller**
```yaml
# Dynamic Window Approach optimizado para eCar
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: -0.5  # eCar puede ir hacia atrás
      max_vel_x: 2.0
      min_vel_y: -1.0  # Movimiento lateral (4WS)
      max_vel_y: 1.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 2.0
      min_speed_theta: 0.0
      # Critics específicos para eCar
      critics: [
        "RotateToGoal", "Oscillation", "BaseObstacle", 
        "GoalAlign", "PathAlign", "PathDist", "GoalDist"
      ]
```

**2. TEB Controller**
```yaml
# Timed Elastic Band - Excelente para eCar 4WD4WS
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "teb_local_planner::TebLocalPlannerROS"
      
      # Robot configuration
      max_vel_x: 2.0
      max_vel_x_backwards: 0.5
      max_vel_y: 1.0          # Habilitar movimiento lateral
      max_vel_theta: 1.0
      acc_lim_x: 1.0
      acc_lim_y: 1.0          # Aceleración lateral
      acc_lim_theta: 1.0
      
      # Holonomic robot (eCar 4WD4WS)
      holonomic_robot: True
      
      # Trajectory configuration
      teb_autosize: True
      dt_ref: 0.3
      dt_hysteresis: 0.1
      min_samples: 3
      global_plan_overwrite_orientation: True
```

### Controlador Personalizado para eCar

```cpp
// include/tadeo_ecar_navigation/ecar_controller.hpp
#ifndef TADEO_ECAR_NAVIGATION__ECAR_CONTROLLER_HPP_
#define TADEO_ECAR_NAVIGATION__ECAR_CONTROLLER_HPP_

#include <nav2_core/controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>

namespace tadeo_ecar_navigation
{
class ECarController : public nav2_core::Controller
{
public:
  ECarController() = default;
  ~ECarController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // Control específico para eCar 4WD4WS
  geometry_msgs::msg::Twist computeOmnidirectionalCommand(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & target_pose);
  
  double calculateLateralError(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & path);
  
  size_t findClosestPathPoint(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & path);

  // Parameters
  double max_linear_vel_x_;
  double max_linear_vel_y_;
  double max_angular_vel_;
  double kp_linear_;
  double kp_angular_;
  double kp_lateral_;
  double lookahead_distance_;
  double goal_tolerance_;
  
  // State
  nav_msgs::msg::Path current_path_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
};
}  // namespace tadeo_ecar_navigation

#endif  // TADEO_ECAR_NAVIGATION__ECAR_CONTROLLER_HPP_
```

```cpp
// src/ecar_controller.cpp
#include "tadeo_ecar_navigation/ecar_controller.hpp"
#include <nav2_util/node_utils.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>
#include <algorithm>
#include <memory>

namespace tadeo_ecar_navigation
{
void ECarController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;

  // Declare parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel_x", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_vel_y", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_linear", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_angular", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".kp_lateral", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.2));

  // Get parameters
  node->get_parameter(plugin_name_ + ".max_linear_vel_x", max_linear_vel_x_);
  node->get_parameter(plugin_name_ + ".max_linear_vel_y", max_linear_vel_y_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".kp_linear", kp_linear_);
  node->get_parameter(plugin_name_ + ".kp_angular", kp_angular_);
  node->get_parameter(plugin_name_ + ".kp_lateral", kp_lateral_);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);

  RCLCPP_INFO(
    node->get_logger(), "eCar Controller configured with omnidirectional support");
}

void ECarController::cleanup() {}
void ECarController::activate() {}
void ECarController::deactivate() {}

geometry_msgs::msg::TwistStamped ECarController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto cmd = geometry_msgs::msg::TwistStamped();
  cmd.header.stamp = pose.header.stamp;
  cmd.header.frame_id = pose.header.frame_id;

  if (current_path_.poses.empty()) {
    return cmd;
  }

  // Find target point with lookahead
  size_t target_index = findClosestPathPoint(pose, current_path_);
  
  // Look ahead for smoother trajectory
  double accumulated_distance = 0.0;
  for (size_t i = target_index; i < current_path_.poses.size() - 1; ++i) {
    auto & p1 = current_path_.poses[i].pose.position;
    auto & p2 = current_path_.poses[i + 1].pose.position;
    
    accumulated_distance += std::sqrt(
      std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    
    if (accumulated_distance >= lookahead_distance_) {
      target_index = i + 1;
      break;
    }
  }
  
  // Ensure target index is valid
  target_index = std::min(target_index, current_path_.poses.size() - 1);
  
  // Compute omnidirectional command
  cmd.twist = computeOmnidirectionalCommand(pose, current_path_.poses[target_index]);
  
  return cmd;
}

geometry_msgs::msg::Twist ECarController::computeOmnidirectionalCommand(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & target_pose)
{
  auto cmd = geometry_msgs::msg::Twist();
  
  // Calculate errors in robot frame
  double dx = target_pose.pose.position.x - current_pose.pose.position.x;
  double dy = target_pose.pose.position.y - current_pose.pose.position.y;
  
  // Get current orientation
  tf2::Quaternion current_quat;
  tf2::fromMsg(current_pose.pose.orientation, current_quat);
  double current_yaw = tf2::getYaw(current_quat);
  
  // Get target orientation
  tf2::Quaternion target_quat;
  tf2::fromMsg(target_pose.pose.orientation, target_quat);
  double target_yaw = tf2::getYaw(target_quat);
  
  // Transform position error to robot frame
  double cos_yaw = std::cos(current_yaw);
  double sin_yaw = std::sin(current_yaw);
  
  double error_x_robot = dx * cos_yaw + dy * sin_yaw;
  double error_y_robot = -dx * sin_yaw + dy * cos_yaw;
  
  // Calculate linear velocities (omnidirectional)
  cmd.linear.x = kp_linear_ * error_x_robot;
  cmd.linear.y = kp_lateral_ * error_y_robot;  // Lateral movement for 4WS
  
  // Calculate angular velocity
  double angular_error = angles::shortest_angular_distance(current_yaw, target_yaw);
  cmd.angular.z = kp_angular_ * angular_error;
  
  // Apply velocity limits
  cmd.linear.x = std::clamp(cmd.linear.x, -max_linear_vel_x_, max_linear_vel_x_);
  cmd.linear.y = std::clamp(cmd.linear.y, -max_linear_vel_y_, max_linear_vel_y_);
  cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_vel_, max_angular_vel_);
  
  return cmd;
}

size_t ECarController::findClosestPathPoint(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return 0;
  }
  
  size_t closest_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - current_pose.pose.position.x;
    double dy = path.poses[i].pose.position.y - current_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  
  return closest_index;
}

void ECarController::setPlan(const nav_msgs::msg::Path & path)
{
  current_path_ = path;
}

void ECarController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_linear_vel_x_ *= speed_limit / 100.0;
    max_linear_vel_y_ *= speed_limit / 100.0;
    max_angular_vel_ *= speed_limit / 100.0;
  } else {
    max_linear_vel_x_ = speed_limit;
    max_linear_vel_y_ = speed_limit * 0.5;  // Lateral speed typically lower
  }
}
}  // namespace tadeo_ecar_navigation

// Register plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tadeo_ecar_navigation::ECarController, nav2_core::Controller)
```

## Controladores

### Configuración de Controller Server

```yaml
# config/controller.yaml
controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001    # Importante para eCar 4WS
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    
    # Progress checker
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
      
    # Goal checker  
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      
    # eCar Controller Plugin
    FollowPath:
      plugin: "tadeo_ecar_navigation::ECarController"
      max_linear_vel_x: 2.0
      max_linear_vel_y: 1.0        # Velocidad lateral
      max_angular_vel: 1.0
      kp_linear: 1.0
      kp_angular: 2.0
      kp_lateral: 1.5              # Ganancia para control lateral
      lookahead_distance: 1.0
      goal_tolerance: 0.2
```

## Behavior Trees

### Introducción a BT en Nav2

Los Behavior Trees coordinan todo el proceso de navegación, manejando la lógica de alto nivel, recovery behaviors y manejo de errores.

```xml
<!-- behavior_trees/navigate_to_pose_w_replanning.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        
        <!-- Compute Path -->
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ReactiveFallback name="ComputePathRecoveryFallback">
              <GoalUpdated/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
          </RecoveryNode>
        </RateController>
        
        <!-- Follow Path -->
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ReactiveFallback name="FollowPathRecoveryFallback">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </ReactiveFallback>
        </RecoveryNode>
        
      </PipelineSequence>
      
      <!-- Recovery Behaviors -->
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.30" backup_speed="0.05"/>
        </RoundRobin>
      </ReactiveFallback>
      
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### BT Personalizado para eCar

```xml
<!-- behavior_trees/ecar_navigation.xml -->
<root main_tree_to_execute="ECarNavigation">
  <BehaviorTree ID="ECarNavigation">
    
    <RecoveryNode number_of_retries="8" name="ECarNavigationRecovery">
      <PipelineSequence name="ECarNavigationSequence">
        
        <!-- Pre-navigation checks -->
        <Sequence name="PreNavigationChecks">
          <Condition ID="IsBatteryOK"/>
          <Condition ID="AreWheelsCalibrated"/>
          <Condition ID="IsSafeToMove"/>
        </Sequence>
        
        <!-- Path planning with eCar-specific settings -->
        <RateController hz="0.5">
          <RecoveryNode number_of_retries="2" name="ECarComputePath">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ReactiveFallback name="PlanningRecovery">
              <GoalUpdated/>
              <Action ID="ReconfigurePlannerForECar"/>
              <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
          </RecoveryNode>
        </RateController>
        
        <!-- Path execution with omnidirectional capabilities -->
        <RecoveryNode number_of_retries="3" name="ECarFollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ReactiveFallback name="ExecutionRecovery">
            <GoalUpdated/>
            <Action ID="EnableOmnidirectionalMode"/>
            <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
          </ReactiveFallback>
        </RecoveryNode>
        
      </PipelineSequence>
      
      <!-- eCar-specific recovery behaviors -->
      <ReactiveFallback name="ECarRecoveryBehaviors">
        <GoalUpdated/>
        <RoundRobin name="ECarRecoveryActions">
          
          <!-- Clear sensors and costmaps -->
          <Sequence name="SensorReset">
            <Action ID="ResetLidar"/>
            <Action ID="RecalibrateIMU"/>
            <ClearEntireCostmap name="ClearBothCostmaps" service_name="local_costmap/clear_entirely_local_costmap"/>
          </Sequence>
          
          <!-- Use 4WS capabilities for complex maneuvers -->
          <Sequence name="FourWheelSteeringRecovery">
            <Action ID="Enable4WSMode"/>
            <Action ID="SidestepObstacle"/>
            <Action ID="Disable4WSMode"/>
          </Sequence>
          
          <!-- Traditional recovery behaviors -->
          <Spin spin_dist="1.57"/>
          <Wait wait_duration="3"/>
          
          <!-- eCar can move sideways for recovery -->
          <Action ID="LateralRecovery"/>
          
          <!-- Last resort backup -->
          <BackUp backup_dist="0.50" backup_speed="0.1"/>
          
        </RoundRobin>
      </ReactiveFallback>
      
    </RecoveryNode>
    
  </BehaviorTree>
</root>
```

### Nodos BT Personalizados

```cpp
// include/tadeo_ecar_navigation/bt_nodes/is_battery_ok.hpp
#ifndef TADEO_ECAR_NAVIGATION__BT_NODES__IS_BATTERY_OK_HPP_
#define TADEO_ECAR_NAVIGATION__BT_NODES__IS_BATTERY_OK_HPP_

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace tadeo_ecar_navigation
{
class IsBatteryOK : public BT::ConditionNode
{
public:
  IsBatteryOK(const std::string & condition_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery_percentage", 20.0, "Minimum battery percentage required")
    };
  }

private:
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  sensor_msgs::msg::BatteryState::SharedPtr last_battery_state_;
  double min_battery_percentage_;
};
}  // namespace tadeo_ecar_navigation

#endif  // TADEO_ECAR_NAVIGATION__BT_NODES__IS_BATTERY_OK_HPP_
```

```cpp
// src/bt_nodes/is_battery_ok.cpp
#include "tadeo_ecar_navigation/bt_nodes/is_battery_ok.hpp"

namespace tadeo_ecar_navigation
{
IsBatteryOK::IsBatteryOK(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = rclcpp::Node::make_shared("is_battery_ok_bt_node");
  
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::SensorDataQoS(),
    std::bind(&IsBatteryOK::batteryCallback, this, std::placeholders::_1));
}

BT::NodeStatus IsBatteryOK::tick()
{
  getInput("min_battery_percentage", min_battery_percentage_);
  
  if (!last_battery_state_) {
    RCLCPP_WARN(node_->get_logger(), "No battery state received yet");
    return BT::NodeStatus::FAILURE;
  }
  
  // Check if battery percentage is above minimum
  if (last_battery_state_->percentage >= min_battery_percentage_) {
    RCLCPP_DEBUG(
      node_->get_logger(), 
      "Battery OK: %.1f%% >= %.1f%%",
      last_battery_state_->percentage, min_battery_percentage_);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "Battery LOW: %.1f%% < %.1f%%",
      last_battery_state_->percentage, min_battery_percentage_);
    return BT::NodeStatus::FAILURE;
  }
}

void IsBatteryOK::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  last_battery_state_ = msg;
}
}  // namespace tadeo_ecar_navigation
```

## Implementación en el eCar

### Launch File Completo

```python
# launch/navigation.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get directories
    bringup_dir = FindPackageShare('nav2_bringup')
    ecar_nav_dir = FindPackageShare('tadeo_ecar_navigation')
    
    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([ecar_nav_dir, 'config', 'nav2_params.yaml']),
        description='Full path to the ROS2 parameters file for the eCar navigator')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=PathJoinSubstitution([ecar_nav_dir, 'behavior_trees', 'ecar_navigation.xml']),
        description='Full path to the behavior tree xml file for eCar navigation')

    declare_use_lifecycle_mgr_cmd = DeclareLaunchArgument(
        'use_lifecycle_mgr', default_value='true', description='Use lifecycle manager')

    declare_map_subscribe_transient_local_cmd = DeclareLaunchArgument(
        'map_subscribe_transient_local', default_value='false',
        description='Whether to set the map subscriber QoS to transient local')

    # Variables
    lifecycle_nodes = [
        'controller_server',
        'planner_server', 
        'recoveries_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'bt_xml_filename': bt_xml_file,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    # Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_dir, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params,
            'use_lifecycle_mgr': use_lifecycle_mgr,
            'map_subscribe_transient_local': map_subscribe_transient_local
        }.items()
    )

    # eCar-specific navigation nodes
    ecar_nav_monitor_node = Node(
        package='tadeo_ecar_navigation',
        executable='navigation_monitor_node',
        name='navigation_monitor',
        namespace=namespace,
        parameters=[configured_params],
        output='screen'
    )

    ecar_path_smoother_node = Node(
        package='tadeo_ecar_navigation',
        executable='path_smoother_node', 
        name='path_smoother',
        namespace=namespace,
        parameters=[configured_params],
        output='screen'
    )

    return LaunchDescription([
        # Declare arguments
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_params_file_cmd,
        declare_bt_xml_cmd,
        declare_use_lifecycle_mgr_cmd,
        declare_map_subscribe_transient_local_cmd,

        # Launch Nav2
        nav2_bringup_launch,

        # eCar-specific nodes
        ecar_nav_monitor_node,
        ecar_path_smoother_node
    ])
```

### Configuración de Parámetros Completa

```yaml
# config/nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "omnidirectional"  # Importante para eCar 4WD4WS
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    navigators: ['navigate_to_pose', 'navigate_through_poses']
    navigate_to_pose:
      plugin: "nav2_bt_navigator/NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator/NavigateThroughPosesNavigator"
    # Plugin para BT personalizado del eCar
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - tadeo_ecar_navigation_bt_nodes  # Nodos personalizados eCar

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # Progress checker
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25

    # Controller plugin eCar personalizado
    FollowPath:
      plugin: "tadeo_ecar_navigation::ECarController"
      max_linear_vel_x: 2.0
      max_linear_vel_y: 1.0
      max_angular_vel: 1.0
      kp_linear: 1.0
      kp_angular: 2.0
      kp_lateral: 1.5
      lookahead_distance: 1.0
      goal_tolerance: 0.2

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      footprint: "[[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]]"
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      footprint: "[[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]]"
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 10.0
          raytrace_min_range: 0.0
          obstacle_max_range: 8.0
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

# Planner personalizado para eCar
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid"
      downsample_costmap: false
      downsampling_factor: 1
      tolerance: 0.5
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 5.0
      motion_model_for_search: "REEDS_SHEPP"  # Cambiado para eCar omnidireccional
      angle_quantization_bins: 72
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 3.0
      minimum_turning_radius: 0.8  # Radio mínimo específico para eCar
      reverse_penalty: 2.0
      change_penalty: 0.0
      non_straight_penalty: 1.2
      cost_penalty: 2.0
      retrospective_penalty: 0.015
      lookup_table_size: 20.0
      cache_obstacle_heuristic: false
      debug_visualizations: false
      use_quadratic_cost_penalty: False
      downsample_obstacle_heuristic: True

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: False
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2

robot_state_publisher:
  ros__parameters:
    use_sim_time: False

waypoint_follower:
  ros__parameters:
    use_sim_time: False
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200
```

## Configuración Avanzada

### Configuración Dinámica

```python
#!/usr/bin/env python3
# scripts/dynamic_nav_config.py

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
import json

class DynamicNavConfig(Node):
    def __init__(self):
        super().__init__('dynamic_nav_config')
        
        # Subscribe to environment changes
        self.env_sub = self.create_subscription(
            String, 'environment_type', self.environment_callback, 10)
        
        # Subscribe to speed limit changes
        self.speed_sub = self.create_subscription(
            String, 'speed_limit_mode', self.speed_callback, 10)
        
        self.get_logger().info('Dynamic Nav Config node initialized')
    
    def environment_callback(self, msg):
        """Adjust navigation parameters based on environment"""
        env_type = msg.data
        
        if env_type == "indoor":
            self.configure_indoor_navigation()
        elif env_type == "outdoor":
            self.configure_outdoor_navigation()
        elif env_type == "warehouse":
            self.configure_warehouse_navigation()
        
        self.get_logger().info(f'Configured navigation for {env_type} environment')
    
    def speed_callback(self, msg):
        """Adjust speed limits based on mode"""
        mode = msg.data
        
        if mode == "slow":
            max_vel_x = 0.5
            max_vel_y = 0.3
        elif mode == "normal":
            max_vel_x = 1.5
            max_vel_y = 0.8
        elif mode == "fast":
            max_vel_x = 2.0
            max_vel_y = 1.0
        else:
            return
        
        # Update controller parameters
        self.set_controller_velocities(max_vel_x, max_vel_y)
        
        self.get_logger().info(f'Set speed mode to {mode}')
    
    def configure_indoor_navigation(self):
        """Indoor navigation configuration"""
        params = {
            'controller_server.FollowPath.max_linear_vel_x': 1.0,
            'controller_server.FollowPath.max_linear_vel_y': 0.5,
            'controller_server.FollowPath.lookahead_distance': 0.8,
            'local_costmap.local_costmap.inflation_layer.inflation_radius': 0.4,
            'planner_server.GridBased.minimum_turning_radius': 0.5
        }
        
        self.update_parameters(params)
    
    def configure_outdoor_navigation(self):
        """Outdoor navigation configuration"""
        params = {
            'controller_server.FollowPath.max_linear_vel_x': 2.0,
            'controller_server.FollowPath.max_linear_vel_y': 1.0,
            'controller_server.FollowPath.lookahead_distance': 1.5,
            'local_costmap.local_costmap.inflation_layer.inflation_radius': 0.6,
            'planner_server.GridBased.minimum_turning_radius': 0.8
        }
        
        self.update_parameters(params)
    
    def configure_warehouse_navigation(self):
        """Warehouse navigation configuration"""
        params = {
            'controller_server.FollowPath.max_linear_vel_x': 1.2,
            'controller_server.FollowPath.max_linear_vel_y': 0.8,
            'controller_server.FollowPath.lookahead_distance': 1.0,
            'local_costmap.local_costmap.inflation_layer.inflation_radius': 0.5,
            'planner_server.GridBased.minimum_turning_radius': 0.6,
            'controller_server.FollowPath.kp_lateral': 2.0  # Más agresivo lateral
        }
        
        self.update_parameters(params)
    
    def set_controller_velocities(self, max_vel_x, max_vel_y):
        """Update controller velocity limits"""
        params = {
            'controller_server.FollowPath.max_linear_vel_x': max_vel_x,
            'controller_server.FollowPath.max_linear_vel_y': max_vel_y
        }
        
        self.update_parameters(params)
    
    def update_parameters(self, param_dict):
        """Update parameters on respective nodes"""
        # In a real implementation, you would use parameter services
        # to update parameters on running nodes
        for param_name, value in param_dict.items():
            self.get_logger().info(f'Would set {param_name} = {value}')
            # Example: 
            # self.set_parameters_service.call_async(param_name, value)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicNavConfig()
    
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

### Monitor de Navegación

```cpp
// include/tadeo_ecar_navigation/navigation_monitor.hpp
#ifndef TADEO_ECAR_NAVIGATION__NAVIGATION_MONITOR_HPP_
#define TADEO_ECAR_NAVIGATION__NAVIGATION_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

namespace tadeo_ecar_navigation
{
class NavigationMonitor : public rclcpp::Node
{
public:
  NavigationMonitor();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  void monitorProgress();
  void checkNavigationHealth();
  void publishNavigationStatus();
  
  double calculatePathProgress();
  double calculateDistanceToGoal();
  bool isRobotStuck();
  
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stuck_detection_pub_;
  
  // Timer
  rclcpp::TimerInterface::SharedPtr monitor_timer_;
  
  // State
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  nav_msgs::msg::Path::SharedPtr current_global_path_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_goal_;
  
  // Previous state for stuck detection
  geometry_msgs::msg::Point last_position_;
  rclcpp::Time last_movement_time_;
  
  // Parameters
  double stuck_timeout_;
  double stuck_threshold_;
  double goal_tolerance_;
  double monitor_frequency_;
};
}  // namespace tadeo_ecar_navigation

#endif  // TADEO_ECAR_NAVIGATION__NAVIGATION_MONITOR_HPP_
```

### Scripts de Testing

```bash
#!/bin/bash
# scripts/test_navigation.sh

echo "=== eCar Navigation Testing Script ==="

# Test 1: Basic navigation system startup
echo "1. Testing navigation system startup..."
timeout 10s ros2 launch tadeo_ecar_navigation navigation.launch.py &
NAV_PID=$!
sleep 5

if ps -p $NAV_PID > /dev/null; then
    echo "✓ Navigation system started successfully"
    kill $NAV_PID
else
    echo "✗ Navigation system failed to start"
    exit 1
fi

# Test 2: Check all required nodes
echo "2. Checking required navigation nodes..."
ros2 launch tadeo_ecar_navigation navigation.launch.py &
NAV_PID=$!
sleep 8

REQUIRED_NODES=(
    "/bt_navigator"
    "/controller_server"
    "/planner_server"
    "/recoveries_server"
)

for node in "${REQUIRED_NODES[@]}"; do
    if ros2 node list | grep -q "$node"; then
        echo "✓ $node is running"
    else
        echo "✗ $node is not running"
        kill $NAV_PID
        exit 1
    fi
done

# Test 3: Test basic navigation goal
echo "3. Testing basic navigation goal..."
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "map", stamp: {sec: 0, nanosec: 0}},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'

sleep 3
echo "✓ Navigation goal published successfully"

# Test 4: Check costmap topics
echo "4. Checking costmap topics..."
COSTMAP_TOPICS=(
    "/local_costmap/costmap"
    "/global_costmap/costmap"
)

for topic in "${COSTMAP_TOPICS[@]}"; do
    if timeout 5s ros2 topic echo $topic --once > /dev/null 2>&1; then
        echo "✓ $topic is publishing"
    else
        echo "✗ $topic is not publishing"
    fi
done

# Cleanup
kill $NAV_PID
echo "=== Navigation Testing Complete ==="
```

## Ejercicios Prácticos

### Ejercicio 1: Configurar Navegación Básica

```bash
# 1. Compilar paquete de navegación
colcon build --packages-select tadeo_ecar_navigation

# 2. Lanzar sistema de navegación
ros2 launch tadeo_ecar_navigation navigation.launch.py

# 3. En RViz, publicar goal pose
# 4. Observar comportamiento de navegación
# 5. Probar diferentes goals
```

### Ejercicio 2: Personalizar Costmap

```yaml
# Crear config/custom_costmap.yaml
# TODO: Configurar inflation_radius específico para eCar
# TODO: Ajustar observation_sources
# TODO: Configurar footprint del robot
# TODO: Probar con diferentes configuraciones
```

### Ejercicio 3: Implementar Recovery Behavior

```cpp
// TODO: Implementar ECarSidestepRecovery
class ECarSidestepRecovery : public nav2_core::Recovery
{
public:
    // TODO: Configurar recovery behavior que use movimiento lateral
    // TODO: Implementar lógica de sidestep usando 4WS
    // TODO: Integrar con behavior tree
};
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Nav2 Stack**: Sistema completo de navegación autónoma
2. **Costmaps**: Representación del mundo para planificación
3. **Planificadores**: Algoritmos de búsqueda de rutas
4. **Controladores**: Seguimiento de trayectorias y evitación
5. **Behavior Trees**: Coordinación y recovery behaviors
6. **Configuración eCar**: Adaptación para robot 4WD4WS
7. **Monitoreo**: Supervisión de la navegación
8. **Testing**: Verificación del sistema completo

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes la arquitectura del stack Nav2
- [ ] Puedes configurar costmaps para el eCar
- [ ] Sabes implementar controladores personalizados
- [ ] Comprendes el uso de behavior trees
- [ ] Puedes configurar navegación omnidireccional
- [ ] Has probado el sistema de navegación completo
- [ ] Sabes debuggear problemas de navegación

### Próximo Capítulo

En el Capítulo 11 estudiaremos:
- SLAM (Simultaneous Localization and Mapping)
- Algoritmos de mapeo
- Localización en tiempo real
- Integración con navegación
- Implementación específica para eCar

## Referencias

- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Concepts](https://navigation.ros.org/concepts/index.html)
- [Behavior Trees](https://www.behaviortree.dev/)
- [Costmap 2D](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)
- [DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)