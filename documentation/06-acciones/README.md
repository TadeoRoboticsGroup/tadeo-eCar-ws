# Capítulo 6: Acciones en ROS2

## Tabla de Contenidos

1. [Concepto de Acciones](#concepto-de-acciones)
2. [Goals, Feedback y Results](#goals-feedback-y-results)
3. [Definición de Acciones](#definición-de-acciones)
4. [Implementación en C++](#implementación-en-c++)
5. [Implementación en Python](#implementación-en-python)
6. [Acciones del eCar](#acciones-del-ecar)
7. [Cancelación de Acciones](#cancelación-de-acciones)
8. [Múltiples Goals](#múltiples-goals)

## Concepto de Acciones

### ¿Qué son las Acciones?

Las acciones en ROS2 proporcionan comunicación **asíncrona** para tareas de larga duración que requieren feedback periódico y posibilidad de cancelación.

```
Cliente                         Servidor
┌──────────────┐               ┌──────────────┐
│ Navigation   │ ──── Goal ──→ │ Move to Goal │
│ Node         │               │ Action       │
│              │ ←── Feedback │              │
│              │ ←─ Feedback ─ │              │
│              │ ←── Result ─── │              │
└──────────────┘               └──────────────┘
```

### Diferencias: Tópicos vs Servicios vs Acciones

```cpp
// TÓPICOS - Streaming continuo
publisher->publish(cmd_vel_msg);  // Fire and forget
// ✓ Flujo continuo  ✗ Sin confirmación  ✗ Sin progreso

// SERVICIOS - Request-Response síncrono
auto response = client->send_request(request);  // Bloquea hasta respuesta
// ✓ Confirmación  ✗ Bloquea ejecución  ✗ Sin progreso

// ACCIONES - Tareas largas asíncronas
client->send_goal(goal);  // No bloquea
// callback_feedback(progress);  // Progreso periódico
// callback_result(final_result);  // Resultado final
// ✓ Asíncrono  ✓ Feedback  ✓ Cancelable
```

### Casos de Uso en el eCar

**Usar Acciones para:**
- Navegación a un punto objetivo (puede tomar minutos)
- Mapeo con SLAM (proceso continuo con progreso)
- Secuencias de maniobras complejas (estacionamiento)
- Carga de batería (monitoreo de progreso)
- Calibración completa del sistema

**No usar Acciones para:**
- Control de velocidad en tiempo real (usar tópicos)
- Consulta rápida de estado (usar servicios)
- Configuración de parámetros (usar servicios)

### Arquitectura de Acciones

Las acciones internamente usan **tópicos y servicios**:

```bash
# Una acción /navigate_to_pose genera:
/navigate_to_pose/_action/send_goal         # Servicio para enviar goal
/navigate_to_pose/_action/cancel_goal       # Servicio para cancelar
/navigate_to_pose/_action/get_result        # Servicio para obtener resultado
/navigate_to_pose/_action/feedback          # Tópico para feedback
/navigate_to_pose/_action/status           # Tópico para estado de goals
```

## Goals, Feedback y Results

### Estructura de una Acción

```
# Ejemplo: NavigateToGoal.action

# GOAL - Lo que queremos lograr
geometry_msgs/PoseStamped target_pose
float32 tolerance
bool use_obstacle_avoidance
string behavior_tree
---
# RESULT - Resultado final cuando termina
bool success
string message
geometry_msgs/PoseStamped final_pose
float32 final_distance_error
float32 navigation_time
---
# FEEDBACK - Progreso durante ejecución
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 eta_seconds
string current_behavior
```

### Estados de una Acción

```cpp
// Estados del goal
enum GoalStatus {
    STATUS_UNKNOWN = 0,
    STATUS_ACCEPTED = 1,      // Goal aceptado
    STATUS_EXECUTING = 2,     // Ejecutándose
    STATUS_CANCELING = 3,     // Cancelándose
    STATUS_SUCCEEDED = 4,     // Completado exitosamente
    STATUS_CANCELED = 5,      // Cancelado
    STATUS_ABORTED = 6       // Falló durante ejecución
};
```

### Flujo Típico de una Acción

```
1. Cliente envía GOAL
2. Servidor acepta/rechaza GOAL
3. Si aceptado → STATUS_EXECUTING
4. Servidor envía FEEDBACK periódico
5. Cliente puede CANCELAR en cualquier momento
6. Servidor envía RESULT final
7. Estado final: SUCCEEDED/CANCELED/ABORTED
```

### Ejemplo: Navegación del eCar

```python
# Cliente envía goal de navegación
goal = NavigateToGoal.Goal()
goal.target_pose.pose.position.x = 5.0
goal.target_pose.pose.position.y = 3.0
goal.tolerance = 0.2
goal.use_obstacle_avoidance = True

# Durante ejecución, recibe feedback
def feedback_callback(feedback):
    print(f"Distance remaining: {feedback.distance_remaining:.2f}m")
    print(f"ETA: {feedback.eta_seconds:.0f} seconds")
    print(f"Current behavior: {feedback.current_behavior}")

# Al finalizar, recibe resultado
def result_callback(result):
    if result.success:
        print(f"Navigation successful in {result.navigation_time:.1f}s")
    else:
        print(f"Navigation failed: {result.message}")
```

## Definición de Acciones

### Acciones Estándar

ROS2 incluye acciones comunes:

```cpp
// Navegación
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>

// Manipulación
#include <control_msgs/action/follow_joint_trajectory.hpp>

// Genéricas
#include <example_interfaces/action/fibonacci.hpp>
```

### Acciones Personalizadas para el eCar

**1. Acción de Maniobra Compleja**
```
# action/PerformManeuver.action
# GOAL
string maneuver_type          # "park_parallel", "park_perpendicular", "three_point_turn"
geometry_msgs/PoseStamped target_pose
float32 space_length         # Para estacionamiento
float32 max_speed           # Velocidad máxima durante maniobra
bool use_sensors            # Usar sensores para ajuste fino
---
# RESULT
bool success
string message
geometry_msgs/PoseStamped final_pose
float32 maneuver_time
float32 total_distance
int32 steering_cycles       # Número de cambios de dirección
---
# FEEDBACK  
string current_phase        # "approaching", "aligning", "reversing", "adjusting"
float32 progress_percentage # 0.0 - 100.0
geometry_msgs/PoseStamped current_pose
float32 distance_to_target
float32 current_speed
bool sensors_detecting_obstacle
```

**2. Acción de Mapeo SLAM**
```
# action/CreateMap.action
# GOAL
string map_name
geometry_msgs/Polygon exploration_area  # Área a mapear
float32 resolution          # Resolución del mapa (m/pixel)
float32 max_exploration_time # Tiempo máximo de exploración
bool save_map_automatically # Guardar automáticamente al completar
---
# RESULT
bool success
string message
string map_file_path
float32 mapping_time
float32 area_covered        # m²
int32 loop_closures         # Número de loop closures detectados
float32 map_quality_score   # 0.0 - 1.0
---
# FEEDBACK
float32 progress_percentage
float32 area_covered_so_far
geometry_msgs/PoseStamped current_pose
int32 landmarks_detected
float32 mapping_confidence  # Confianza en la calidad del mapa actual
string current_activity     # "exploring", "loop_closing", "optimizing"
```

**3. Acción de Carga de Batería**
```
# action/ChargeBattery.action  
# GOAL
float32 target_charge_percentage  # Porcentaje objetivo (0-100)
bool return_to_dock              # Ir a estación de carga primero
float32 max_charging_time        # Tiempo máximo de carga (segundos)
bool optimize_battery_health     # Optimizar salud de batería vs velocidad
---
# RESULT
bool success
string message
float32 final_charge_percentage
float32 charging_time
float32 energy_consumed          # kWh
bool battery_health_optimal
---
# FEEDBACK
float32 current_charge_percentage
float32 charging_rate           # A (corriente de carga)
float32 estimated_time_remaining # segundos
float32 battery_temperature     # °C
string charging_phase          # "docking", "charging", "balancing", "complete"
bool dock_connected
```

### Configuración en CMakeLists.txt

```cmake
# Agregar al CMakeLists.txt de tadeo_ecar_interfaces
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Declarar acciones
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PerformManeuver.action"
  "action/CreateMap.action" 
  "action/ChargeBattery.action"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)
```

## Implementación en C++

### Servidor de Acción

```cpp
// include/tadeo_ecar_navigation/navigation_action_server.hpp
#ifndef TADEO_ECAR_NAVIGATION__NAVIGATION_ACTION_SERVER_HPP_
#define TADEO_ECAR_NAVIGATION__NAVIGATION_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <thread>

namespace tadeo_ecar_navigation
{
class NavigationActionServer : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    NavigationActionServer();

private:
    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigate> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle);

    // Navigation execution
    void execute_navigation(const std::shared_ptr<GoalHandleNavigate> goal_handle);
    bool is_goal_reached(const geometry_msgs::msg::PoseStamped& target,
                        const geometry_msgs::msg::PoseStamped& current,
                        double tolerance);
    
    void publish_feedback(const std::shared_ptr<GoalHandleNavigate> goal_handle,
                         const geometry_msgs::msg::PoseStamped& current_pose,
                         const geometry_msgs::msg::PoseStamped& target_pose);

    // Subscribers
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Action server
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

    // Publishers/Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // State
    geometry_msgs::msg::PoseStamped current_pose_;
    bool navigation_active_;
    std::mutex navigation_mutex_;

    // Parameters
    double linear_velocity_;
    double angular_velocity_;
    double goal_tolerance_;
    double feedback_rate_;
};
}  // namespace tadeo_ecar_navigation

#endif  // TADEO_ECAR_NAVIGATION__NAVIGATION_ACTION_SERVER_HPP_
```

```cpp
// src/navigation_action_server.cpp
#include "tadeo_ecar_navigation/navigation_action_server.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>

namespace tadeo_ecar_navigation
{
NavigationActionServer::NavigationActionServer()
    : Node("navigation_action_server"),
      navigation_active_(false)
{
    // Declarar parámetros
    this->declare_parameter("linear_velocity", 0.5);
    this->declare_parameter("angular_velocity", 0.3);
    this->declare_parameter("goal_tolerance", 0.2);
    this->declare_parameter("feedback_rate", 2.0);

    // Obtener parámetros
    linear_velocity_ = this->get_parameter("linear_velocity").as_double();
    angular_velocity_ = this->get_parameter("angular_velocity").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    feedback_rate_ = this->get_parameter("feedback_rate").as_double();

    // Configurar TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publishers y Subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&NavigationActionServer::odom_callback, this, std::placeholders::_1));

    // Crear action server
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this,
        "navigate_to_pose",
        std::bind(&NavigationActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NavigationActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&NavigationActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Navigation action server initialized");
    RCLCPP_INFO(this->get_logger(), "Linear velocity: %.2f m/s", linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "Angular velocity: %.2f rad/s", angular_velocity_);
    RCLCPP_INFO(this->get_logger(), "Goal tolerance: %.2f m", goal_tolerance_);
}

rclcpp_action::GoalResponse NavigationActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
{
    (void)uuid;  // Suprimir warning de parámetro no usado

    RCLCPP_INFO(this->get_logger(), 
                "Received navigation goal: x=%.2f, y=%.2f",
                goal->pose.pose.position.x,
                goal->pose.pose.position.y);

    // Validar goal
    if (std::isnan(goal->pose.pose.position.x) || 
        std::isnan(goal->pose.pose.position.y)) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal with NaN values");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Verificar si hay otra navegación activa
    std::lock_guard<std::mutex> lock(navigation_mutex_);
    if (navigation_active_) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: navigation already active");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Accepting navigation goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
    (void)goal_handle;
    
    RCLCPP_INFO(this->get_logger(), "Received request to cancel navigation");
    
    // Detener robot
    auto stop_cmd = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(stop_cmd);
    
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationActionServer::handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
    // Ejecutar navegación en thread separado para no bloquear
    std::thread{std::bind(&NavigationActionServer::execute_navigation, this, goal_handle)}.detach();
}

void NavigationActionServer::execute_navigation(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting navigation execution");
    
    {
        std::lock_guard<std::mutex> lock(navigation_mutex_);
        navigation_active_ = true;
    }

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();
    
    auto start_time = this->get_clock()->now();
    rclcpp::Rate feedback_rate(feedback_rate_);

    // Bucle principal de navegación
    while (rclcpp::ok()) {
        // Verificar si la acción fue cancelada
        if (goal_handle->is_canceling()) {
            // Detener robot
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);
            
            result->result.result = nav2_msgs::msg::NavigationResult::CANCELED;
            goal_handle->canceled(result);
            
            RCLCPP_INFO(this->get_logger(), "Navigation canceled");
            break;
        }

        // Obtener pose actual
        geometry_msgs::msg::PoseStamped current_pose;
        try {
            auto transform = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            current_pose.header.frame_id = "map";
            current_pose.header.stamp = this->get_clock()->now();
            current_pose.pose.position.x = transform.transform.translation.x;
            current_pose.pose.position.y = transform.transform.translation.y;
            current_pose.pose.position.z = transform.transform.translation.z;
            current_pose.pose.orientation = transform.transform.rotation;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            // Usar última pose conocida
            current_pose = current_pose_;
        }

        // Verificar si llegamos al objetivo
        if (is_goal_reached(goal->pose, current_pose, goal_tolerance_)) {
            // Detener robot
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);

            // Resultado exitoso
            auto end_time = this->get_clock()->now();
            result->result.result = nav2_msgs::msg::NavigationResult::SUCCEEDED;
            result->navigation_time = (end_time - start_time).seconds();
            
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), 
                       "Navigation completed successfully in %.2f seconds",
                       result->navigation_time);
            break;
        }

        // Calcular y publicar comando de velocidad
        auto cmd_vel = geometry_msgs::msg::Twist();
        
        // Cálculo simple de control hacia el objetivo
        double dx = goal->pose.pose.position.x - current_pose.pose.position.x;
        double dy = goal->pose.pose.position.y - current_pose.pose.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        double target_yaw = atan2(dy, dx);
        double current_yaw = tf2::getYaw(current_pose.pose.orientation);
        double yaw_error = target_yaw - current_yaw;
        
        // Normalizar ángulo
        while (yaw_error > M_PI) yaw_error -= 2*M_PI;
        while (yaw_error < -M_PI) yaw_error += 2*M_PI;

        // Control proporcional simple
        if (std::abs(yaw_error) > 0.1) {
            // Principalmente girar
            cmd_vel.angular.z = std::copysign(angular_velocity_, yaw_error);
            cmd_vel.linear.x = linear_velocity_ * 0.3; // Avanzar lento mientras gira
        } else {
            // Principalmente avanzar
            cmd_vel.linear.x = std::min(linear_velocity_, distance);
            cmd_vel.angular.z = yaw_error * 0.5; // Corrección menor
        }

        cmd_vel_pub_->publish(cmd_vel);

        // Publicar feedback
        publish_feedback(goal_handle, current_pose, goal->pose);

        // Verificar timeout (ejemplo: 5 minutos)
        auto current_time = this->get_clock()->now();
        if ((current_time - start_time).seconds() > 300.0) {
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);
            
            result->result.result = nav2_msgs::msg::NavigationResult::FAILED;
            goal_handle->abort(result);
            
            RCLCPP_ERROR(this->get_logger(), "Navigation timeout");
            break;
        }

        feedback_rate.sleep();
    }

    {
        std::lock_guard<std::mutex> lock(navigation_mutex_);
        navigation_active_ = false;
    }
}

bool NavigationActionServer::is_goal_reached(
    const geometry_msgs::msg::PoseStamped& target,
    const geometry_msgs::msg::PoseStamped& current,
    double tolerance)
{
    double dx = target.pose.position.x - current.pose.position.x;
    double dy = target.pose.position.y - current.pose.position.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    return distance <= tolerance;
}

void NavigationActionServer::publish_feedback(
    const std::shared_ptr<GoalHandleNavigate> goal_handle,
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::PoseStamped& target_pose)
{
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    
    feedback->current_pose = current_pose;
    
    // Calcular distancia restante
    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    feedback->distance_remaining = sqrt(dx*dx + dy*dy);
    
    // Estimar tiempo restante (simplificado)
    feedback->estimated_time_remaining = rclcpp::Duration::from_seconds(
        feedback->distance_remaining / linear_velocity_);
    
    // Velocidad actual
    feedback->speed = linear_velocity_;
    
    goal_handle->publish_feedback(feedback);
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Feedback: distance=%.2fm, eta=%.1fs",
                feedback->distance_remaining,
                feedback->estimated_time_remaining.seconds());
}

void NavigationActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Actualizar pose actual
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
}
}  // namespace tadeo_ecar_navigation

// Main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_navigation::NavigationActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Cliente de Acción

```cpp
// include/tadeo_ecar_navigation/navigation_action_client.hpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace tadeo_ecar_navigation
{
class NavigationActionClient : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationActionClient();

    bool navigate_to_pose(double x, double y, double yaw = 0.0, double tolerance = 0.2);
    void cancel_navigation();

private:
    void goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleNavigate::SharedPtr,
                          const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandleNavigate::WrappedResult & result);

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    GoalHandleNavigate::SharedPtr current_goal_handle_;
    bool navigation_result_received_;
    bool navigation_successful_;
};
}  // namespace tadeo_ecar_navigation
```

```cpp
// src/navigation_action_client.cpp
#include "tadeo_ecar_navigation/navigation_action_client.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tadeo_ecar_navigation
{
NavigationActionClient::NavigationActionClient()
    : Node("navigation_action_client"),
      navigation_result_received_(false),
      navigation_successful_(false)
{
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "Navigation action client initialized");
}

bool NavigationActionClient::navigate_to_pose(double x, double y, double yaw, double tolerance)
{
    // Esperar a que el servidor esté disponible
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Navigation action server not available");
        return false;
    }

    // Crear goal
    auto goal_msg = NavigateToPose::Goal();
    
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    // Convertir yaw a quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);
    
    goal_msg.tolerance = tolerance;

    RCLCPP_INFO(this->get_logger(), 
                "Sending navigation goal: x=%.2f, y=%.2f, yaw=%.2f",
                x, y, yaw);

    // Configurar opciones de envío
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        std::bind(&NavigationActionClient::goal_response_callback, this, std::placeholders::_1);
    
    send_goal_options.feedback_callback =
        std::bind(&NavigationActionClient::feedback_callback, this, 
                 std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback =
        std::bind(&NavigationActionClient::result_callback, this, std::placeholders::_1);

    // Enviar goal
    navigation_result_received_ = false;
    current_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    // Esperar resultado
    while (!navigation_result_received_ && rclcpp::ok()) {
        rclcpp::spin_some(this->shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return navigation_successful_;
}

void NavigationActionClient::cancel_navigation()
{
    if (current_goal_handle_) {
        RCLCPP_INFO(this->get_logger(), "Canceling navigation");
        action_client_->async_cancel_goal(current_goal_handle_);
    } else {
        RCLCPP_WARN(this->get_logger(), "No active navigation to cancel");
    }
}

void NavigationActionClient::goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Navigation goal was rejected by server");
        navigation_result_received_ = true;
        navigation_successful_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Navigation goal accepted by server, waiting for result");
    }
}

void NavigationActionClient::feedback_callback(
    GoalHandleNavigate::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(),
                "Navigation feedback: distance=%.2fm, eta=%.1fs, speed=%.2fm/s",
                feedback->distance_remaining,
                feedback->estimated_time_remaining.seconds(),
                feedback->speed);
}

void NavigationActionClient::result_callback(const GoalHandleNavigate::WrappedResult & result)
{
    navigation_result_received_ = true;
    current_goal_handle_.reset();

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            navigation_successful_ = true;
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            navigation_successful_ = false;
            RCLCPP_ERROR(this->get_logger(), "Navigation was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            navigation_successful_ = false;
            RCLCPP_INFO(this->get_logger(), "Navigation was canceled");
            break;
        default:
            navigation_successful_ = false;
            RCLCPP_ERROR(this->get_logger(), "Unknown navigation result code");
            break;
    }

    // Información del resultado
    if (result.result) {
        RCLCPP_INFO(this->get_logger(), 
                    "Navigation time: %.2f seconds",
                    result.result->navigation_time);
    }
}
}  // namespace tadeo_ecar_navigation

// Example usage
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<tadeo_ecar_navigation::NavigationActionClient>();

    // Navegar a punto específico
    if (client->navigate_to_pose(5.0, 3.0, 1.57, 0.2)) {
        RCLCPP_INFO(client->get_logger(), "Navigation completed successfully");
    } else {
        RCLCPP_ERROR(client->get_logger(), "Navigation failed");
    }

    rclcpp::shutdown();
    return 0;
}
```

## Implementación en Python

### Servidor de Acción en Python

```python
#!/usr/bin/env python3
# scripts/maneuver_action_server.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
import time
import math

from tadeo_ecar_interfaces.action import PerformManeuver
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs

class ManeuverActionServer(Node):
    def __init__(self):
        super().__init__('maneuver_action_server')
        
        # Action server
        self._action_server = ActionServer(
            self,
            PerformManeuver,
            'perform_maneuver',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        # Publishers y subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Estado
        self.current_pose = None
        self.maneuver_active = False
        
        # Parámetros
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('position_tolerance', 0.1)
        
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        
        self.get_logger().info('Maneuver action server initialized')
    
    def goal_callback(self, goal_request):
        """Callback para validar y aceptar/rechazar goals"""
        self.get_logger().info(f'Received maneuver goal: {goal_request.maneuver_type}')
        
        # Validar tipo de maniobra
        valid_maneuvers = ['park_parallel', 'park_perpendicular', 'three_point_turn']
        if goal_request.maneuver_type not in valid_maneuvers:
            self.get_logger().warn(f'Invalid maneuver type: {goal_request.maneuver_type}')
            return GoalResponse.REJECT
        
        # Verificar si hay maniobra activa
        if self.maneuver_active:
            self.get_logger().warn('Rejecting goal: maneuver already active')
            return GoalResponse.REJECT
        
        # Validar velocidad máxima
        if goal_request.max_speed <= 0 or goal_request.max_speed > self.max_linear_vel:
            self.get_logger().warn(f'Invalid max speed: {goal_request.max_speed}')
            return GoalResponse.REJECT
        
        self.get_logger().info('Accepting maneuver goal')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Callback para cancelar maniobra"""
        self.get_logger().info('Received cancel request')
        
        # Detener robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Ejecutar maniobra"""
        self.get_logger().info('Executing maneuver...')
        self.maneuver_active = True
        
        goal = goal_handle.request
        feedback_msg = PerformManeuver.Feedback()
        result = PerformManeuver.Result()
        
        start_time = time.time()
        
        try:
            # Ejecutar maniobra según tipo
            if goal.maneuver_type == 'park_parallel':
                success = await self.execute_parallel_parking(goal_handle, goal, feedback_msg)
            elif goal.maneuver_type == 'park_perpendicular':
                success = await self.execute_perpendicular_parking(goal_handle, goal, feedback_msg)
            elif goal.maneuver_type == 'three_point_turn':
                success = await self.execute_three_point_turn(goal_handle, goal, feedback_msg)
            else:
                success = False
            
            # Resultado final
            end_time = time.time()
            result.success = success
            result.maneuver_time = end_time - start_time
            
            if success:
                result.message = f'Maneuver {goal.maneuver_type} completed successfully'
                result.final_pose = self.get_current_pose()
                goal_handle.succeed()
            else:
                result.message = f'Maneuver {goal.maneuver_type} failed'
                goal_handle.abort()
            
        except Exception as e:
            self.get_logger().error(f'Exception during maneuver: {e}')
            result.success = False
            result.message = f'Maneuver failed with exception: {str(e)}'
            goal_handle.abort()
        
        finally:
            # Detener robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            self.maneuver_active = False
        
        return result
    
    async def execute_parallel_parking(self, goal_handle, goal, feedback_msg):
        """Ejecutar estacionamiento en paralelo"""
        self.get_logger().info('Starting parallel parking maneuver')
        
        phases = [
            'approaching_space',
            'aligning_with_space', 
            'reversing_into_space',
            'adjusting_position'
        ]
        
        total_phases = len(phases)
        
        for i, phase in enumerate(phases):
            if goal_handle.is_cancel_requested:
                return False
            
            feedback_msg.current_phase = phase
            feedback_msg.progress_percentage = (i / total_phases) * 100.0
            feedback_msg.current_pose = self.get_current_pose()
            
            self.get_logger().info(f'Parallel parking phase: {phase}')
            goal_handle.publish_feedback(feedback_msg)
            
            # Simular fase de la maniobra
            if phase == 'approaching_space':
                await self.move_forward(goal.max_speed * 0.5, 2.0)  # 2 segundos
            elif phase == 'aligning_with_space':
                await self.turn_in_place(math.pi/4, goal.max_speed)  # 45 grados
            elif phase == 'reversing_into_space':
                await self.move_backward(goal.max_speed * 0.3, 3.0)  # 3 segundos
            elif phase == 'adjusting_position':
                await self.turn_in_place(-math.pi/6, goal.max_speed * 0.5)  # Ajuste final
            
            # Actualizar progreso
            feedback_msg.progress_percentage = ((i + 1) / total_phases) * 100.0
            goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info('Parallel parking completed')
        return True
    
    async def execute_perpendicular_parking(self, goal_handle, goal, feedback_msg):
        """Ejecutar estacionamiento perpendicular"""
        self.get_logger().info('Starting perpendicular parking maneuver')
        
        phases = [
            'approaching_space',
            'positioning_for_turn',
            'turning_into_space',
            'centering_in_space'
        ]
        
        total_phases = len(phases)
        
        for i, phase in enumerate(phases):
            if goal_handle.is_cancel_requested:
                return False
            
            feedback_msg.current_phase = phase
            feedback_msg.progress_percentage = (i / total_phases) * 100.0
            feedback_msg.current_pose = self.get_current_pose()
            
            self.get_logger().info(f'Perpendicular parking phase: {phase}')
            goal_handle.publish_feedback(feedback_msg)
            
            # Simular fase de la maniobra
            if phase == 'approaching_space':
                await self.move_forward(goal.max_speed * 0.6, 1.5)
            elif phase == 'positioning_for_turn':
                await self.move_forward(goal.max_speed * 0.3, 1.0)
            elif phase == 'turning_into_space':
                await self.turn_and_move(math.pi/2, goal.max_speed * 0.4, 2.0)
            elif phase == 'centering_in_space':
                await self.move_forward(goal.max_speed * 0.2, 0.5)
            
            feedback_msg.progress_percentage = ((i + 1) / total_phases) * 100.0
            goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info('Perpendicular parking completed')
        return True
    
    async def execute_three_point_turn(self, goal_handle, goal, feedback_msg):
        """Ejecutar giro de tres puntos"""
        self.get_logger().info('Starting three-point turn maneuver')
        
        phases = [
            'turn_right_forward',
            'reverse_turn_left', 
            'forward_complete_turn'
        ]
        
        for i, phase in enumerate(phases):
            if goal_handle.is_cancel_requested:
                return False
            
            feedback_msg.current_phase = phase
            feedback_msg.progress_percentage = (i / 3) * 100.0
            feedback_msg.current_pose = self.get_current_pose()
            feedback_msg.steering_cycles = i + 1
            
            self.get_logger().info(f'Three-point turn phase: {phase}')
            goal_handle.publish_feedback(feedback_msg)
            
            if phase == 'turn_right_forward':
                await self.turn_and_move(math.pi/3, goal.max_speed * 0.4, 2.0)
            elif phase == 'reverse_turn_left':
                await self.turn_and_move(-math.pi/3, -goal.max_speed * 0.3, 2.0)
            elif phase == 'forward_complete_turn':
                await self.turn_and_move(math.pi/6, goal.max_speed * 0.4, 1.5)
            
            feedback_msg.progress_percentage = ((i + 1) / 3) * 100.0
            goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info('Three-point turn completed')
        return True
    
    async def move_forward(self, speed, duration):
        """Mover hacia adelante por tiempo específico"""
        cmd = Twist()
        cmd.linear.x = speed
        
        end_time = time.time() + duration
        rate = self.create_rate(10)  # 10 Hz
        
        while time.time() < end_time:
            self.cmd_vel_pub.publish(cmd)
            await rate.sleep()
    
    async def move_backward(self, speed, duration):
        """Mover hacia atrás por tiempo específico"""
        cmd = Twist()
        cmd.linear.x = -speed
        
        end_time = time.time() + duration
        rate = self.create_rate(10)
        
        while time.time() < end_time:
            self.cmd_vel_pub.publish(cmd)
            await rate.sleep()
    
    async def turn_in_place(self, angle, speed):
        """Girar en el lugar por ángulo específico"""
        cmd = Twist()
        cmd.angular.z = speed if angle > 0 else -speed
        
        # Calcular tiempo basado en velocidad angular
        duration = abs(angle) / speed
        end_time = time.time() + duration
        rate = self.create_rate(10)
        
        while time.time() < end_time:
            self.cmd_vel_pub.publish(cmd)
            await rate.sleep()
    
    async def turn_and_move(self, turn_angle, linear_speed, duration):
        """Girar y mover simultáneamente"""
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = turn_angle / duration  # Velocidad angular para completar giro
        
        end_time = time.time() + duration
        rate = self.create_rate(10)
        
        while time.time() < end_time:
            self.cmd_vel_pub.publish(cmd)
            await rate.sleep()
    
    def get_current_pose(self):
        """Obtener pose actual del robot"""
        if self.current_pose is not None:
            return self.current_pose
        
        # Crear pose por defecto si no tenemos odometría
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        return pose
    
    def odom_callback(self, msg):
        """Callback de odometría"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.current_pose = pose

def main(args=None):
    rclpy.init(args=args)
    
    maneuver_server = ManeuverActionServer()
    
    # Usar MultiThreadedExecutor para manejar callbacks async
    executor = MultiThreadedExecutor()
    executor.add_node(maneuver_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        maneuver_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Cliente de Acción en Python

```python
#!/usr/bin/env python3
# scripts/maneuver_action_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from tadeo_ecar_interfaces.action import PerformManeuver
from geometry_msgs.msg import PoseStamped

class ManeuverActionClient(Node):
    def __init__(self):
        super().__init__('maneuver_action_client')
        
        self._action_client = ActionClient(self, PerformManeuver, 'perform_maneuver')
        
        self.get_logger().info('Maneuver action client initialized')
    
    def send_goal(self, maneuver_type, target_pose, space_length=0.0, max_speed=0.3):
        """Enviar goal de maniobra"""
        
        # Esperar servidor
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return False
        
        # Crear goal
        goal_msg = PerformManeuver.Goal()
        goal_msg.maneuver_type = maneuver_type
        goal_msg.target_pose = target_pose
        goal_msg.space_length = space_length
        goal_msg.max_speed = max_speed
        goal_msg.use_sensors = True
        
        self.get_logger().info(f'Sending {maneuver_type} maneuver goal...')
        
        # Enviar goal con callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def goal_response_callback(self, future):
        """Callback cuando el servidor responde al goal"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        
        # Obtener resultado
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Callback para feedback de progreso"""
        feedback = feedback_msg.feedback
        
        self.get_logger().info(
            f'Progress: {feedback.progress_percentage:.1f}% - '
            f'Phase: {feedback.current_phase} - '
            f'Cycles: {feedback.steering_cycles}')
        
        if hasattr(feedback, 'sensors_detecting_obstacle') and feedback.sensors_detecting_obstacle:
            self.get_logger().warn('Obstacle detected by sensors!')
    
    def get_result_callback(self, future):
        """Callback cuando se recibe el resultado final"""
        result = future.result().result
        
        if result.success:
            self.get_logger().info(
                f'Maneuver successful! '
                f'Time: {result.maneuver_time:.2f}s, '
                f'Distance: {result.total_distance:.2f}m')
        else:
            self.get_logger().error(f'Maneuver failed: {result.message}')
        
        # Terminar el nodo
        rclpy.shutdown()
    
    def cancel_goal(self):
        """Cancelar goal actual"""
        if hasattr(self, '_send_goal_future'):
            self.get_logger().info('Canceling current goal...')
            cancel_future = self._send_goal_future.result().cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
    
    def cancel_done_callback(self, future):
        """Callback cuando se completa la cancelación"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

def main(args=None):
    rclpy.init(args=args)
    
    action_client = ManeuverActionClient()
    
    # Crear pose objetivo para estacionamiento
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    target_pose.header.stamp = action_client.get_clock().now().to_msg()
    target_pose.pose.position.x = 5.0
    target_pose.pose.position.y = 2.0
    target_pose.pose.position.z = 0.0
    target_pose.pose.orientation.w = 1.0
    
    # Enviar goal de estacionamiento en paralelo
    if action_client.send_goal('park_parallel', target_pose, space_length=6.0, max_speed=0.2):
        
        # Simular cancelación después de 10 segundos (opcional)
        # import threading
        # def cancel_after_delay():
        #     time.sleep(10)
        #     action_client.cancel_goal()
        # threading.Thread(target=cancel_after_delay, daemon=True).start()
        
        # Spin hasta que termine la acción
        rclpy.spin(action_client)
    
    action_client.destroy_node()

if __name__ == '__main__':
    main()
```

## Acciones del eCar

### Arquitectura de Acciones del Sistema

```bash
# Acciones de Navegación
/navigate_to_pose          # Navegación básica a punto
/follow_waypoints         # Seguir secuencia de waypoints
/explore_area            # Exploración autónoma de área
/return_to_dock          # Volver a estación de carga

# Acciones de Maniobras
/perform_maneuver        # Maniobras complejas (estacionamiento, giros)
/calibrate_system       # Calibración completa del sistema
/emergency_sequence     # Secuencia de emergencia

# Acciones de Mapeo
/create_map             # Crear mapa con SLAM
/update_map             # Actualizar mapa existente
/merge_maps             # Fusionar múltiples mapas

# Acciones de Carga
/charge_battery         # Proceso de carga completo
/dock_to_charger       # Secuencia de acoplamiento
```

### Launch File para Acciones

```python
# launch/ecar_actions.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    
    # Argumentos
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='ecar',
        description='Robot namespace'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Parámetros comunes
    common_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }
    
    # Grupo de nodos con namespace
    ecar_actions_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        
        # Servidor de navegación
        Node(
            package='tadeo_ecar_navigation',
            executable='navigation_action_server',
            name='navigation_action_server',
            parameters=[common_params, {
                'linear_velocity': 0.5,
                'angular_velocity': 0.3,
                'goal_tolerance': 0.2,
                'feedback_rate': 2.0
            }]
        ),
        
        # Servidor de maniobras
        Node(
            package='tadeo_ecar_navigation',
            executable='maneuver_action_server.py',
            name='maneuver_action_server',
            parameters=[common_params, {
                'max_linear_velocity': 0.3,
                'max_angular_velocity': 0.5,
                'position_tolerance': 0.1
            }]
        ),
        
        # Servidor de mapeo
        Node(
            package='tadeo_ecar_slam',
            executable='mapping_action_server.py',
            name='mapping_action_server',
            parameters=[common_params, {
                'map_resolution': 0.05,
                'exploration_radius': 10.0,
                'min_exploration_time': 60.0
            }]
        ),
        
        # Servidor de carga
        Node(
            package='tadeo_ecar_power',
            executable='charging_action_server.py',
            name='charging_action_server',
            parameters=[common_params, {
                'charging_voltage': 12.6,
                'charging_current_max': 5.0,
                'battery_capacity': 10000  # mAh
            }]
        ),
    ])
    
    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        ecar_actions_group
    ])
```

## Cancelación de Acciones

### Cancelación Cooperativa

```cpp
// En el servidor de acción
void execute_navigation(const std::shared_ptr<GoalHandle> goal_handle)
{
    rclcpp::Rate rate(10);
    
    while (rclcpp::ok() && !goal_reached) {
        // Verificar cancelación
        if (goal_handle->is_canceling()) {
            // Detener robot de forma segura
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);
            
            // Marcar como cancelado
            auto result = std::make_shared<NavigateToPose::Result>();
            result->result.result = nav2_msgs::msg::NavigationResult::CANCELED;
            goal_handle->canceled(result);
            
            RCLCPP_INFO(this->get_logger(), "Navigation canceled safely");
            return;
        }
        
        // Continuar con navegación...
        rate.sleep();
    }
}
```

### Cancelación con Timeout

```python
class RobustActionClient(Node):
    def __init__(self):
        super().__init__('robust_action_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
    
    async def navigate_with_timeout(self, x, y, timeout=60.0):
        """Navegar con timeout automático"""
        
        # Enviar goal
        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        
        goal_handle = await self.action_client.send_goal_async(goal)
        
        if not goal_handle.accepted:
            return False, "Goal rejected"
        
        # Esperar resultado con timeout
        try:
            result = await asyncio.wait_for(
                goal_handle.get_result_async(),
                timeout=timeout
            )
            return result.success, result.message
            
        except asyncio.TimeoutError:
            # Cancelar goal por timeout
            self.get_logger().warn(f'Navigation timed out after {timeout}s, canceling...')
            
            cancel_response = await goal_handle.cancel_goal_async()
            if cancel_response.return_code == CancelResponse.ACCEPT:
                return False, "Navigation canceled due to timeout"
            else:
                return False, "Navigation timed out and failed to cancel"
```

### Cancelación de Emergencia

```cpp
class EmergencyActionManager : public rclcpp::Node
{
public:
    EmergencyActionManager() : Node("emergency_action_manager")
    {
        // Suscribirse a botón de emergencia
        emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "emergency_stop", 10,
            std::bind(&EmergencyActionManager::emergencyCallback, this, std::placeholders::_1));
        
        // Clientes para todas las acciones
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        maneuver_client_ = rclcpp_action::create_client<PerformManeuver>(this, "perform_maneuver");
        mapping_client_ = rclcpp_action::create_client<CreateMap>(this, "create_map");
    }

private:
    void emergencyCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP ACTIVATED - Canceling all actions");
            
            // Cancelar todas las acciones activas
            cancelAllActions();
            
            // Detener robot inmediatamente
            auto stop_cmd = geometry_msgs::msg::Twist();
            emergency_stop_pub_->publish(stop_cmd);
        }
    }
    
    void cancelAllActions()
    {
        // Cancelar navegación
        if (nav_client_->get_number_of_running_goals() > 0) {
            nav_client_->async_cancel_all_goals();
            RCLCPP_INFO(this->get_logger(), "Canceled navigation actions");
        }
        
        // Cancelar maniobras
        if (maneuver_client_->get_number_of_running_goals() > 0) {
            maneuver_client_->async_cancel_all_goals();
            RCLCPP_INFO(this->get_logger(), "Canceled maneuver actions");
        }
        
        // Cancelar mapeo
        if (mapping_client_->get_number_of_running_goals() > 0) {
            mapping_client_->async_cancel_all_goals();
            RCLCPP_INFO(this->get_logger(), "Canceled mapping actions");
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr emergency_stop_pub_;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::Client<PerformManeuver>::SharedPtr maneuver_client_;
    rclcpp_action::Client<CreateMap>::SharedPtr mapping_client_;
};
```

## Múltiples Goals

### Gestor de Secuencia de Goals

```python
#!/usr/bin/env python3
# scripts/mission_sequence_manager.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import asyncio
from typing import List, Dict, Any

from nav2_msgs.action import NavigateToPose
from tadeo_ecar_interfaces.action import PerformManeuver, ChargeBattery
from geometry_msgs.msg import PoseStamped

class MissionSequenceManager(Node):
    def __init__(self):
        super().__init__('mission_sequence_manager')
        
        # Clientes de acción
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.maneuver_client = ActionClient(self, PerformManeuver, 'perform_maneuver')
        self.charge_client = ActionClient(self, ChargeBattery, 'charge_battery')
        
        self.get_logger().info('Mission sequence manager initialized')
    
    async def execute_mission_sequence(self, mission_steps: List[Dict[str, Any]]):
        """Ejecutar secuencia de misión con múltiples goals"""
        
        self.get_logger().info(f'Starting mission with {len(mission_steps)} steps')
        
        for i, step in enumerate(mission_steps):
            step_type = step.get('type')
            step_params = step.get('params', {})
            
            self.get_logger().info(f'Executing step {i+1}/{len(mission_steps)}: {step_type}')
            
            try:
                if step_type == 'navigate':
                    success = await self.execute_navigation_step(step_params)
                elif step_type == 'maneuver':
                    success = await self.execute_maneuver_step(step_params)
                elif step_type == 'charge':
                    success = await self.execute_charging_step(step_params)
                elif step_type == 'wait':
                    success = await self.execute_wait_step(step_params)
                else:
                    self.get_logger().error(f'Unknown step type: {step_type}')
                    success = False
                
                if not success:
                    self.get_logger().error(f'Step {i+1} failed, aborting mission')
                    return False
                    
            except Exception as e:
                self.get_logger().error(f'Exception in step {i+1}: {e}')
                return False
        
        self.get_logger().info('Mission completed successfully!')
        return True
    
    async def execute_navigation_step(self, params):
        """Ejecutar paso de navegación"""
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        tolerance = params.get('tolerance', 0.2)
        
        # Esperar servidor
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False
        
        # Crear goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        goal.tolerance = tolerance
        
        # Enviar goal y esperar resultado
        goal_handle = await self.nav_client.send_goal_async(goal)
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False
        
        result = await goal_handle.get_result_async()
        return result.result.result == nav2_msgs.msg.NavigationResult.SUCCEEDED
    
    async def execute_maneuver_step(self, params):
        """Ejecutar paso de maniobra"""
        maneuver_type = params.get('type', 'three_point_turn')
        max_speed = params.get('max_speed', 0.3)
        
        if not self.maneuver_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Maneuver server not available')
            return False
        
        # Crear goal
        goal = PerformManeuver.Goal()
        goal.maneuver_type = maneuver_type
        goal.max_speed = max_speed
        goal.use_sensors = True
        
        # Crear pose objetivo por defecto
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_handle = await self.maneuver_client.send_goal_async(goal)
        
        if not goal_handle.accepted:
            self.get_logger().error('Maneuver goal rejected')
            return False
        
        result = await goal_handle.get_result_async()
        return result.result.success
    
    async def execute_charging_step(self, params):
        """Ejecutar paso de carga"""
        target_percentage = params.get('target_percentage', 80.0)
        max_time = params.get('max_time', 3600.0)  # 1 hora por defecto
        
        if not self.charge_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Charging server not available')
            return False
        
        goal = ChargeBattery.Goal()
        goal.target_charge_percentage = target_percentage
        goal.return_to_dock = True
        goal.max_charging_time = max_time
        goal.optimize_battery_health = True
        
        goal_handle = await self.charge_client.send_goal_async(goal)
        
        if not goal_handle.accepted:
            self.get_logger().error('Charging goal rejected')
            return False
        
        result = await goal_handle.get_result_async()
        return result.result.success
    
    async def execute_wait_step(self, params):
        """Ejecutar paso de espera"""
        duration = params.get('duration', 5.0)
        
        self.get_logger().info(f'Waiting for {duration} seconds...')
        await asyncio.sleep(duration)
        return True

async def main():
    rclpy.init()
    
    manager = MissionSequenceManager()
    
    # Definir misión de ejemplo
    mission = [
        {
            'type': 'navigate',
            'params': {'x': 2.0, 'y': 1.0, 'tolerance': 0.3}
        },
        {
            'type': 'maneuver', 
            'params': {'type': 'park_parallel', 'max_speed': 0.2}
        },
        {
            'type': 'wait',
            'params': {'duration': 10.0}
        },
        {
            'type': 'navigate',
            'params': {'x': 0.0, 'y': 0.0, 'tolerance': 0.2}
        },
        {
            'type': 'charge',
            'params': {'target_percentage': 90.0, 'max_time': 1800.0}
        }
    ]
    
    # Ejecutar misión
    success = await manager.execute_mission_sequence(mission)
    
    if success:
        manager.get_logger().info('Mission sequence completed successfully!')
    else:
        manager.get_logger().error('Mission sequence failed!')
    
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
```

### Paralelización de Acciones

```cpp
class ParallelActionManager : public rclcpp::Node
{
public:
    ParallelActionManager() : Node("parallel_action_manager")
    {
        // Múltiples clientes para diferentes tipos de acciones
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        mapping_client_ = rclcpp_action::create_client<CreateMap>(this, "create_map");
        monitoring_client_ = rclcpp_action::create_client<MonitorSystem>(this, "monitor_system");
    }
    
    void executeParallelMission()
    {
        // Ejecutar navegación y mapeo en paralelo
        auto nav_future = sendNavigationGoal(5.0, 3.0);
        auto map_future = sendMappingGoal("exploration_map");
        auto monitor_future = sendMonitoringGoal();
        
        // Esperar a que todas las acciones terminen
        std::vector<std::future<bool>> futures = {
            std::move(nav_future),
            std::move(map_future), 
            std::move(monitor_future)
        };
        
        // Monitorear progreso
        bool all_completed = false;
        while (!all_completed && rclcpp::ok()) {
            all_completed = true;
            
            for (auto& future : futures) {
                if (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
                    all_completed = false;
                }
            }
            
            rclcpp::spin_some(this->shared_from_this());
        }
        
        // Verificar resultados
        RCLCPP_INFO(this->get_logger(), "All parallel actions completed");
    }

private:
    std::future<bool> sendNavigationGoal(double x, double y)
    {
        return std::async(std::launch::async, [this, x, y]() {
            auto goal = NavigateToPose::Goal();
            goal.pose.pose.position.x = x;
            goal.pose.pose.position.y = y;
            
            auto future = nav_client_->async_send_goal(goal);
            // Implementar lógica de espera...
            return true; // Simplificado
        });
    }
    
    // Métodos similares para mapping y monitoring...
};
```

## Ejercicios Prácticos

### Ejercicio 1: Acción de Exploración

Crear una acción para exploración autónoma:

```
# action/ExploreArea.action
# GOAL
geometry_msgs/Polygon exploration_boundary
float32 max_exploration_time
float32 target_coverage_percentage
---
# RESULT
bool success
string message
float32 area_explored
int32 waypoints_visited
float32 exploration_time
---
# FEEDBACK
float32 coverage_percentage
geometry_msgs/PoseStamped current_position
int32 waypoints_remaining
float32 estimated_completion_time
```

### Ejercicio 2: Cliente Inteligente

Crear un cliente que maneja múltiples tipos de acciones:

```python
class IntelligentActionClient(Node):
    def __init__(self):
        super().__init__('intelligent_action_client')
        # TODO: Crear clientes para diferentes acciones
        # TODO: Implementar estrategia de retry automático
        # TODO: Manejar cancelación inteligente
        # TODO: Priorizar acciones por importancia
```

### Ejercicio 3: Integración Completa

```bash
# 1. Compilar acciones personalizadas
colcon build --packages-select tadeo_ecar_interfaces

# 2. Ejecutar servidores de acción
ros2 launch tadeo_ecar_bringup ecar_actions.launch.py

# 3. Enviar goals desde línea de comandos
ros2 action send_goal /perform_maneuver tadeo_ecar_interfaces/action/PerformManeuver "{maneuver_type: 'park_parallel', max_speed: 0.2}"

# 4. Monitorear progreso
ros2 action list
ros2 action info /perform_maneuver

# 5. Ejecutar secuencia de misión
ros2 run tadeo_ecar_navigation mission_sequence_manager.py
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Concepto de acciones**: Comunicación asíncrona para tareas largas
2. **Goals, Feedback y Results**: Estructura completa de una acción
3. **Definición de acciones**: Acciones personalizadas para el eCar
4. **Implementación**: Servidores y clientes robustos en C++ y Python
5. **Cancelación**: Estrategias para detener acciones de forma segura
6. **Múltiples goals**: Secuencias y paralelización de acciones

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes cuándo usar acciones vs servicios vs tópicos
- [ ] Puedes crear acciones personalizadas
- [ ] Sabes implementar servidores de acción con feedback
- [ ] Comprendes la cancelación cooperativa
- [ ] Puedes manejar múltiples goals simultáneos
- [ ] Has probado las acciones con el sistema eCar

### Próximo Capítulo

En el Capítulo 7 estudiaremos:
- Sistema de parámetros en ROS2
- Parámetros dinámicos
- Archivos de configuración YAML
- Parámetros por nodo y globales
- Configuración del sistema eCar

## Referencias

- [ROS2 Actions Concepts](https://docs.ros.org/en/humble/Concepts/About-Actions.html)
- [Action Server C++](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [Action Server Python](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Custom Action Interfaces](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html)