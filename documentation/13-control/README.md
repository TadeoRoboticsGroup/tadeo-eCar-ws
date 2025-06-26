# Capítulo 13: Control Avanzado para eCar 4WD4WS

## Tabla de Contenidos

1. [Introducción al Control del eCar](#introducción-al-control-del-ecar)
2. [Cinemática del Robot 4WD4WS](#cinemática-del-robot-4wd4ws)
3. [Controladores PID](#controladores-pid)
4. [Control Omnidireccional](#control-omnidireccional)
5. [Model Predictive Control (MPC)](#model-predictive-control-mpc)
6. [Control de Bajo Nivel](#control-de-bajo-nivel)
7. [Integración con ROS2](#integración-con-ros2)
8. [Testing y Tuning](#testing-y-tuning)

## Introducción al Control del eCar

### Características Únicas del eCar 4WD4WS

El eCar (electric Car) es un robot móvil con características especiales que lo diferencia de robots convencionales:

```
eCar 4WD4WS = 4 Wheel Drive + 4 Wheel Steering

Capacidades:
✓ Movimiento omnidireccional (X, Y, θ)
✓ Rotación in-situ sin translación
✓ Movimiento lateral puro
✓ Movimiento diagonal
✓ Mayor maniobrabilidad en espacios reducidos
```

### Arquitectura de Control

```
                Control Architecture eCar 4WD4WS
                           |
    ┌─────────────────────────────────────────────────────────┐
    |                High Level Control                       |
    |    (Navigation Commands, Path Following)                |
    └─────────────────┬───────────────────────────────────────┘
                      |
    ┌─────────────────┼───────────────────────────────────────┐
    |                 |                                       |
┌───▼────┐    ┌───────▼────┐    ┌──────────▼────┐    ┌───────▼───┐
│Velocity│    │Kinematics  │    │Wheel Speed    │    │Steering   │
│Planning│    │Controller  │    │Controller     │    │Controller │
└────────┘    └────────────┘    └───────────────┘    └───────────┘
     |             |                      |                  |
┌────▼────┐   ┌────▼────┐         ┌──────▼──────┐     ┌──────▼──────┐
│Path     │   │4WD4WS   │         │Motor        │     │Servo        │
│Planner  │   │Inverse  │         │Controllers  │     │Controllers  │
│         │   │Kinematics│         │(4 motors)   │     │(4 servos)   │
└─────────┘   └─────────┘         └─────────────┘     └─────────────┘
```

### Ventajas del Sistema 4WD4WS

**1. Maniobrabilidad Superior**
```cpp
// Movimientos que otros robots no pueden realizar
void lateralMove(double velocity_y) {
    // Movimiento lateral puro sin rotación
    geometry_msgs::msg::Twist cmd;
    cmd.linear.y = velocity_y;  // Solo movimiento lateral
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
}

void diagonalMove(double vx, double vy) {
    // Movimiento diagonal simultáneo
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = 0.0;  // Sin rotación
}

void spinInPlace(double angular_velocity) {
    // Rotación pura sin translación
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = angular_velocity;
}
```

**2. Eficiencia Energética**
```cpp
// Trayectorias más cortas = menos energía
// Ejemplo: estacionamiento paralelo
void parallelPark() {
    // Fase 1: Movimiento lateral hacia espacio
    lateralMove(0.3);
    
    // Fase 2: Rotación in-situ para alinearse
    spinInPlace(0.5);
    
    // Fase 3: Movimiento hacia atrás
    forwardMove(-0.2);
    
    // Total: 3 maniobras vs 8+ en robot convencional
}
```

## Cinemática del Robot 4WD4WS

### Modelo Cinemático

El eCar 4WD4WS tiene 3 grados de libertad (DOF) en el plano:

```
DOF del eCar:
- Translación X (adelante/atrás)
- Translación Y (izquierda/derecha)  
- Rotación Z (yaw)

Configuración:
- 4 ruedas motrices independientes
- 4 ángulos de dirección independientes
- Centro de rotación variable
```

### Cinemática Directa

```cpp
// include/tadeo_ecar_control/kinematics_4wd4ws.hpp
#ifndef TADEO_ECAR_CONTROL__KINEMATICS_4WD4WS_HPP_
#define TADEO_ECAR_CONTROL__KINEMATICS_4WD4WS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>

namespace tadeo_ecar_control
{
struct WheelState
{
  double speed;      // Velocidad angular de la rueda (rad/s)
  double angle;      // Ángulo de dirección (rad)
  double position_x; // Posición X relativa al centro del robot
  double position_y; // Posición Y relativa al centro del robot
};

struct RobotState
{
  double velocity_x;     // Velocidad linear X (m/s)
  double velocity_y;     // Velocidad linear Y (m/s)
  double angular_z;      // Velocidad angular Z (rad/s)
  double icr_x;         // Centro instantáneo de rotación X
  double icr_y;         // Centro instantáneo de rotación Y
};

class Kinematics4WD4WS
{
public:
  Kinematics4WD4WS(double wheelbase, double track_width, double wheel_radius);
  
  // Cinemática directa: wheel states -> robot velocity
  RobotState forwardKinematics(const std::vector<WheelState>& wheel_states);
  
  // Cinemática inversa: robot velocity -> wheel states  
  std::vector<WheelState> inverseKinematics(const RobotState& robot_state);
  
  // Validación de comandos
  bool isValidCommand(const RobotState& robot_state);
  
  // Optimización de ángulos de dirección
  std::vector<WheelState> optimizeSteeringAngles(
    const std::vector<WheelState>& target_states,
    const std::vector<WheelState>& current_states);

private:
  // Parámetros geométricos del robot
  double wheelbase_;     // Distancia entre ejes (m)
  double track_width_;   // Ancho de vía (m) 
  double wheel_radius_;  // Radio de rueda (m)
  
  // Posiciones de las ruedas relativas al centro
  std::vector<Eigen::Vector2d> wheel_positions_;
  
  // Funciones auxiliares
  Eigen::Matrix3d calculateJacobian(const std::vector<WheelState>& wheel_states);
  double normalizeAngle(double angle);
  double shortestAngularDistance(double target, double current);
};
}  // namespace tadeo_ecar_control

#endif  // TADEO_ECAR_CONTROL__KINEMATICS_4WD4WS_HPP_
```

```cpp
// src/kinematics_4wd4ws.cpp
#include "tadeo_ecar_control/kinematics_4wd4ws.hpp"
#include <cmath>
#include <algorithm>

namespace tadeo_ecar_control
{
Kinematics4WD4WS::Kinematics4WD4WS(double wheelbase, double track_width, double wheel_radius)
  : wheelbase_(wheelbase), track_width_(track_width), wheel_radius_(wheel_radius)
{
  // Definir posiciones de las ruedas (FL, FR, RL, RR)
  wheel_positions_.resize(4);
  wheel_positions_[0] = Eigen::Vector2d(wheelbase_/2,  track_width_/2);   // Front Left
  wheel_positions_[1] = Eigen::Vector2d(wheelbase_/2, -track_width_/2);   // Front Right  
  wheel_positions_[2] = Eigen::Vector2d(-wheelbase_/2,  track_width_/2);  // Rear Left
  wheel_positions_[3] = Eigen::Vector2d(-wheelbase_/2, -track_width_/2);  // Rear Right
}

RobotState Kinematics4WD4WS::forwardKinematics(const std::vector<WheelState>& wheel_states)
{
  if (wheel_states.size() != 4) {
    throw std::invalid_argument("Expected 4 wheel states");
  }
  
  RobotState robot_state;
  
  // Método simplificado: promedio de velocidades
  // En implementación completa se usaría least-squares
  
  double vx_sum = 0.0, vy_sum = 0.0, wz_sum = 0.0;
  
  for (size_t i = 0; i < 4; ++i) {
    const auto& wheel = wheel_states[i];
    
    // Velocidad lineal de la rueda
    double wheel_linear_velocity = wheel.speed * wheel_radius_;
    
    // Componentes de velocidad en coordenadas del robot
    double vx_wheel = wheel_linear_velocity * cos(wheel.angle);
    double vy_wheel = wheel_linear_velocity * sin(wheel.angle);
    
    vx_sum += vx_wheel;
    vy_sum += vy_wheel;
    
    // Contribución a velocidad angular (simplificado)
    // En implementación completa se calcularía usando ICR
    wz_sum += (vy_wheel * wheel_positions_[i][0] - vx_wheel * wheel_positions_[i][1]) /
              (wheel_positions_[i][0] * wheel_positions_[i][0] + wheel_positions_[i][1] * wheel_positions_[i][1]);
  }
  
  robot_state.velocity_x = vx_sum / 4.0;
  robot_state.velocity_y = vy_sum / 4.0;
  robot_state.angular_z = wz_sum / 4.0;
  
  // Calcular ICR (Centro Instantáneo de Rotación)
  if (std::abs(robot_state.angular_z) > 1e-6) {
    robot_state.icr_x = -robot_state.velocity_y / robot_state.angular_z;
    robot_state.icr_y = robot_state.velocity_x / robot_state.angular_z;
  } else {
    robot_state.icr_x = std::numeric_limits<double>::infinity();
    robot_state.icr_y = std::numeric_limits<double>::infinity();
  }
  
  return robot_state;
}

std::vector<WheelState> Kinematics4WD4WS::inverseKinematics(const RobotState& robot_state)
{
  std::vector<WheelState> wheel_states(4);
  
  // Para cada rueda, calcular velocidad y ángulo requeridos
  for (size_t i = 0; i < 4; ++i) {
    const Eigen::Vector2d& pos = wheel_positions_[i];
    
    // Velocidad de la rueda en coordenadas del robot
    double vx_wheel = robot_state.velocity_x - robot_state.angular_z * pos[1];
    double vy_wheel = robot_state.velocity_y + robot_state.angular_z * pos[0];
    
    // Velocidad y ángulo de la rueda
    double wheel_speed_linear = sqrt(vx_wheel * vx_wheel + vy_wheel * vy_wheel);
    double wheel_angle = atan2(vy_wheel, vx_wheel);
    
    // Convertir a velocidad angular
    wheel_states[i].speed = wheel_speed_linear / wheel_radius_;
    wheel_states[i].angle = normalizeAngle(wheel_angle);
    wheel_states[i].position_x = pos[0];
    wheel_states[i].position_y = pos[1];
  }
  
  return wheel_states;
}

bool Kinematics4WD4WS::isValidCommand(const RobotState& robot_state)
{
  // Verificar límites físicos
  const double MAX_LINEAR_VELOCITY = 3.0;  // m/s
  const double MAX_ANGULAR_VELOCITY = 2.0; // rad/s
  
  if (std::abs(robot_state.velocity_x) > MAX_LINEAR_VELOCITY ||
      std::abs(robot_state.velocity_y) > MAX_LINEAR_VELOCITY ||
      std::abs(robot_state.angular_z) > MAX_ANGULAR_VELOCITY) {
    return false;
  }
  
  // Verificar compatibilidad cinemática
  auto wheel_states = inverseKinematics(robot_state);
  
  const double MAX_WHEEL_SPEED = 10.0; // rad/s
  for (const auto& wheel : wheel_states) {
    if (std::abs(wheel.speed) > MAX_WHEEL_SPEED) {
      return false;
    }
  }
  
  return true;
}

std::vector<WheelState> Kinematics4WD4WS::optimizeSteeringAngles(
  const std::vector<WheelState>& target_states,
  const std::vector<WheelState>& current_states)
{
  std::vector<WheelState> optimized_states = target_states;
  
  for (size_t i = 0; i < 4; ++i) {
    double target_angle = target_states[i].angle;
    double current_angle = current_states[i].angle;
    
    // Encontrar el ángulo objetivo que requiera menor rotación
    double angle_diff = shortestAngularDistance(target_angle, current_angle);
    
    // Si la diferencia es > 90°, invertir dirección y velocidad
    if (std::abs(angle_diff) > M_PI_2) {
      optimized_states[i].angle = normalizeAngle(target_angle + M_PI);
      optimized_states[i].speed = -optimized_states[i].speed;
    }
  }
  
  return optimized_states;
}

double Kinematics4WD4WS::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double Kinematics4WD4WS::shortestAngularDistance(double target, double current)
{
  double diff = normalizeAngle(target - current);
  return diff;
}
}  // namespace tadeo_ecar_control
```

## Controladores PID

### Controlador PID para Velocidad

```cpp
// include/tadeo_ecar_control/pid_controller.hpp
#ifndef TADEO_ECAR_CONTROL__PID_CONTROLLER_HPP_
#define TADEO_ECAR_CONTROL__PID_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace tadeo_ecar_control
{
class PIDController
{
public:
  struct PIDGains
  {
    double kp;  // Proportional gain
    double ki;  // Integral gain  
    double kd;  // Derivative gain
  };
  
  struct PIDLimits
  {
    double max_output;
    double min_output;
    double max_integral;
    double min_integral;
  };
  
  PIDController(const PIDGains& gains, const PIDLimits& limits);
  
  double update(double setpoint, double measurement, double dt);
  void reset();
  void setGains(const PIDGains& gains);
  void setLimits(const PIDLimits& limits);
  
  // Debug information
  double getProportionalTerm() const { return proportional_term_; }
  double getIntegralTerm() const { return integral_term_; }
  double getDerivativeTerm() const { return derivative_term_; }
  double getError() const { return error_; }

private:
  PIDGains gains_;
  PIDLimits limits_;
  
  double error_;
  double previous_error_;
  double integral_;
  double proportional_term_;
  double integral_term_;
  double derivative_term_;
  
  bool first_run_;
};

class VelocityController
{
public:
  VelocityController();
  
  void configure(const PIDGains& linear_gains, const PIDGains& angular_gains);
  
  geometry_msgs::msg::Twist controlVelocity(
    const geometry_msgs::msg::Twist& desired_velocity,
    const geometry_msgs::msg::Twist& current_velocity,
    double dt);

private:
  std::unique_ptr<PIDController> vx_controller_;
  std::unique_ptr<PIDController> vy_controller_;
  std::unique_ptr<PIDController> wz_controller_;
};
}  // namespace tadeo_ecar_control

#endif  // TADEO_ECAR_CONTROL__PID_CONTROLLER_HPP_
```

```cpp
// src/pid_controller.cpp
#include "tadeo_ecar_control/pid_controller.hpp"
#include <algorithm>

namespace tadeo_ecar_control
{
PIDController::PIDController(const PIDGains& gains, const PIDLimits& limits)
  : gains_(gains), limits_(limits), error_(0.0), previous_error_(0.0), 
    integral_(0.0), proportional_term_(0.0), integral_term_(0.0), 
    derivative_term_(0.0), first_run_(true)
{
}

double PIDController::update(double setpoint, double measurement, double dt)
{
  // Calculate error
  error_ = setpoint - measurement;
  
  // Proportional term
  proportional_term_ = gains_.kp * error_;
  
  // Integral term
  integral_ += error_ * dt;
  integral_ = std::clamp(integral_, limits_.min_integral, limits_.max_integral);
  integral_term_ = gains_.ki * integral_;
  
  // Derivative term
  if (!first_run_) {
    derivative_term_ = gains_.kd * (error_ - previous_error_) / dt;
  } else {
    derivative_term_ = 0.0;
    first_run_ = false;
  }
  
  // Calculate output
  double output = proportional_term_ + integral_term_ + derivative_term_;
  output = std::clamp(output, limits_.min_output, limits_.max_output);
  
  // Store for next iteration
  previous_error_ = error_;
  
  return output;
}

void PIDController::reset()
{
  error_ = 0.0;
  previous_error_ = 0.0;
  integral_ = 0.0;
  proportional_term_ = 0.0;
  integral_term_ = 0.0;
  derivative_term_ = 0.0;
  first_run_ = true;
}

void PIDController::setGains(const PIDGains& gains)
{
  gains_ = gains;
}

void PIDController::setLimits(const PIDLimits& limits)
{
  limits_ = limits;
}

VelocityController::VelocityController()
{
  // Default gains for eCar (to be tuned)
  PIDGains linear_gains{1.0, 0.1, 0.05};
  PIDGains angular_gains{2.0, 0.2, 0.1};
  
  // Default limits
  PIDLimits linear_limits{5.0, -5.0, 2.0, -2.0};  // Max output ±5 m/s
  PIDLimits angular_limits{3.0, -3.0, 1.0, -1.0}; // Max output ±3 rad/s
  
  vx_controller_ = std::make_unique<PIDController>(linear_gains, linear_limits);
  vy_controller_ = std::make_unique<PIDController>(linear_gains, linear_limits);
  wz_controller_ = std::make_unique<PIDController>(angular_gains, angular_limits);
}

void VelocityController::configure(const PIDGains& linear_gains, const PIDGains& angular_gains)
{
  vx_controller_->setGains(linear_gains);
  vy_controller_->setGains(linear_gains);
  wz_controller_->setGains(angular_gains);
}

geometry_msgs::msg::Twist VelocityController::controlVelocity(
  const geometry_msgs::msg::Twist& desired_velocity,
  const geometry_msgs::msg::Twist& current_velocity,
  double dt)
{
  geometry_msgs::msg::Twist control_output;
  
  // Control each DOF independently
  control_output.linear.x = vx_controller_->update(
    desired_velocity.linear.x, current_velocity.linear.x, dt);
  
  control_output.linear.y = vy_controller_->update(
    desired_velocity.linear.y, current_velocity.linear.y, dt);
  
  control_output.angular.z = wz_controller_->update(
    desired_velocity.angular.z, current_velocity.angular.z, dt);
  
  return control_output;
}
}  // namespace tadeo_ecar_control
```

### Controlador de Ruedas Individual

```cpp
// include/tadeo_ecar_control/wheel_controller.hpp
#ifndef TADEO_ECAR_CONTROL__WHEEL_CONTROLLER_HPP_
#define TADEO_ECAR_CONTROL__WHEEL_CONTROLLER_HPP_

#include "tadeo_ecar_control/pid_controller.hpp"
#include <memory>

namespace tadeo_ecar_control
{
class WheelController
{
public:
  WheelController(int wheel_id);
  
  void configure(const PIDGains& speed_gains, const PIDGains& steering_gains);
  
  struct WheelCommand
  {
    double motor_effort;     // PWM o voltaje para motor
    double steering_effort;  // PWM o posición para servo
  };
  
  WheelCommand controlWheel(
    const WheelState& desired_state,
    const WheelState& current_state,
    double dt);
  
  void reset();
  
  // Diagnostics
  struct WheelDiagnostics
  {
    double speed_error;
    double steering_error;
    double motor_effort;
    double steering_effort;
  };
  
  WheelDiagnostics getDiagnostics() const;

private:
  int wheel_id_;
  std::unique_ptr<PIDController> speed_controller_;
  std::unique_ptr<PIDController> steering_controller_;
  
  WheelDiagnostics diagnostics_;
};

class MultiWheelController
{
public:
  MultiWheelController();
  
  void configure(const PIDGains& speed_gains, const PIDGains& steering_gains);
  
  std::vector<WheelController::WheelCommand> controlWheels(
    const std::vector<WheelState>& desired_states,
    const std::vector<WheelState>& current_states,
    double dt);
  
  void reset();
  
  std::vector<WheelController::WheelDiagnostics> getAllDiagnostics() const;

private:
  std::vector<std::unique_ptr<WheelController>> wheel_controllers_;
};
}  // namespace tadeo_ecar_control

#endif  // TADEO_ECAR_CONTROL__WHEEL_CONTROLLER_HPP_
```

```cpp
// src/wheel_controller.cpp
#include "tadeo_ecar_control/wheel_controller.hpp"

namespace tadeo_ecar_control
{
WheelController::WheelController(int wheel_id) : wheel_id_(wheel_id)
{
  // Default gains (to be tuned for each motor/servo)
  PIDGains speed_gains{5.0, 1.0, 0.1};   // Motor speed control
  PIDGains steering_gains{8.0, 0.5, 0.2}; // Servo position control
  
  // Limits for motor (PWM: -255 to 255)
  PIDLimits speed_limits{255.0, -255.0, 100.0, -100.0};
  
  // Limits for servo (angle in radians, effort in PWM)
  PIDLimits steering_limits{180.0, -180.0, 50.0, -50.0};
  
  speed_controller_ = std::make_unique<PIDController>(speed_gains, speed_limits);
  steering_controller_ = std::make_unique<PIDController>(steering_gains, steering_limits);
}

void WheelController::configure(const PIDGains& speed_gains, const PIDGains& steering_gains)
{
  speed_controller_->setGains(speed_gains);
  steering_controller_->setGains(steering_gains);
}

WheelController::WheelCommand WheelController::controlWheel(
  const WheelState& desired_state,
  const WheelState& current_state,
  double dt)
{
  WheelCommand command;
  
  // Control motor speed
  command.motor_effort = speed_controller_->update(
    desired_state.speed, current_state.speed, dt);
  
  // Control steering angle
  command.steering_effort = steering_controller_->update(
    desired_state.angle, current_state.angle, dt);
  
  // Update diagnostics
  diagnostics_.speed_error = speed_controller_->getError();
  diagnostics_.steering_error = steering_controller_->getError();
  diagnostics_.motor_effort = command.motor_effort;
  diagnostics_.steering_effort = command.steering_effort;
  
  return command;
}

void WheelController::reset()
{
  speed_controller_->reset();
  steering_controller_->reset();
}

WheelController::WheelDiagnostics WheelController::getDiagnostics() const
{
  return diagnostics_;
}

MultiWheelController::MultiWheelController()
{
  // Create controllers for 4 wheels
  wheel_controllers_.resize(4);
  for (int i = 0; i < 4; ++i) {
    wheel_controllers_[i] = std::make_unique<WheelController>(i);
  }
}

void MultiWheelController::configure(const PIDGains& speed_gains, const PIDGains& steering_gains)
{
  for (auto& controller : wheel_controllers_) {
    controller->configure(speed_gains, steering_gains);
  }
}

std::vector<WheelController::WheelCommand> MultiWheelController::controlWheels(
  const std::vector<WheelState>& desired_states,
  const std::vector<WheelState>& current_states,
  double dt)
{
  if (desired_states.size() != 4 || current_states.size() != 4) {
    throw std::invalid_argument("Expected 4 wheel states");
  }
  
  std::vector<WheelController::WheelCommand> commands(4);
  
  for (size_t i = 0; i < 4; ++i) {
    commands[i] = wheel_controllers_[i]->controlWheel(
      desired_states[i], current_states[i], dt);
  }
  
  return commands;
}

void MultiWheelController::reset()
{
  for (auto& controller : wheel_controllers_) {
    controller->reset();
  }
}

std::vector<WheelController::WheelDiagnostics> MultiWheelController::getAllDiagnostics() const
{
  std::vector<WheelController::WheelDiagnostics> all_diagnostics(4);
  
  for (size_t i = 0; i < 4; ++i) {
    all_diagnostics[i] = wheel_controllers_[i]->getDiagnostics();
  }
  
  return all_diagnostics;
}
}  // namespace tadeo_ecar_control
```

## Control Omnidireccional

### Controlador de Alto Nivel

```cpp
// include/tadeo_ecar_control/omnidirectional_controller.hpp
#ifndef TADEO_ECAR_CONTROL__OMNIDIRECTIONAL_CONTROLLER_HPP_
#define TADEO_ECAR_CONTROL__OMNIDIRECTIONAL_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tadeo_ecar_control/kinematics_4wd4ws.hpp"
#include "tadeo_ecar_control/pid_controller.hpp"

namespace tadeo_ecar_control
{
class OmnidirectionalController : public rclcpp::Node
{
public:
  OmnidirectionalController();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  void controlLoop();
  void publishWheelCommands();
  void publishDiagnostics();
  
  geometry_msgs::msg::Twist trajectoryControl(
    const geometry_msgs::msg::PoseStamped& goal,
    const nav_msgs::msg::Odometry& current_odom);
  
  bool isGoalReached(const geometry_msgs::msg::PoseStamped& goal,
                     const nav_msgs::msg::Odometry& current_odom);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  
  // Timer
  rclcpp::TimerInterface::SharedPtr control_timer_;
  
  // Control components
  std::unique_ptr<Kinematics4WD4WS> kinematics_;
  std::unique_ptr<VelocityController> velocity_controller_;
  std::unique_ptr<MultiWheelController> wheel_controller_;
  
  // Trajectory following controllers
  std::unique_ptr<PIDController> x_position_controller_;
  std::unique_ptr<PIDController> y_position_controller_;
  std::unique_ptr<PIDController> yaw_position_controller_;
  
  // State
  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_goal_;
  std::vector<WheelState> current_wheel_states_;
  
  // Parameters
  double control_frequency_;
  double goal_tolerance_position_;
  double goal_tolerance_orientation_;
  bool use_trajectory_control_;
  
  // Robot parameters
  double wheelbase_;
  double track_width_;
  double wheel_radius_;
};
}  // namespace tadeo_ecar_control

#endif  // TADEO_ECAR_CONTROL__OMNIDIRECTIONAL_CONTROLLER_HPP_
```

```cpp
// src/omnidirectional_controller.cpp
#include "tadeo_ecar_control/omnidirectional_controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace tadeo_ecar_control
{
OmnidirectionalController::OmnidirectionalController() : Node("omnidirectional_controller")
{
  // Declare parameters
  this->declare_parameter("control_frequency", 50.0);
  this->declare_parameter("wheelbase", 1.2);
  this->declare_parameter("track_width", 0.8);
  this->declare_parameter("wheel_radius", 0.15);
  this->declare_parameter("goal_tolerance_position", 0.1);
  this->declare_parameter("goal_tolerance_orientation", 0.1);
  this->declare_parameter("use_trajectory_control", false);
  
  // Get parameters
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  wheelbase_ = this->get_parameter("wheelbase").as_double();
  track_width_ = this->get_parameter("track_width").as_double();
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  goal_tolerance_position_ = this->get_parameter("goal_tolerance_position").as_double();
  goal_tolerance_orientation_ = this->get_parameter("goal_tolerance_orientation").as_double();
  use_trajectory_control_ = this->get_parameter("use_trajectory_control").as_bool();
  
  // Initialize control components
  kinematics_ = std::make_unique<Kinematics4WD4WS>(wheelbase_, track_width_, wheel_radius_);
  velocity_controller_ = std::make_unique<VelocityController>();
  wheel_controller_ = std::make_unique<MultiWheelController>();
  
  // Initialize trajectory controllers
  PIDGains position_gains{2.0, 0.1, 0.3};
  PIDGains orientation_gains{3.0, 0.2, 0.5};
  PIDLimits position_limits{2.0, -2.0, 1.0, -1.0};
  PIDLimits orientation_limits{1.5, -1.5, 0.5, -0.5};
  
  x_position_controller_ = std::make_unique<PIDController>(position_gains, position_limits);
  y_position_controller_ = std::make_unique<PIDController>(position_gains, position_limits);
  yaw_position_controller_ = std::make_unique<PIDController>(orientation_gains, orientation_limits);
  
  // Publishers
  wheel_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "wheel_commands", 10);
  motor_effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "motor_efforts", 10);
  servo_effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "servo_efforts", 10);
  diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>(
    "control_diagnostics", 10);
  
  // Subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&OmnidirectionalController::cmdVelCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&OmnidirectionalController::odomCallback, this, std::placeholders::_1));
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10,
    std::bind(&OmnidirectionalController::goalCallback, this, std::placeholders::_1));
  
  // Control timer
  auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&OmnidirectionalController::controlLoop, this));
  
  // Initialize wheel states
  current_wheel_states_.resize(4);
  
  RCLCPP_INFO(this->get_logger(), "Omnidirectional Controller initialized");
  RCLCPP_INFO(this->get_logger(), "Robot params: wheelbase=%.2fm, track=%.2fm, wheel_radius=%.3fm",
              wheelbase_, track_width_, wheel_radius_);
}

void OmnidirectionalController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_cmd_vel_ = msg;
}

void OmnidirectionalController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
}

void OmnidirectionalController::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_goal_ = msg;
  
  // Reset trajectory controllers when new goal received
  x_position_controller_->reset();
  y_position_controller_->reset();
  yaw_position_controller_->reset();
  
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f, %.2f°)",
              msg->pose.position.x, msg->pose.position.y,
              tf2::getYaw(msg->pose.orientation) * 180.0 / M_PI);
}

void OmnidirectionalController::controlLoop()
{
  if (!current_odom_) {
    return;  // Wait for odometry
  }
  
  geometry_msgs::msg::Twist target_velocity;
  
  // Determine control mode
  if (use_trajectory_control_ && current_goal_) {
    // Use trajectory following control
    target_velocity = trajectoryControl(*current_goal_, *current_odom_);
    
    // Check if goal reached
    if (isGoalReached(*current_goal_, *current_odom_)) {
      current_goal_.reset();  // Clear goal
      target_velocity = geometry_msgs::msg::Twist();  // Stop
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
    }
  } else if (current_cmd_vel_) {
    // Use direct velocity control
    target_velocity = *current_cmd_vel_;
  } else {
    // No command, stop robot
    target_velocity = geometry_msgs::msg::Twist();
  }
  
  // Convert to robot state
  RobotState target_robot_state;
  target_robot_state.velocity_x = target_velocity.linear.x;
  target_robot_state.velocity_y = target_velocity.linear.y;
  target_robot_state.angular_z = target_velocity.angular.z;
  
  // Validate command
  if (!kinematics_->isValidCommand(target_robot_state)) {
    RCLCPP_WARN(this->get_logger(), "Invalid velocity command, stopping robot");
    target_robot_state = RobotState{0.0, 0.0, 0.0, 0.0, 0.0};
  }
  
  // Inverse kinematics
  auto target_wheel_states = kinematics_->inverseKinematics(target_robot_state);
  
  // Optimize steering angles
  target_wheel_states = kinematics_->optimizeSteeringAngles(target_wheel_states, current_wheel_states_);
  
  // Control wheels
  double dt = 1.0 / control_frequency_;
  auto wheel_commands = wheel_controller_->controlWheels(target_wheel_states, current_wheel_states_, dt);
  
  // Update current wheel states (in real system, this would come from encoders)
  current_wheel_states_ = target_wheel_states;
  
  // Publish commands
  publishWheelCommands();
  publishDiagnostics();
}

geometry_msgs::msg::Twist OmnidirectionalController::trajectoryControl(
  const geometry_msgs::msg::PoseStamped& goal,
  const nav_msgs::msg::Odometry& current_odom)
{
  geometry_msgs::msg::Twist cmd_vel;
  
  // Position errors
  double error_x = goal.pose.position.x - current_odom.pose.pose.position.x;
  double error_y = goal.pose.position.y - current_odom.pose.pose.position.y;
  
  // Orientation error
  double current_yaw = tf2::getYaw(current_odom.pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  double error_yaw = goal_yaw - current_yaw;
  
  // Normalize yaw error
  while (error_yaw > M_PI) error_yaw -= 2.0 * M_PI;
  while (error_yaw < -M_PI) error_yaw += 2.0 * M_PI;
  
  // Control update
  double dt = 1.0 / control_frequency_;
  
  cmd_vel.linear.x = x_position_controller_->update(goal.pose.position.x, 
                                                    current_odom.pose.pose.position.x, dt);
  cmd_vel.linear.y = y_position_controller_->update(goal.pose.position.y,
                                                    current_odom.pose.pose.position.y, dt);
  cmd_vel.angular.z = yaw_position_controller_->update(goal_yaw, current_yaw, dt);
  
  return cmd_vel;
}

bool OmnidirectionalController::isGoalReached(const geometry_msgs::msg::PoseStamped& goal,
                                             const nav_msgs::msg::Odometry& current_odom)
{
  // Position error
  double error_x = goal.pose.position.x - current_odom.pose.pose.position.x;
  double error_y = goal.pose.position.y - current_odom.pose.pose.position.y;
  double position_error = sqrt(error_x * error_x + error_y * error_y);
  
  // Orientation error
  double current_yaw = tf2::getYaw(current_odom.pose.pose.orientation);
  double goal_yaw = tf2::getYaw(goal.pose.orientation);
  double orientation_error = std::abs(goal_yaw - current_yaw);
  
  // Normalize orientation error
  while (orientation_error > M_PI) orientation_error -= 2.0 * M_PI;
  orientation_error = std::abs(orientation_error);
  
  return (position_error < goal_tolerance_position_) && 
         (orientation_error < goal_tolerance_orientation_);
}

void OmnidirectionalController::publishWheelCommands()
{
  // Publish joint state commands
  auto joint_state = sensor_msgs::msg::JointState();
  joint_state.header.stamp = this->get_clock()->now();
  
  joint_state.name = {"wheel_fl_speed", "wheel_fr_speed", "wheel_rl_speed", "wheel_rr_speed",
                      "wheel_fl_steer", "wheel_fr_steer", "wheel_rl_steer", "wheel_rr_steer"};
  
  joint_state.velocity.resize(8);
  joint_state.position.resize(8);
  
  for (size_t i = 0; i < 4; ++i) {
    joint_state.velocity[i] = current_wheel_states_[i].speed;
    joint_state.position[i + 4] = current_wheel_states_[i].angle;
  }
  
  wheel_cmd_pub_->publish(joint_state);
}

void OmnidirectionalController::publishDiagnostics()
{
  auto diagnostics = wheel_controller_->getAllDiagnostics();
  
  std::string diag_msg = "Wheel Diagnostics:\n";
  for (size_t i = 0; i < 4; ++i) {
    diag_msg += "Wheel " + std::to_string(i) + ": ";
    diag_msg += "Speed Error=" + std::to_string(diagnostics[i].speed_error) + " ";
    diag_msg += "Steer Error=" + std::to_string(diagnostics[i].steering_error) + "\n";
  }
  
  auto msg = std_msgs::msg::String();
  msg.data = diag_msg;
  diagnostics_pub_->publish(msg);
}
}  // namespace tadeo_ecar_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tadeo_ecar_control::OmnidirectionalController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## Model Predictive Control (MPC)

### Implementación MPC para eCar

```cpp
// include/tadeo_ecar_control/mpc_controller.hpp
#ifndef TADEO_ECAR_CONTROL__MPC_CONTROLLER_HPP_
#define TADEO_ECAR_CONTROL__MPC_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace tadeo_ecar_control
{
struct MPCParameters
{
  int prediction_horizon;     // N steps ahead
  int control_horizon;        // Number of control moves
  double sampling_time;       // dt
  
  // Cost function weights
  Eigen::Matrix3d Q;          // State cost
  Eigen::Matrix3d R;          // Control cost
  Eigen::Matrix3d Qf;         // Terminal cost
  
  // Constraints
  double max_linear_velocity;
  double max_angular_velocity;
  double max_linear_acceleration;
  double max_angular_acceleration;
};

class MPCController
{
public:
  MPCController(const MPCParameters& params);
  
  geometry_msgs::msg::Twist computeControl(
    const nav_msgs::msg::Odometry& current_state,
    const nav_msgs::msg::Path& reference_trajectory);
  
  void setParameters(const MPCParameters& params);
  
  // State vector: [x, y, theta]
  // Control vector: [vx, vy, omega]
  
private:
  MPCParameters params_;
  
  // MPC matrices
  Eigen::MatrixXd A_;  // State transition matrix
  Eigen::MatrixXd B_;  // Control input matrix
  Eigen::MatrixXd C_;  // Output matrix
  
  void setupMPCMatrices();
  Eigen::VectorXd solveMPC(const Eigen::Vector3d& current_state,
                          const Eigen::MatrixXd& reference_trajectory);
  
  Eigen::Vector3d predictState(const Eigen::Vector3d& state,
                              const Eigen::Vector3d& control,
                              double dt);
  
  // Constraint handling
  bool checkConstraints(const Eigen::Vector3d& control);
  Eigen::Vector3d applyConstraints(const Eigen::Vector3d& control);
};

class MPCPathFollower : public rclcpp::Node
{
public:
  MPCPathFollower();

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void controlLoop();
  
  nav_msgs::msg::Path interpolateTrajectory(const nav_msgs::msg::Path& path);
  Eigen::MatrixXd pathToTrajectoryMatrix(const nav_msgs::msg::Path& path);
  
  std::unique_ptr<MPCController> mpc_controller_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  
  rclcpp::TimerInterface::SharedPtr control_timer_;
  
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  nav_msgs::msg::Path::SharedPtr current_path_;
  
  double control_frequency_;
  int path_lookahead_points_;
};
}  // namespace tadeo_ecar_control

#endif  // TADEO_ECAR_CONTROL__MPC_CONTROLLER_HPP_
```

```cpp
// src/mpc_controller.cpp
#include "tadeo_ecar_control/mpc_controller.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace tadeo_ecar_control
{
MPCController::MPCController(const MPCParameters& params) : params_(params)
{
  setupMPCMatrices();
}

void MPCController::setupMPCMatrices()
{
  // Discrete-time model for omnidirectional robot
  // State: [x, y, theta]
  // Control: [vx, vy, omega]
  
  double dt = params_.sampling_time;
  
  // State transition matrix (linear model)
  A_ = Eigen::Matrix3d::Identity();
  // For omnidirectional robot, position integrates velocity directly
  
  // Control input matrix
  B_ = Eigen::Matrix3d::Zero();
  B_(0, 0) = dt;  // x position from vx
  B_(1, 1) = dt;  // y position from vy  
  B_(2, 2) = dt;  // theta from omega
  
  // Output matrix (we observe all states)
  C_ = Eigen::Matrix3d::Identity();
}

geometry_msgs::msg::Twist MPCController::computeControl(
  const nav_msgs::msg::Odometry& current_state,
  const nav_msgs::msg::Path& reference_trajectory)
{
  // Convert current state to Eigen vector
  Eigen::Vector3d current_state_vec;
  current_state_vec(0) = current_state.pose.pose.position.x;
  current_state_vec(1) = current_state.pose.pose.position.y;
  current_state_vec(2) = tf2::getYaw(current_state.pose.pose.orientation);
  
  // Convert reference trajectory to matrix
  Eigen::MatrixXd ref_trajectory = Eigen::MatrixXd::Zero(3, params_.prediction_horizon);
  
  int available_points = std::min(params_.prediction_horizon, 
                                 static_cast<int>(reference_trajectory.poses.size()));
  
  for (int i = 0; i < available_points; ++i) {
    const auto& pose = reference_trajectory.poses[i].pose;
    ref_trajectory(0, i) = pose.position.x;
    ref_trajectory(1, i) = pose.position.y;
    ref_trajectory(2, i) = tf2::getYaw(pose.orientation);
  }
  
  // Fill remaining horizon with last available point
  if (available_points > 0) {
    for (int i = available_points; i < params_.prediction_horizon; ++i) {
      ref_trajectory.col(i) = ref_trajectory.col(available_points - 1);
    }
  }
  
  // Solve MPC optimization
  Eigen::VectorXd optimal_controls = solveMPC(current_state_vec, ref_trajectory);
  
  // Extract first control action
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = optimal_controls(0);
  cmd_vel.linear.y = optimal_controls(1);
  cmd_vel.angular.z = optimal_controls(2);
  
  // Apply constraints
  Eigen::Vector3d control_vec(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  control_vec = applyConstraints(control_vec);
  
  cmd_vel.linear.x = control_vec(0);
  cmd_vel.linear.y = control_vec(1);
  cmd_vel.angular.z = control_vec(2);
  
  return cmd_vel;
}

Eigen::VectorXd MPCController::solveMPC(const Eigen::Vector3d& current_state,
                                       const Eigen::MatrixXd& reference_trajectory)
{
  // Simplified MPC solution using analytical approach
  // In practice, you would use a QP solver like OSQP or qpOASES
  
  int N = params_.prediction_horizon;
  int m = 3;  // Number of control inputs
  
  // Initialize optimal control sequence
  Eigen::VectorXd u_opt = Eigen::VectorXd::Zero(m * N);
  
  // Simple receding horizon approach
  for (int i = 0; i < N; ++i) {
    // Predict state at time i
    Eigen::Vector3d predicted_state = current_state;
    for (int j = 0; j < i; ++j) {
      Eigen::Vector3d control_j = u_opt.segment(j * m, m);
      predicted_state = predictState(predicted_state, control_j, params_.sampling_time);
    }
    
    // Calculate error to reference
    Eigen::Vector3d error = reference_trajectory.col(i) - predicted_state;
    
    // Simple proportional control (in practice, solve QP)
    Eigen::Vector3d control;
    control(0) = params_.Q(0, 0) * error(0) / params_.R(0, 0);  // vx
    control(1) = params_.Q(1, 1) * error(1) / params_.R(1, 1);  // vy
    control(2) = params_.Q(2, 2) * error(2) / params_.R(2, 2);  // omega
    
    // Apply constraints
    control = applyConstraints(control);
    
    u_opt.segment(i * m, m) = control;
  }
  
  return u_opt;
}

Eigen::Vector3d MPCController::predictState(const Eigen::Vector3d& state,
                                           const Eigen::Vector3d& control,
                                           double dt)
{
  // Simple integration for omnidirectional robot
  Eigen::Vector3d next_state = state;
  
  // Update position (considering current orientation)
  double cos_theta = cos(state(2));
  double sin_theta = sin(state(2));
  
  // Transform control from robot frame to world frame
  next_state(0) += dt * (control(0) * cos_theta - control(1) * sin_theta);
  next_state(1) += dt * (control(0) * sin_theta + control(1) * cos_theta);
  next_state(2) += dt * control(2);
  
  // Normalize angle
  while (next_state(2) > M_PI) next_state(2) -= 2.0 * M_PI;
  while (next_state(2) < -M_PI) next_state(2) += 2.0 * M_PI;
  
  return next_state;
}

Eigen::Vector3d MPCController::applyConstraints(const Eigen::Vector3d& control)
{
  Eigen::Vector3d constrained_control = control;
  
  // Velocity constraints
  constrained_control(0) = std::clamp(constrained_control(0), 
                                     -params_.max_linear_velocity, params_.max_linear_velocity);
  constrained_control(1) = std::clamp(constrained_control(1),
                                     -params_.max_linear_velocity, params_.max_linear_velocity);
  constrained_control(2) = std::clamp(constrained_control(2),
                                     -params_.max_angular_velocity, params_.max_angular_velocity);
  
  return constrained_control;
}

MPCPathFollower::MPCPathFollower() : Node("mpc_path_follower")
{
  // Declare parameters
  this->declare_parameter("control_frequency", 20.0);
  this->declare_parameter("prediction_horizon", 10);
  this->declare_parameter("control_horizon", 5);
  this->declare_parameter("sampling_time", 0.1);
  this->declare_parameter("path_lookahead_points", 20);
  
  // Get parameters
  control_frequency_ = this->get_parameter("control_frequency").as_double();
  path_lookahead_points_ = this->get_parameter("path_lookahead_points").as_int();
  
  // Setup MPC parameters
  MPCParameters mpc_params;
  mpc_params.prediction_horizon = this->get_parameter("prediction_horizon").as_int();
  mpc_params.control_horizon = this->get_parameter("control_horizon").as_int();
  mpc_params.sampling_time = this->get_parameter("sampling_time").as_double();
  
  // Cost matrices (tuning required)
  mpc_params.Q = Eigen::Matrix3d::Identity() * 10.0;  // State cost
  mpc_params.R = Eigen::Matrix3d::Identity() * 1.0;   // Control cost
  mpc_params.Qf = Eigen::Matrix3d::Identity() * 100.0; // Terminal cost
  
  // Constraints
  mpc_params.max_linear_velocity = 2.0;
  mpc_params.max_angular_velocity = 1.5;
  mpc_params.max_linear_acceleration = 1.0;
  mpc_params.max_angular_acceleration = 1.0;
  
  // Create MPC controller
  mpc_controller_ = std::make_unique<MPCController>(mpc_params);
  
  // Publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  predicted_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("predicted_path", 10);
  
  // Subscribers
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&MPCPathFollower::odomCallback, this, std::placeholders::_1));
  
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "path", 10,
    std::bind(&MPCPathFollower::pathCallback, this, std::placeholders::_1));
  
  // Control timer
  auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
    std::bind(&MPCPathFollower::controlLoop, this));
  
  RCLCPP_INFO(this->get_logger(), "MPC Path Follower initialized");
}

void MPCPathFollower::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
}

void MPCPathFollower::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = msg;
  RCLCPP_INFO(this->get_logger(), "Received path with %zu points", msg->poses.size());
}

void MPCPathFollower::controlLoop()
{
  if (!current_odom_ || !current_path_ || current_path_->poses.empty()) {
    return;
  }
  
  // Extract relevant portion of path
  nav_msgs::msg::Path trajectory_segment;
  trajectory_segment.header = current_path_->header;
  
  int start_idx = 0;  // In practice, find closest point on path
  int end_idx = std::min(start_idx + path_lookahead_points_, 
                        static_cast<int>(current_path_->poses.size()));
  
  for (int i = start_idx; i < end_idx; ++i) {
    trajectory_segment.poses.push_back(current_path_->poses[i]);
  }
  
  // Compute control using MPC
  auto cmd_vel = mpc_controller_->computeControl(*current_odom_, trajectory_segment);
  
  // Publish control command
  cmd_vel_pub_->publish(cmd_vel);
  
  // Publish predicted trajectory for visualization
  predicted_path_pub_->publish(trajectory_segment);
}
}  // namespace tadeo_ecar_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tadeo_ecar_control::MPCPathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## Control de Bajo Nivel

### Interface con Hardware

```cpp
// include/tadeo_ecar_control/hardware_interface.hpp
#ifndef TADEO_ECAR_CONTROL__HARDWARE_INTERFACE_HPP_
#define TADEO_ECAR_CONTROL__HARDWARE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

namespace tadeo_ecar_control
{
class HardwareInterface : public rclcpp::Node
{
public:
  HardwareInterface();

private:
  void wheelCommandsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void motorEffortsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void servoEffortsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  
  void readSensors();
  void publishJointStates();
  void publishDiagnostics();
  
  // Hardware communication
  bool initializeHardware();
  void shutdownHardware();
  bool sendMotorCommands(const std::vector<int16_t>& motor_pwm);
  bool sendServoCommands(const std::vector<int16_t>& servo_pwm);
  bool readEncoders(std::vector<double>& wheel_speeds);
  bool readServoPositions(std::vector<double>& servo_angles);
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hardware_diagnostics_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_commands_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_efforts_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr servo_efforts_sub_;
  
  // Timers
  rclcpp::TimerInterface::SharedPtr read_timer_;
  rclcpp::TimerInterface::SharedPtr diagnostics_timer_;
  
  // Hardware state
  std::vector<double> current_wheel_speeds_;
  std::vector<double> current_servo_angles_;
  std::vector<int16_t> current_motor_pwm_;
  std::vector<int16_t> current_servo_pwm_;
  
  // Parameters
  double sensor_read_frequency_;
  double diagnostics_frequency_;
  std::string hardware_port_;
  int hardware_baudrate_;
  
  // Hardware status
  bool hardware_initialized_;
  std::string hardware_status_;
};

class SerialCommunication
{
public:
  SerialCommunication(const std::string& port, int baudrate);
  ~SerialCommunication();
  
  bool initialize();
  void shutdown();
  
  bool sendMotorPWM(const std::vector<int16_t>& pwm_values);
  bool sendServoPWM(const std::vector<int16_t>& pwm_values);
  bool readEncoders(std::vector<double>& speeds);
  bool readServoPositions(std::vector<double>& angles);
  
  bool isConnected() const { return is_connected_; }
  
private:
  std::string port_;
  int baudrate_;
  int serial_fd_;
  bool is_connected_;
  
  bool writeCommand(const std::string& command);
  std::string readResponse();
  bool validateChecksum(const std::string& data);
};
}  // namespace tadeo_ecar_control

#endif  // TADEO_ECAR_CONTROL__HARDWARE_INTERFACE_HPP_
```

### Configuración de Parámetros

```yaml
# config/control_params.yaml
control_system:
  ros__parameters:
    
    # Robot physical parameters
    robot_params:
      wheelbase: 1.2              # Distance between front and rear axles (m)
      track_width: 0.8            # Distance between left and right wheels (m)
      wheel_radius: 0.15          # Wheel radius (m)
      max_wheel_speed: 10.0       # Maximum wheel speed (rad/s)
      max_steering_angle: 1.57    # Maximum steering angle (rad, ±90°)
    
    # Control frequencies
    frequencies:
      high_level_control: 50.0    # High-level control loop (Hz)
      low_level_control: 100.0    # Low-level motor control (Hz)
      sensor_reading: 100.0       # Sensor reading frequency (Hz)
      diagnostics: 10.0           # Diagnostics publishing (Hz)
    
    # PID gains for velocity control
    velocity_control:
      linear_x:
        kp: 1.5
        ki: 0.2
        kd: 0.1
      linear_y:
        kp: 1.5
        ki: 0.2
        kd: 0.1
      angular_z:
        kp: 2.0
        ki: 0.3
        kd: 0.15
    
    # PID gains for wheel speed control
    wheel_speed_control:
      kp: 5.0
      ki: 1.0
      kd: 0.1
      max_output: 255.0           # PWM range: -255 to 255
      min_output: -255.0
      max_integral: 100.0
      min_integral: -100.0
    
    # PID gains for steering control
    steering_control:
      kp: 8.0
      ki: 0.5
      kd: 0.2
      max_output: 180.0           # Servo range: -180 to 180
      min_output: -180.0
      max_integral: 50.0
      min_integral: -50.0
    
    # Position control (for trajectory following)
    position_control:
      x_position:
        kp: 2.0
        ki: 0.1
        kd: 0.3
      y_position:
        kp: 2.0
        ki: 0.1
        kd: 0.3
      yaw_position:
        kp: 3.0
        ki: 0.2
        kd: 0.5
    
    # Velocity limits
    velocity_limits:
      max_linear_velocity: 2.0    # m/s
      max_angular_velocity: 1.5   # rad/s
      max_linear_acceleration: 1.0 # m/s²
      max_angular_acceleration: 1.0 # rad/s²
    
    # Trajectory following
    trajectory_control:
      goal_tolerance_position: 0.1    # m
      goal_tolerance_orientation: 0.1 # rad
      lookahead_distance: 1.0         # m
      lateral_error_gain: 1.5
      heading_error_gain: 2.0
    
    # Hardware interface
    hardware:
      port: "/dev/ttyACM0"
      baudrate: 115200
      timeout: 1.0                # Communication timeout (s)
      retry_attempts: 3
    
    # Safety parameters
    safety:
      emergency_stop_deceleration: 3.0  # m/s²
      watchdog_timeout: 0.5             # s
      max_control_error: 1.0            # m/s
      sensor_timeout: 1.0               # s
    
    # MPC parameters (if using MPC)
    mpc:
      prediction_horizon: 10
      control_horizon: 5
      sampling_time: 0.1
      state_weight: [10.0, 10.0, 5.0]    # [x, y, theta]
      control_weight: [1.0, 1.0, 1.0]    # [vx, vy, omega]
      terminal_weight: [100.0, 100.0, 50.0]
    
    # Diagnostic thresholds
    diagnostics:
      max_control_error: 0.5      # m/s
      max_tracking_error: 0.3     # m
      min_battery_voltage: 11.0   # V
      max_motor_temperature: 70.0 # °C
      max_cpu_usage: 80.0         # %
```

## Integración con ROS2

### Launch File Completo

```python
# launch/control_system.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get directories
    control_pkg = FindPackageShare('tadeo_ecar_control')
    
    # Launch arguments
    use_mpc = LaunchConfiguration('use_mpc')
    use_hardware = LaunchConfiguration('use_hardware')
    control_config = LaunchConfiguration('control_config')
    
    declare_use_mpc = DeclareLaunchArgument(
        'use_mpc', default_value='false',
        description='Use Model Predictive Control instead of PID')
    
    declare_use_hardware = DeclareLaunchArgument(
        'use_hardware', default_value='true',
        description='Connect to real hardware')
    
    declare_control_config = DeclareLaunchArgument(
        'control_config',
        default_value=PathJoinSubstitution([control_pkg, 'config', 'control_params.yaml']),
        description='Control configuration file')
    
    # High-level omnidirectional controller
    omnidirectional_controller = Node(
        condition=IfCondition(LaunchConfiguration('use_mpc', default_value='false')),
        package='tadeo_ecar_control',
        executable='omnidirectional_controller_node',
        name='omnidirectional_controller',
        parameters=[control_config],
        output='screen'
    )
    
    # MPC controller (alternative)
    mpc_controller = Node(
        condition=IfCondition(use_mpc),
        package='tadeo_ecar_control',
        executable='mpc_path_follower_node',
        name='mpc_controller',
        parameters=[control_config],
        output='screen'
    )
    
    # Hardware interface
    hardware_interface = Node(
        condition=IfCondition(use_hardware),
        package='tadeo_ecar_control',
        executable='hardware_interface_node',
        name='hardware_interface',
        parameters=[control_config],
        output='screen'
    )
    
    # Control diagnostics
    control_diagnostics = Node(
        package='tadeo_ecar_control',
        executable='control_diagnostics_node',
        name='control_diagnostics',
        parameters=[control_config],
        output='screen'
    )
    
    # Safety monitor
    safety_monitor = Node(
        package='tadeo_ecar_control',
        executable='safety_monitor_node',
        name='safety_monitor',
        parameters=[control_config],
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        declare_use_mpc,
        declare_use_hardware,
        declare_control_config,
        
        # Nodes
        omnidirectional_controller,
        mpc_controller,
        hardware_interface,
        control_diagnostics,
        safety_monitor
    ])
```

## Testing y Tuning

### Scripts de Testing

```bash
#!/bin/bash
# scripts/test_control_system.sh

echo "=== eCar Control System Testing ==="

# Test 1: Basic system startup
echo "1. Testing control system startup..."
ros2 launch tadeo_ecar_control control_system.launch.py use_hardware:=false &
CONTROL_PID=$!
sleep 5

if ps -p $CONTROL_PID > /dev/null; then
    echo "✓ Control system started successfully"
else
    echo "✗ Control system failed to start"
    exit 1
fi

# Test 2: Basic velocity commands
echo "2. Testing velocity commands..."

# Forward movement
echo "Testing forward movement..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
sleep 2

# Lateral movement (4WD4WS specific)
echo "Testing lateral movement..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.5}}'
sleep 2

# Rotation
echo "Testing rotation..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'
sleep 2

# Stop
echo "Testing stop..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
sleep 1

echo "✓ Basic velocity commands tested"

# Test 3: Omnidirectional movement
echo "3. Testing omnidirectional movement..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.3}, angular: {z: 0.2}}'
sleep 3
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'

echo "✓ Omnidirectional movement tested"

# Test 4: Control diagnostics
echo "4. Testing control diagnostics..."
DIAGNOSTICS=$(timeout 5s ros2 topic echo /control_diagnostics --once 2>/dev/null)
if [ -n "$DIAGNOSTICS" ]; then
    echo "✓ Control diagnostics available"
else
    echo "✗ Control diagnostics not available"
fi

# Test 5: Hardware interface (if available)
echo "5. Testing hardware interface..."
if ros2 node list | grep -q "hardware_interface"; then
    echo "✓ Hardware interface node running"
    
    # Test joint state publishing
    JOINT_STATES=$(timeout 3s ros2 topic echo /joint_states --once 2>/dev/null)
    if [ -n "$JOINT_STATES" ]; then
        echo "✓ Joint states being published"
    else
        echo "✗ No joint states received"
    fi
else
    echo "- Hardware interface not running (simulation mode)"
fi

# Cleanup
kill $CONTROL_PID
echo "=== Control System Testing Complete ==="
```

### Herramientas de Tuning

```python
#!/usr/bin/env python3
# scripts/tune_pid_gains.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as plt
import time

class PIDTuner(Node):
    def __init__(self):
        super().__init__('pid_tuner')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Data logging
        self.time_data = []
        self.velocity_data = []
        self.command_data = []
        self.error_data = []
        
        self.current_odom = None
        self.start_time = time.time()
        
        self.get_logger().info('PID Tuner initialized')
    
    def odom_callback(self, msg):
        self.current_odom = msg
        
        # Log data for analysis
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        
        # Log current velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.angular.z
        
        self.velocity_data.append([vx, vy, vz])
    
    def run_step_response_test(self, axis='linear_x', target_velocity=1.0, duration=10.0):
        """Run step response test for PID tuning"""
        
        self.get_logger().info(f'Starting step response test for {axis}')
        
        # Clear previous data
        self.time_data.clear()
        self.velocity_data.clear()
        self.command_data.clear()
        self.error_data.clear()
        
        # Create step input
        cmd = Twist()
        if axis == 'linear_x':
            cmd.linear.x = target_velocity
        elif axis == 'linear_y':
            cmd.linear.y = target_velocity
        elif axis == 'angular_z':
            cmd.angular.z = target_velocity
        
        # Record start time
        self.start_time = time.time()
        
        # Send command and collect data
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(cmd)
            self.command_data.append(target_velocity)
            
            if self.current_odom:
                if axis == 'linear_x':
                    current_vel = self.current_odom.twist.twist.linear.x
                elif axis == 'linear_y':
                    current_vel = self.current_odom.twist.twist.linear.y
                elif axis == 'angular_z':
                    current_vel = self.current_odom.twist.twist.angular.z
                
                error = target_velocity - current_vel
                self.error_data.append(error)
            
            time.sleep(0.1)  # 10 Hz
        
        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Analyze results
        self.analyze_step_response(axis, target_velocity)
    
    def analyze_step_response(self, axis, target_velocity):
        """Analyze step response for PID tuning recommendations"""
        
        if len(self.velocity_data) < 10:
            self.get_logger().error('Insufficient data for analysis')
            return
        
        # Convert to numpy arrays
        velocities = np.array(self.velocity_data)
        errors = np.array(self.error_data)
        time_array = np.array(self.time_data)
        
        # Get the relevant velocity component
        if axis == 'linear_x':
            vel_component = velocities[:, 0]
        elif axis == 'linear_y':
            vel_component = velocities[:, 1]
        elif axis == 'angular_z':
            vel_component = velocities[:, 2]
        
        # Calculate performance metrics
        steady_state_error = np.mean(errors[-20:])  # Last 2 seconds
        max_overshoot = np.max(vel_component) - target_velocity
        overshoot_percent = (max_overshoot / target_velocity) * 100
        
        # Rise time (10% to 90% of final value)
        final_value = np.mean(vel_component[-20:])
        rise_start_idx = np.where(vel_component >= 0.1 * final_value)[0]
        rise_end_idx = np.where(vel_component >= 0.9 * final_value)[0]
        
        if len(rise_start_idx) > 0 and len(rise_end_idx) > 0:
            rise_time = time_array[rise_end_idx[0]] - time_array[rise_start_idx[0]]
        else:
            rise_time = float('inf')
        
        # Settling time (within 2% of final value)
        settling_threshold = 0.02 * final_value
        settled_indices = np.where(np.abs(vel_component - final_value) <= settling_threshold)[0]
        if len(settled_indices) > 0:
            settling_time = time_array[settled_indices[0]]
        else:
            settling_time = float('inf')
        
        # Generate tuning recommendations
        recommendations = self.generate_tuning_recommendations(
            steady_state_error, overshoot_percent, rise_time, settling_time)
        
        # Print results
        print(f"\n=== Step Response Analysis for {axis} ===")
        print(f"Target Velocity: {target_velocity}")
        print(f"Final Velocity: {final_value:.3f}")
        print(f"Steady State Error: {steady_state_error:.3f}")
        print(f"Overshoot: {overshoot_percent:.1f}%")
        print(f"Rise Time: {rise_time:.2f}s")
        print(f"Settling Time: {settling_time:.2f}s")
        print(f"\nTuning Recommendations:")
        for rec in recommendations:
            print(f"  - {rec}")
        
        # Plot results
        self.plot_step_response(time_array, vel_component, target_velocity, axis)
    
    def generate_tuning_recommendations(self, sse, overshoot, rise_time, settling_time):
        """Generate PID tuning recommendations based on performance metrics"""
        recommendations = []
        
        # Steady state error recommendations
        if abs(sse) > 0.1:
            recommendations.append("Increase Ki to reduce steady-state error")
        
        # Overshoot recommendations
        if overshoot > 10:
            recommendations.append("Decrease Kp or increase Kd to reduce overshoot")
        elif overshoot < 0:
            recommendations.append("System may be overdamped, consider increasing Kp")
        
        # Rise time recommendations
        if rise_time > 2.0:
            recommendations.append("Increase Kp to improve rise time")
        elif rise_time < 0.5:
            recommendations.append("System may be too aggressive, consider decreasing Kp")
        
        # Settling time recommendations
        if settling_time > 5.0:
            recommendations.append("Increase Kd to improve settling time")
        
        # Oscillation check
        vel_diff = np.diff(np.array(self.velocity_data)[:, 0])
        zero_crossings = np.where(np.diff(np.sign(vel_diff)))[0]
        if len(zero_crossings) > 10:  # Too many oscillations
            recommendations.append("System is oscillating, decrease Kp or increase Kd")
        
        if not recommendations:
            recommendations.append("Current tuning appears acceptable")
        
        return recommendations
    
    def plot_step_response(self, time_data, velocity_data, target, axis):
        """Plot step response for visualization"""
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(time_data, velocity_data, 'b-', label='Actual Velocity')
        plt.axhline(y=target, color='r', linestyle='--', label='Target Velocity')
        plt.xlabel('Time (s)')
        plt.ylabel(f'{axis} Velocity')
        plt.title(f'Step Response - {axis}')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(2, 1, 2)
        error_data = target - velocity_data
        plt.plot(time_data, error_data, 'r-', label='Error')
        plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.title('Tracking Error')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(f'step_response_{axis}.png')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    tuner = PIDTuner()
    
    # Run step response tests
    try:
        print("Starting PID tuning tests...")
        print("Make sure the control system is running!")
        
        input("Press Enter to start linear_x test...")
        tuner.run_step_response_test('linear_x', 1.0, 10.0)
        
        input("Press Enter to start linear_y test...")
        tuner.run_step_response_test('linear_y', 1.0, 10.0)
        
        input("Press Enter to start angular_z test...")
        tuner.run_step_response_test('angular_z', 0.5, 10.0)
        
        print("PID tuning tests completed!")
        
    except KeyboardInterrupt:
        pass
    finally:
        tuner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Ejercicios Prácticos

### Ejercicio 1: Implementar Control Básico

```cpp
// TODO: Implementar controlador de velocidad simple
class SimpleVelocityController : public rclcpp::Node
{
public:
    SimpleVelocityController();
    
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void controlLoop();
    
    // TODO: Implementar cinemática inversa básica
    // TODO: Aplicar comandos a motores y servos
    // TODO: Leer feedback de encoders
};
```

### Ejercicio 2: Tuning de Controladores

```bash
# TODO: Realizar tuning de ganancias PID
# 1. Ejecutar sistema de control
ros2 launch tadeo_ecar_control control_system.launch.py

# 2. Ejecutar herramienta de tuning
python3 scripts/tune_pid_gains.py

# 3. Ajustar parámetros en config/control_params.yaml
# 4. Probar diferentes configuraciones

# 5. Validar performance final
./scripts/test_control_system.sh
```

### Ejercicio 3: Implementar Modo de Emergencia

```cpp
// TODO: Implementar sistema de parada de emergencia
class EmergencyController
{
public:
    void emergencyStop();
    void emergencyBrake(double deceleration);
    bool isEmergencyActive();
    
private:
    // TODO: Implementar lógica de emergencia
    // TODO: Overrides de seguridad
    // TODO: Comunicación con hardware
};
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Control eCar 4WD4WS**: Capacidades omnidireccionales únicas
2. **Cinemática**: Modelos directos e inversos para 4WD4WS
3. **Controladores PID**: Implementación para múltiples DOF
4. **Control Omnidireccional**: Aprovechando capacidades 4WS
5. **MPC**: Control predictivo para seguimiento de trayectorias
6. **Hardware**: Interface de bajo nivel con motores y sensores
7. **Integración ROS2**: Sistema completo de control
8. **Testing**: Herramientas de validación y tuning

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes las ventajas del sistema 4WD4WS
- [ ] Puedes implementar cinemática inversa
- [ ] Sabes diseñar controladores PID
- [ ] Comprendes control omnidireccional
- [ ] Puedes integrar control con ROS2
- [ ] Sabes hacer tuning de controladores
- [ ] Has probado el sistema completo

### Próximo Capítulo

En el Capítulo 14 estudiaremos:
- Sistema de percepción avanzada
- Procesamiento de LiDAR y cámaras
- Fusión de sensores
- Detección de objetos
- Implementación para eCar

## Referencias

- [ROS2 Control](https://control.ros.org/)
- [Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- [PID Control](https://en.wikipedia.org/wiki/PID_controller)
- [Model Predictive Control](https://en.wikipedia.org/wiki/Model_predictive_control)
- [Omnidirectional Robots](https://www.robotics.org/joseph-engelberger-awards/joseph-engelberger-award-recipients.cfm)