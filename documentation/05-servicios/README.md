# Capítulo 5: Servicios en ROS2

## Tabla de Contenidos

1. [Concepto de Servicios](#concepto-de-servicios)
2. [Cliente-Servidor](#cliente-servidor)
3. [Definición de Servicios](#definición-de-servicios)
4. [Implementación en C++](#implementación-en-c++)
5. [Implementación en Python](#implementación-en-python)
6. [Servicios del eCar](#servicios-del-ecar)
7. [Manejo de Errores](#manejo-de-errores)
8. [Servicios Asíncronos](#servicios-asíncronos)

## Concepto de Servicios

### ¿Qué son los Servicios?

Los servicios en ROS2 permiten comunicación **síncrona** de tipo request-response entre nodos. A diferencia de los tópicos (asíncronos), los servicios garantizan respuesta.

```
Cliente                    Servidor
┌──────────────┐          ┌──────────────┐
│ Navigation   │ Request  │ Map Service  │
│ Node         │ ──────→  │              │
│              │          │              │
│              │ ←────── │              │
│              │ Response │              │
└──────────────┘          └──────────────┘
```

### Características de los Servicios

**1. Comunicación Síncrona**
```cpp
// Cliente espera respuesta antes de continuar
auto response = client->send_request(request);
// Bloquea hasta recibir respuesta
```

**2. Request-Response**
```cpp
// Estructura típica de servicio
struct CalibrateWheels {
    // Request
    string calibration_type;
    bool reset_offsets;
    
    // Response  
    bool success;
    string message;
    float64[] wheel_offsets;
};
```

**3. Uno a Uno**
```bash
# Un cliente se conecta a un servidor específico
# (vs. tópicos que son muchos-a-muchos)
```

### Cuándo Usar Servicios vs Tópicos

**Usar Servicios para:**
- Operaciones que requieren confirmación
- Configuración de parámetros
- Comandos que necesitan resultado
- Operaciones esporádicas (no continuas)

**Usar Tópicos para:**
- Flujo continuo de datos
- Información de sensores
- Comandos de control en tiempo real
- Broadcasting de información

### Ejemplos en el eCar

```bash
# Servicios (request-response)
/calibrate_wheels       # Calibrar sistema de dirección
/emergency_stop         # Parada de emergencia con confirmación
/save_map              # Guardar mapa SLAM
/get_robot_status      # Obtener estado completo
/set_navigation_goal   # Establecer objetivo con validación

# Tópicos (flujo continuo)
/scan                  # Datos LiDAR continuos
/cmd_vel              # Comandos de velocidad
/odom                 # Odometría continua
```

## Cliente-Servidor

### Patrón Cliente-Servidor

```cpp
// Servidor - Implementa la funcionalidad
class MapService : public rclcpp::Node
{
    void saveMapCallback(Request, Response) {
        // Realizar operación
        // Devolver resultado
    }
};

// Cliente - Solicita operaciones
class NavigationNode : public rclcpp::Node
{
    void requestMapSave() {
        auto request = std::make_shared<SaveMapRequest>();
        auto response = client_->send_request(request);
        // Usar respuesta
    }
};
```

### Ventajas del Patrón

**1. Confirmación de Operaciones**
```cpp
// El cliente sabe si la operación fue exitosa
if (response->success) {
    RCLCPP_INFO(this->get_logger(), "Map saved successfully");
} else {
    RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", response->message.c_str());
}
```

**2. Transferencia de Datos**
```cpp
// El servidor puede devolver datos complejos
auto response = calibration_client->send_request(request);
for (auto offset : response->wheel_offsets) {
    // Usar datos de calibración
}
```

**3. Validación de Parámetros**
```cpp
// El servidor valida la petición antes de ejecutar
if (request->wheel_index < 0 || request->wheel_index > 3) {
    response->success = false;
    response->message = "Invalid wheel index. Must be 0-3 for 4WD4WS";
    return;
}
```

## Definición de Servicios

### Servicios Estándar

ROS2 incluye servicios comunes:

```cpp
// Servicios básicos
#include <std_srvs/srv/trigger.hpp>           // bool → bool + string
#include <std_srvs/srv/set_bool.hpp>          // bool → bool + string
#include <std_srvs/srv/empty.hpp>             // void → void

// Ejemplo de uso
std_srvs::srv::Trigger::Request::SharedPtr request;
std_srvs::srv::Trigger::Response::SharedPtr response;
```

### Servicios Personalizados para el eCar

Crear servicios específicos del sistema eCar:

**1. Servicio de Calibración de Ruedas**
```
# srv/CalibrateWheels.srv
# Request
string calibration_type    # "steering", "odometry", "full"
bool reset_offsets        # Resetear offsets previos
int32 wheel_mask          # Bitmask: qué ruedas calibrar (0x0F = todas)
float64 max_steering_angle # Ángulo máximo para calibración steering
---
# Response  
bool success
string message
float64[] steering_offsets    # Offsets de steering por rueda [FL, FR, RL, RR]
float64[] encoder_offsets     # Offsets de encoder por rueda
float64 calibration_quality  # Calidad de la calibración (0.0-1.0)
float64 calibration_time     # Tiempo que tomó la calibración (segundos)
```

**2. Servicio de Configuración del Robot**
```
# srv/ConfigureRobot.srv
# Request
string configuration_name    # "default", "highway", "offroad", "indoor"
bool validate_config        # Validar configuración antes de aplicar
geometry_msgs/Twist max_velocities  # Velocidades máximas
float64 wheel_base          # Distancia entre ejes
float64 track_width         # Ancho de vía
---
# Response
bool success
string message
string applied_config       # Configuración que se aplicó realmente
string[] modified_parameters # Lista de parámetros modificados
geometry_msgs/Twist actual_max_velocities # Velocidades realmente configuradas
```

**3. Servicio de Estado del Robot**
```
# srv/GetRobotStatus.srv
# Request
bool include_diagnostics    # Incluir información de diagnóstico
bool include_sensors       # Incluir estado de sensores
bool include_hardware      # Incluir estado de hardware
---
# Response
bool success
string message

# Estado general
string robot_mode          # "manual", "autonomous", "emergency", "maintenance"
bool system_ready
builtin_interfaces/Time last_update

# Estado de hardware
bool[] wheel_status        # Estado de cada rueda (4 elements)
bool[] steering_status     # Estado de cada sistema de dirección
float64[] wheel_temperatures # Temperatura de motores
float64[] battery_voltages # Voltajes de baterías

# Estado de sensores  
bool lidar_online
bool camera_online
bool imu_online
bool gps_online
string[] sensor_errors     # Lista de errores de sensores

# Diagnósticos (si include_diagnostics=true)
diagnostic_msgs/DiagnosticArray diagnostics
```

### Configuración en CMakeLists.txt

```cmake
# Agregar al CMakeLists.txt de tadeo_ecar_interfaces
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Declarar servicios
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalibrateWheels.srv"
  "srv/ConfigureRobot.srv"
  "srv/GetRobotStatus.srv"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  diagnostic_msgs
  builtin_interfaces
)
```

## Implementación en C++

### Servidor de Calibración

```cpp
// include/tadeo_ecar_control/calibration_service.hpp
#ifndef TADEO_ECAR_CONTROL__CALIBRATION_SERVICE_HPP_
#define TADEO_ECAR_CONTROL__CALIBRATION_SERVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tadeo_ecar_interfaces/srv/calibrate_wheels.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>
#include <thread>

namespace tadeo_ecar_control
{
class CalibrationService : public rclcpp::Node
{
public:
    CalibrationService();

private:
    using CalibrateWheels = tadeo_ecar_interfaces::srv::CalibrateWheels;
    
    void calibrateWheelsCallback(
        const std::shared_ptr<CalibrateWheels::Request> request,
        std::shared_ptr<CalibrateWheels::Response> response);
    
    bool performSteeringCalibration(const std::shared_ptr<CalibrateWheels::Request> request,
                                   std::shared_ptr<CalibrateWheels::Response> response);
    
    bool performOdometryCalibration(const std::shared_ptr<CalibrateWheels::Request> request,
                                   std::shared_ptr<CalibrateWheels::Response> response);
    
    bool performFullCalibration(const std::shared_ptr<CalibrateWheels::Request> request,
                               std::shared_ptr<CalibrateWheels::Response> response);
    
    void wheelStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // Service server
    rclcpp::Service<CalibrateWheels>::SharedPtr calibration_service_;
    
    // Publishers for wheel commands
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_cmd_pub_;
    
    // Subscriber for wheel feedback
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_state_sub_;
    
    // Calibration state
    std::vector<double> current_wheel_positions_;
    std::vector<double> current_steering_angles_;
    bool calibration_in_progress_;
    std::mutex calibration_mutex_;
    
    // Hardware interface
    bool moveWheel(int wheel_index, double target_position, double timeout = 5.0);
    bool moveSteering(int wheel_index, double target_angle, double timeout = 5.0);
    bool waitForMovementComplete(double timeout = 10.0);
    
    // Configuration
    std::vector<double> wheel_gear_ratios_;
    std::vector<double> steering_gear_ratios_;
    double position_tolerance_;
    double angle_tolerance_;
};
}  // namespace tadeo_ecar_control

#endif  // TADEO_ECAR_CONTROL__CALIBRATION_SERVICE_HPP_
```

```cpp
// src/calibration_service.cpp
#include "tadeo_ecar_control/calibration_service.hpp"
#include <cmath>
#include <chrono>

namespace tadeo_ecar_control
{
CalibrationService::CalibrationService() 
    : Node("calibration_service_node"),
      calibration_in_progress_(false)
{
    // Declarar parámetros
    this->declare_parameter("position_tolerance", 0.01);
    this->declare_parameter("angle_tolerance", 0.017); // 1 grado
    
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();
    angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
    
    // Configurar gear ratios para 4WD4WS
    wheel_gear_ratios_ = {1.0, 1.0, 1.0, 1.0};      // Ruedas
    steering_gear_ratios_ = {1.0, 1.0, 1.0, 1.0};   // Dirección
    
    // Inicializar estado
    current_wheel_positions_.resize(4, 0.0);
    current_steering_angles_.resize(4, 0.0);
    
    // Crear servicio
    calibration_service_ = this->create_service<CalibrateWheels>(
        "calibrate_wheels",
        std::bind(&CalibrationService::calibrateWheelsCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    // Publishers para comandos
    wheel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "wheel_position_commands", 10);
    
    steering_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "steering_angle_commands", 10);
    
    // Subscriber para estado de ruedas
    wheel_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "wheel_states", 10,
        std::bind(&CalibrationService::wheelStateCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Calibration service initialized");
    RCLCPP_INFO(this->get_logger(), "Position tolerance: %.3f rad", position_tolerance_);
    RCLCPP_INFO(this->get_logger(), "Angle tolerance: %.3f rad (%.1f deg)", 
                angle_tolerance_, angle_tolerance_ * 180.0 / M_PI);
}

void CalibrationService::calibrateWheelsCallback(
    const std::shared_ptr<CalibrateWheels::Request> request,
    std::shared_ptr<CalibrateWheels::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Calibration request received: type='%s'", 
                request->calibration_type.c_str());
    
    // Verificar si ya hay calibración en progreso
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    if (calibration_in_progress_) {
        response->success = false;
        response->message = "Calibration already in progress";
        return;
    }
    
    calibration_in_progress_ = true;
    
    auto start_time = std::chrono::steady_clock::now();
    bool success = false;
    
    try {
        // Resetear offsets si se solicita
        if (request->reset_offsets) {
            RCLCPP_INFO(this->get_logger(), "Resetting previous calibration offsets");
            // Reset implementation would go here
        }
        
        // Ejecutar calibración según tipo
        if (request->calibration_type == "steering") {
            success = performSteeringCalibration(request, response);
        } else if (request->calibration_type == "odometry") {
            success = performOdometryCalibration(request, response);
        } else if (request->calibration_type == "full") {
            success = performFullCalibration(request, response);
        } else {
            response->success = false;
            response->message = "Unknown calibration type: " + request->calibration_type;
            calibration_in_progress_ = false;
            return;
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Calibration failed with exception: %s", e.what());
        response->success = false;
        response->message = "Calibration failed: " + std::string(e.what());
        success = false;
    }
    
    // Calcular tiempo de calibración
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    response->calibration_time = duration.count() / 1000.0;
    
    response->success = success;
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Calibration completed successfully in %.2f seconds", 
                    response->calibration_time);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Calibration failed: %s", response->message.c_str());
    }
    
    calibration_in_progress_ = false;
}

bool CalibrationService::performSteeringCalibration(
    const std::shared_ptr<CalibrateWheels::Request> request,
    std::shared_ptr<CalibrateWheels::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Starting steering calibration");
    
    response->steering_offsets.resize(4);
    response->encoder_offsets.resize(4);
    
    double max_angle = request->max_steering_angle;
    if (max_angle <= 0.0 || max_angle > M_PI/2) {
        max_angle = M_PI/4; // 45 grados por defecto
    }
    
    // Calibrar cada rueda según la máscara
    for (int wheel = 0; wheel < 4; ++wheel) {
        if (!(request->wheel_mask & (1 << wheel))) {
            RCLCPP_INFO(this->get_logger(), "Skipping wheel %d (not in mask)", wheel);
            continue;
        }
        
        RCLCPP_INFO(this->get_logger(), "Calibrating steering for wheel %d", wheel);
        
        // Mover a posición central
        if (!moveSteering(wheel, 0.0, 5.0)) {
            response->message = "Failed to move wheel " + std::to_string(wheel) + " to center";
            return false;
        }
        
        double center_position = current_steering_angles_[wheel];
        
        // Mover a extremo positivo
        if (!moveSteering(wheel, max_angle, 5.0)) {
            response->message = "Failed to move wheel " + std::to_string(wheel) + " to max angle";
            return false;
        }
        
        double max_position = current_steering_angles_[wheel];
        
        // Mover a extremo negativo
        if (!moveSteering(wheel, -max_angle, 5.0)) {
            response->message = "Failed to move wheel " + std::to_string(wheel) + " to min angle";
            return false;
        }
        
        double min_position = current_steering_angles_[wheel];
        
        // Volver a centro
        if (!moveSteering(wheel, 0.0, 5.0)) {
            response->message = "Failed to return wheel " + std::to_string(wheel) + " to center";
            return false;
        }
        
        // Calcular offset basado en simetría
        double measured_center = (max_position + min_position) / 2.0;
        double offset = center_position - measured_center;
        
        response->steering_offsets[wheel] = offset;
        response->encoder_offsets[wheel] = 0.0; // No encoder calibration in steering mode
        
        RCLCPP_INFO(this->get_logger(), 
                    "Wheel %d: center=%.3f, max=%.3f, min=%.3f, offset=%.3f",
                    wheel, center_position, max_position, min_position, offset);
    }
    
    // Calcular calidad de calibración (basada en simetría)
    double quality_sum = 0.0;
    int calibrated_wheels = 0;
    
    for (int wheel = 0; wheel < 4; ++wheel) {
        if (request->wheel_mask & (1 << wheel)) {
            double offset_magnitude = std::abs(response->steering_offsets[wheel]);
            double quality = std::max(0.0, 1.0 - offset_magnitude / (M_PI/6)); // Quality based on offset
            quality_sum += quality;
            calibrated_wheels++;
        }
    }
    
    response->calibration_quality = calibrated_wheels > 0 ? quality_sum / calibrated_wheels : 0.0;
    response->message = "Steering calibration completed successfully";
    
    return true;
}

bool CalibrationService::performOdometryCalibration(
    const std::shared_ptr<CalibrateWheels::Request> request,
    std::shared_ptr<CalibrateWheels::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Starting odometry calibration");
    
    response->steering_offsets.resize(4, 0.0); // No steering calibration in odometry mode
    response->encoder_offsets.resize(4);
    
    // Para calibración de odometría, realizamos giros conocidos y medimos encoders
    double test_rotations = 2.0; // 2 rotaciones completas
    
    for (int wheel = 0; wheel < 4; ++wheel) {
        if (!(request->wheel_mask & (1 << wheel))) {
            continue;
        }
        
        RCLCPP_INFO(this->get_logger(), "Calibrating odometry for wheel %d", wheel);
        
        double initial_position = current_wheel_positions_[wheel];
        double target_position = initial_position + test_rotations * 2.0 * M_PI;
        
        // Mover rueda la distancia conocida
        if (!moveWheel(wheel, target_position, 10.0)) {
            response->message = "Failed to move wheel " + std::to_string(wheel) + " for odometry calibration";
            return false;
        }
        
        double final_position = current_wheel_positions_[wheel];
        double measured_rotation = final_position - initial_position;
        double expected_rotation = test_rotations * 2.0 * M_PI;
        
        // Calcular factor de corrección
        double correction_factor = expected_rotation / measured_rotation;
        response->encoder_offsets[wheel] = correction_factor - 1.0;
        
        RCLCPP_INFO(this->get_logger(), 
                    "Wheel %d: expected=%.3f, measured=%.3f, correction=%.6f",
                    wheel, expected_rotation, measured_rotation, correction_factor);
    }
    
    response->calibration_quality = 0.95; // Simplified quality for odometry
    response->message = "Odometry calibration completed successfully";
    
    return true;
}

bool CalibrationService::performFullCalibration(
    const std::shared_ptr<CalibrateWheels::Request> request,
    std::shared_ptr<CalibrateWheels::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Starting full calibration (steering + odometry)");
    
    // Realizar calibración de dirección primero
    if (!performSteeringCalibration(request, response)) {
        return false;
    }
    
    // Luego calibración de odometría
    std::vector<double> steering_offsets = response->steering_offsets;
    if (!performOdometryCalibration(request, response)) {
        return false;
    }
    
    // Restaurar offsets de dirección
    response->steering_offsets = steering_offsets;
    
    // Calidad combinada
    response->calibration_quality = std::min(response->calibration_quality, 0.90);
    response->message = "Full calibration (steering + odometry) completed successfully";
    
    return true;
}

void CalibrationService::wheelStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Actualizar estado actual de ruedas
    if (msg->position.size() >= 4) {
        for (size_t i = 0; i < 4 && i < msg->position.size(); ++i) {
            current_wheel_positions_[i] = msg->position[i];
        }
    }
    
    // Actualizar ángulos de dirección (asumiendo que están después de las posiciones de rueda)
    if (msg->position.size() >= 8) {
        for (size_t i = 0; i < 4; ++i) {
            current_steering_angles_[i] = msg->position[i + 4];
        }
    }
}

bool CalibrationService::moveWheel(int wheel_index, double target_position, double timeout)
{
    if (wheel_index < 0 || wheel_index >= 4) {
        return false;
    }
    
    auto cmd_msg = std_msgs::msg::Float64MultiArray();
    cmd_msg.data.resize(4, 0.0);
    cmd_msg.data[wheel_index] = target_position;
    
    wheel_cmd_pub_->publish(cmd_msg);
    
    // Esperar a que se complete el movimiento
    return waitForMovementComplete(timeout);
}

bool CalibrationService::moveSteering(int wheel_index, double target_angle, double timeout)
{
    if (wheel_index < 0 || wheel_index >= 4) {
        return false;
    }
    
    auto cmd_msg = std_msgs::msg::Float64MultiArray();
    cmd_msg.data.resize(4, 0.0);
    cmd_msg.data[wheel_index] = target_angle;
    
    steering_cmd_pub_->publish(cmd_msg);
    
    // Esperar a que se complete el movimiento
    return waitForMovementComplete(timeout);
}

bool CalibrationService::waitForMovementComplete(double timeout)
{
    // Implementación simplificada - en sistema real monitorizaría posición actual
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        
        if (elapsed.count() / 1000.0 >= timeout) {
            return false; // Timeout
        }
        
        // En implementación real, verificaríamos si la posición/ángulo actual
        // está dentro de la tolerancia del objetivo
        
        // Por ahora, asumimos que el movimiento toma 1 segundo
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        if (elapsed.count() >= 1000) { // 1 segundo
            return true;
        }
    }
    
    return true;
}
}  // namespace tadeo_ecar_control

// Main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_control::CalibrationService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Cliente de Calibración

```cpp
// include/tadeo_ecar_control/calibration_client.hpp
#include <rclcpp/rclcpp.hpp>
#include <tadeo_ecar_interfaces/srv/calibrate_wheels.hpp>
#include <memory>

namespace tadeo_ecar_control
{
class CalibrationClient : public rclcpp::Node
{
public:
    CalibrationClient();
    
    bool calibrateWheels(const std::string& calibration_type, 
                        bool reset_offsets = false,
                        int wheel_mask = 0x0F,
                        double max_steering_angle = M_PI/4,
                        double timeout = 30.0);
    
    void printLastCalibrationResults();

private:
    using CalibrateWheels = tadeo_ecar_interfaces::srv::CalibrateWheels;
    
    rclcpp::Client<CalibrateWheels>::SharedPtr calibration_client_;
    CalibrateWheels::Response::SharedPtr last_response_;
};
}  // namespace tadeo_ecar_control
```

```cpp
// src/calibration_client.cpp
#include "tadeo_ecar_control/calibration_client.hpp"
#include <chrono>

namespace tadeo_ecar_control
{
CalibrationClient::CalibrationClient() : Node("calibration_client_node")
{
    calibration_client_ = this->create_client<CalibrateWheels>("calibrate_wheels");
    
    RCLCPP_INFO(this->get_logger(), "Calibration client initialized");
}

bool CalibrationClient::calibrateWheels(const std::string& calibration_type,
                                       bool reset_offsets,
                                       int wheel_mask,
                                       double max_steering_angle,
                                       double timeout)
{
    // Esperar a que el servicio esté disponible
    if (!calibration_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Calibration service not available");
        return false;
    }
    
    // Crear request
    auto request = std::make_shared<CalibrateWheels::Request>();
    request->calibration_type = calibration_type;
    request->reset_offsets = reset_offsets;
    request->wheel_mask = wheel_mask;
    request->max_steering_angle = max_steering_angle;
    
    RCLCPP_INFO(this->get_logger(), "Sending calibration request: type='%s', wheels=0x%X", 
                calibration_type.c_str(), wheel_mask);
    
    // Enviar request de forma asíncrona
    auto future = calibration_client_->async_send_request(request);
    
    // Esperar respuesta con timeout
    auto timeout_duration = std::chrono::duration<double>(timeout);
    if (future.wait_for(timeout_duration) == std::future_status::ready) {
        last_response_ = future.get();
        
        if (last_response_->success) {
            RCLCPP_INFO(this->get_logger(), "Calibration successful: %s", 
                        last_response_->message.c_str());
            printLastCalibrationResults();
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Calibration failed: %s", 
                         last_response_->message.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Calibration request timed out after %.1f seconds", timeout);
        return false;
    }
}

void CalibrationClient::printLastCalibrationResults()
{
    if (!last_response_) {
        RCLCPP_WARN(this->get_logger(), "No calibration results to display");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "=== Calibration Results ===");
    RCLCPP_INFO(this->get_logger(), "Quality: %.2f%%", last_response_->calibration_quality * 100.0);
    RCLCPP_INFO(this->get_logger(), "Time: %.2f seconds", last_response_->calibration_time);
    
    if (!last_response_->steering_offsets.empty()) {
        RCLCPP_INFO(this->get_logger(), "Steering offsets (rad):");
        for (size_t i = 0; i < last_response_->steering_offsets.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  Wheel %zu: %.6f", i, last_response_->steering_offsets[i]);
        }
    }
    
    if (!last_response_->encoder_offsets.empty()) {
        RCLCPP_INFO(this->get_logger(), "Encoder offsets:");
        for (size_t i = 0; i < last_response_->encoder_offsets.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  Wheel %zu: %.6f", i, last_response_->encoder_offsets[i]);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "===========================");
}
}  // namespace tadeo_ecar_control

// Example usage in main
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<tadeo_ecar_control::CalibrationClient>();
    
    // Ejemplo de calibración completa
    if (client_node->calibrateWheels("full", true, 0x0F, M_PI/4, 60.0)) {
        RCLCPP_INFO(client_node->get_logger(), "Full calibration completed successfully");
    } else {
        RCLCPP_ERROR(client_node->get_logger(), "Full calibration failed");
    }
    
    rclcpp::shutdown();
    return 0;
}
```

## Implementación en Python

### Servidor en Python

```python
#!/usr/bin/env python3
# scripts/robot_status_service.py

import rclpy
from rclpy.node import Node
import psutil
import json
from datetime import datetime

from tadeo_ecar_interfaces.srv import GetRobotStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class RobotStatusService(Node):
    def __init__(self):
        super().__init__('robot_status_service_node')
        
        # Crear servicio
        self.status_service = self.create_service(
            GetRobotStatus,
            'get_robot_status', 
            self.get_robot_status_callback)
        
        # Estado interno del robot
        self.robot_mode = "autonomous"
        self.system_ready = True
        self.wheel_status = [True, True, True, True]
        self.steering_status = [True, True, True, True]
        self.sensor_status = {
            'lidar': True,
            'camera': True,
            'imu': True,
            'gps': False  # GPS might not be available indoors
        }
        
        self.get_logger().info('Robot status service initialized')
    
    def get_robot_status_callback(self, request, response):
        """Callback para servicio de estado del robot"""
        try:
            # Estado general
            response.success = True
            response.message = "Status retrieved successfully"
            response.robot_mode = self.robot_mode
            response.system_ready = self.system_ready
            response.last_update = self.get_clock().now().to_msg()
            
            # Estado de hardware
            response.wheel_status = self.wheel_status
            response.steering_status = self.steering_status
            
            # Simular temperaturas de motores
            response.wheel_temperatures = [45.2, 46.1, 44.8, 45.5]  # °C
            
            # Simular voltajes de baterías (múltiples baterías)
            response.battery_voltages = [12.6, 12.4, 12.7]  # V
            
            # Estado de sensores si se solicita
            if request.include_sensors:
                response.lidar_online = self.sensor_status['lidar']
                response.camera_online = self.sensor_status['camera']
                response.imu_online = self.sensor_status['imu']
                response.gps_online = self.sensor_status['gps']
                
                # Errores de sensores
                response.sensor_errors = []
                for sensor, status in self.sensor_status.items():
                    if not status:
                        response.sensor_errors.append(f"{sensor}_offline")
            
            # Diagnósticos detallados si se solicitan
            if request.include_diagnostics:
                response.diagnostics = self.generate_diagnostics()
            
            # Información de hardware detallada si se solicita
            if request.include_hardware:
                # Agregar información adicional de hardware
                pass
            
            self.get_logger().debug(f"Status request served: mode={self.robot_mode}, ready={self.system_ready}")
            
        except Exception as e:
            response.success = False
            response.message = f"Error retrieving status: {str(e)}"
            self.get_logger().error(f"Error in status service: {e}")
        
        return response
    
    def generate_diagnostics(self):
        """Generar información de diagnósticos"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        diag_array.header.frame_id = ""
        
        # Diagnóstico de CPU
        cpu_status = DiagnosticStatus()
        cpu_status.name = "System CPU"
        cpu_status.hardware_id = "cpu"
        
        cpu_percent = psutil.cpu_percent(interval=1)
        cpu_status.values.append(KeyValue(key="usage_percent", value=str(cpu_percent)))
        
        if cpu_percent < 70:
            cpu_status.level = DiagnosticStatus.OK
            cpu_status.message = f"CPU usage normal: {cpu_percent:.1f}%"
        elif cpu_percent < 85:
            cpu_status.level = DiagnosticStatus.WARN
            cpu_status.message = f"CPU usage high: {cpu_percent:.1f}%"
        else:
            cpu_status.level = DiagnosticStatus.ERROR
            cpu_status.message = f"CPU usage critical: {cpu_percent:.1f}%"
        
        diag_array.status.append(cpu_status)
        
        # Diagnóstico de memoria
        memory_status = DiagnosticStatus()
        memory_status.name = "System Memory"
        memory_status.hardware_id = "memory"
        
        memory = psutil.virtual_memory()
        memory_status.values.append(KeyValue(key="usage_percent", value=str(memory.percent)))
        memory_status.values.append(KeyValue(key="available_gb", value=str(memory.available / (1024**3))))
        memory_status.values.append(KeyValue(key="total_gb", value=str(memory.total / (1024**3))))
        
        if memory.percent < 75:
            memory_status.level = DiagnosticStatus.OK
            memory_status.message = f"Memory usage normal: {memory.percent:.1f}%"
        elif memory.percent < 90:
            memory_status.level = DiagnosticStatus.WARN
            memory_status.message = f"Memory usage high: {memory.percent:.1f}%"
        else:
            memory_status.level = DiagnosticStatus.ERROR
            memory_status.message = f"Memory usage critical: {memory.percent:.1f}%"
        
        diag_array.status.append(memory_status)
        
        # Diagnóstico de sensores
        for sensor_name, is_online in self.sensor_status.items():
            sensor_status = DiagnosticStatus()
            sensor_status.name = f"Sensor {sensor_name.upper()}"
            sensor_status.hardware_id = sensor_name
            
            if is_online:
                sensor_status.level = DiagnosticStatus.OK
                sensor_status.message = f"{sensor_name} operating normally"
            else:
                sensor_status.level = DiagnosticStatus.ERROR
                sensor_status.message = f"{sensor_name} offline or not responding"
            
            sensor_status.values.append(KeyValue(key="online", value=str(is_online)))
            diag_array.status.append(sensor_status)
        
        # Diagnóstico de ruedas
        for i, wheel_ok in enumerate(self.wheel_status):
            wheel_status = DiagnosticStatus()
            wheel_status.name = f"Wheel {i+1}"
            wheel_status.hardware_id = f"wheel_{i}"
            
            if wheel_ok:
                wheel_status.level = DiagnosticStatus.OK
                wheel_status.message = f"Wheel {i+1} operating normally"
            else:
                wheel_status.level = DiagnosticStatus.ERROR
                wheel_status.message = f"Wheel {i+1} malfunction detected"
            
            wheel_status.values.append(KeyValue(key="operational", value=str(wheel_ok)))
            # Simular temperatura
            temp = 45.0 + i * 0.5  # Simulación simple
            wheel_status.values.append(KeyValue(key="temperature_c", value=str(temp)))
            diag_array.status.append(wheel_status)
        
        return diag_array
    
    def update_robot_status(self, mode=None, ready=None, wheels=None, sensors=None):
        """Método para actualizar estado del robot externamente"""
        if mode is not None:
            self.robot_mode = mode
        if ready is not None:
            self.system_ready = ready
        if wheels is not None:
            self.wheel_status = wheels
        if sensors is not None:
            self.sensor_status.update(sensors)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusService()
    
    # Ejemplo de actualización de estado
    import threading
    import time
    
    def status_updater():
        """Hilo que simula cambios de estado"""
        while rclpy.ok():
            time.sleep(10)
            # Simular cambio ocasional de estado
            if node.robot_mode == "autonomous":
                # Ocasionalmente simular falla de sensor
                import random
                if random.random() < 0.1:  # 10% chance
                    node.sensor_status['camera'] = False
                    node.get_logger().warn("Simulated camera failure")
                else:
                    node.sensor_status['camera'] = True
    
    # Iniciar hilo de actualización
    status_thread = threading.Thread(target=status_updater, daemon=True)
    status_thread.start()
    
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

### Cliente en Python

```python
#!/usr/bin/env python3
# scripts/status_client.py

import rclpy
from rclpy.node import Node
from tadeo_ecar_interfaces.srv import GetRobotStatus
import json

class StatusClient(Node):
    def __init__(self):
        super().__init__('status_client_node')
        
        # Crear cliente de servicio
        self.status_client = self.create_client(GetRobotStatus, 'get_robot_status')
        
        self.get_logger().info('Status client initialized')
    
    def get_robot_status(self, include_diagnostics=True, include_sensors=True, include_hardware=True, timeout=5.0):
        """Solicitar estado completo del robot"""
        
        # Esperar a que el servicio esté disponible
        if not self.status_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('Robot status service not available')
            return None
        
        # Crear request
        request = GetRobotStatus.Request()
        request.include_diagnostics = include_diagnostics
        request.include_sensors = include_sensors
        request.include_hardware = include_hardware
        
        # Enviar request
        try:
            future = self.status_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info('Status retrieved successfully')
                    return response
                else:
                    self.get_logger().error(f'Status request failed: {response.message}')
                    return None
            else:
                self.get_logger().error('Status request timed out')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Exception during status request: {e}')
            return None
    
    def print_robot_status(self, status):
        """Imprimir estado del robot de forma legible"""
        if not status:
            print("No status data to display")
            return
        
        print("=" * 50)
        print("          ECAR ROBOT STATUS")
        print("=" * 50)
        
        # Estado general
        print(f"Robot Mode: {status.robot_mode}")
        print(f"System Ready: {'✓' if status.system_ready else '✗'}")
        print(f"Last Update: {status.last_update.sec}")
        
        # Hardware
        print("\n--- HARDWARE STATUS ---")
        print("Wheels:", end=" ")
        for i, wheel_ok in enumerate(status.wheel_status):
            print(f"W{i+1}:{'✓' if wheel_ok else '✗'}", end=" ")
        print()
        
        print("Steering:", end=" ")
        for i, steering_ok in enumerate(status.steering_status):
            print(f"S{i+1}:{'✓' if steering_ok else '✗'}", end=" ")
        print()
        
        if status.wheel_temperatures:
            print("Wheel Temps:", end=" ")
            for i, temp in enumerate(status.wheel_temperatures):
                print(f"W{i+1}:{temp:.1f}°C", end=" ")
            print()
        
        if status.battery_voltages:
            print("Batteries:", end=" ")
            for i, voltage in enumerate(status.battery_voltages):
                print(f"B{i+1}:{voltage:.1f}V", end=" ")
            print()
        
        # Sensores
        print("\n--- SENSOR STATUS ---")
        sensor_status = [
            ("LiDAR", status.lidar_online),
            ("Camera", status.camera_online),
            ("IMU", status.imu_online),
            ("GPS", status.gps_online)
        ]
        
        for sensor_name, is_online in sensor_status:
            status_icon = "✓" if is_online else "✗"
            print(f"{sensor_name}: {status_icon}")
        
        if status.sensor_errors:
            print("Sensor Errors:")
            for error in status.sensor_errors:
                print(f"  - {error}")
        
        # Diagnósticos
        if status.diagnostics and status.diagnostics.status:
            print("\n--- DIAGNOSTICS ---")
            for diag in status.diagnostics.status:
                level_icons = {0: "✓", 1: "⚠", 2: "✗", 3: "?"}
                level_icon = level_icons.get(diag.level, "?")
                print(f"{level_icon} {diag.name}: {diag.message}")
        
        print("=" * 50)
    
    def get_and_print_status(self):
        """Método de conveniencia para obtener e imprimir estado"""
        status = self.get_robot_status()
        self.print_robot_status(status)
        return status

def main(args=None):
    rclpy.init(args=args)
    client = StatusClient()
    
    # Obtener y mostrar estado
    status = client.get_and_print_status()
    
    # Ejemplo de uso programático
    if status and status.success:
        if status.robot_mode == "autonomous" and status.system_ready:
            client.get_logger().info("Robot ready for autonomous operation")
        else:
            client.get_logger().warn(f"Robot not ready: mode={status.robot_mode}, ready={status.system_ready}")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Servicios del eCar

### Arquitectura de Servicios

```bash
# Servicios de Configuración
/configure_robot          # Configurar parámetros del robot
/calibrate_wheels         # Calibrar sistema 4WD4WS
/set_operation_mode       # Cambiar modo de operación
/load_mission            # Cargar misión de navegación

# Servicios de Estado
/get_robot_status        # Estado completo del robot
/get_system_diagnostics  # Diagnósticos detallados
/get_performance_metrics # Métricas de rendimiento

# Servicios de Control
/emergency_stop          # Parada de emergencia
/reset_system           # Reiniciar sistema
/home_robot             # Volver a posición home
/dock_robot             # Ir a estación de carga

# Servicios de Navegación
/save_waypoint          # Guardar punto de referencia
/clear_costmaps         # Limpiar mapas de costos
/relocalize_robot       # Relocalizar robot en mapa

# Servicios de Mapeo
/save_map               # Guardar mapa actual
/load_map               # Cargar mapa específico
/start_mapping          # Iniciar proceso SLAM
/stop_mapping           # Detener proceso SLAM
```

### Launch File para Servicios

```python
# launch/ecar_services.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Argumentos de launch
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level for service nodes'
    )
    
    # Nodos de servicios
    calibration_service_node = Node(
        package='tadeo_ecar_control',
        executable='calibration_service_node',
        name='calibration_service',
        parameters=[{
            'position_tolerance': 0.01,
            'angle_tolerance': 0.017,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    status_service_node = Node(
        package='tadeo_ecar_safety',
        executable='robot_status_service.py',
        name='robot_status_service',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    emergency_service_node = Node(
        package='tadeo_ecar_safety',
        executable='emergency_service_node',
        name='emergency_service',
        parameters=[{
            'enable_hardware_stop': True,
            'stop_timeout': 5.0,
        }]
    )
    
    navigation_services_node = Node(
        package='tadeo_ecar_navigation',
        executable='navigation_services_node',
        name='navigation_services',
        parameters=[{
            'waypoint_tolerance': 0.2,
            'max_waypoints': 100,
        }]
    )
    
    return LaunchDescription([
        log_level_arg,
        calibration_service_node,
        status_service_node,
        emergency_service_node,
        navigation_services_node
    ])
```

## Manejo de Errores

### Tipos de Errores en Servicios

**1. Servicio No Disponible**
```cpp
if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Service not available after 5 seconds");
    return false;
}
```

**2. Timeout de Request**
```cpp
auto future = client->async_send_request(request);
auto timeout = std::chrono::seconds(10);

if (future.wait_for(timeout) == std::future_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "Service call timed out");
    return false;
}
```

**3. Error en la Respuesta**
```cpp
auto response = future.get();
if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "Service failed: %s", response->message.c_str());
    return false;
}
```

### Manejo Robusto de Errores

```cpp
class RobustServiceClient : public rclcpp::Node
{
public:
    template<typename ServiceT>
    bool callServiceWithRetry(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request,
        typename ServiceT::Response::SharedPtr& response,
        int max_retries = 3,
        double timeout = 5.0)
    {
        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            RCLCPP_INFO(this->get_logger(), "Service call attempt %d/%d", attempt, max_retries);
            
            // Verificar disponibilidad del servicio
            if (!client->wait_for_service(std::chrono::seconds(static_cast<int>(timeout)))) {
                RCLCPP_WARN(this->get_logger(), "Service not available on attempt %d", attempt);
                if (attempt == max_retries) {
                    RCLCPP_ERROR(this->get_logger(), "Service not available after %d attempts", max_retries);
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            
            // Realizar llamada
            try {
                auto future = client->async_send_request(request);
                auto timeout_duration = std::chrono::duration<double>(timeout);
                
                if (future.wait_for(timeout_duration) == std::future_status::ready) {
                    response = future.get();
                    
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Service call successful on attempt %d", attempt);
                        return true;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Service returned failure on attempt %d: %s", 
                                   attempt, response->message.c_str());
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Service call timed out on attempt %d", attempt);
                }
                
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Exception during service call attempt %d: %s", 
                           attempt, e.what());
            }
            
            // Esperar antes del siguiente intento
            if (attempt < max_retries) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Service call failed after %d attempts", max_retries);
        return false;
    }
};
```

### Validación de Requests

```cpp
bool CalibrateWheelsService::validateRequest(const CalibrateWheels::Request::SharedPtr request,
                                            CalibrateWheels::Response::SharedPtr response)
{
    // Validar tipo de calibración
    std::vector<std::string> valid_types = {"steering", "odometry", "full"};
    if (std::find(valid_types.begin(), valid_types.end(), request->calibration_type) == valid_types.end()) {
        response->success = false;
        response->message = "Invalid calibration type. Valid types: steering, odometry, full";
        return false;
    }
    
    // Validar wheel mask
    if (request->wheel_mask < 0 || request->wheel_mask > 0x0F) {
        response->success = false;
        response->message = "Invalid wheel mask. Must be between 0x00 and 0x0F";
        return false;
    }
    
    // Validar ángulo máximo de dirección
    if (request->max_steering_angle <= 0 || request->max_steering_angle > M_PI/2) {
        response->success = false;
        response->message = "Invalid max steering angle. Must be between 0 and π/2 radians";
        return false;
    }
    
    // Verificar que el sistema esté listo para calibración
    if (system_busy_) {
        response->success = false;
        response->message = "System busy. Cannot perform calibration at this time";
        return false;
    }
    
    return true;
}
```

## Servicios Asíncronos

### Cliente Asíncrono en C++

```cpp
class AsyncServiceClient : public rclcpp::Node
{
public:
    AsyncServiceClient() : Node("async_service_client")
    {
        calibration_client_ = this->create_client<CalibrateWheels>("calibrate_wheels");
        
        // Timer para verificar respuestas pendientes
        response_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AsyncServiceClient::checkPendingResponses, this));
    }
    
    void calibrateWheelsAsync(const std::string& calibration_type,
                             std::function<void(bool, const std::string&)> callback)
    {
        auto request = std::make_shared<CalibrateWheels::Request>();
        request->calibration_type = calibration_type;
        request->reset_offsets = true;
        request->wheel_mask = 0x0F;
        request->max_steering_angle = M_PI/4;
        
        // Crear future y almacenar callback
        auto future = calibration_client_->async_send_request(request);
        pending_requests_[future] = callback;
        
        RCLCPP_INFO(this->get_logger(), "Async calibration request sent: %s", calibration_type.c_str());
    }

private:
    void checkPendingResponses()
    {
        for (auto it = pending_requests_.begin(); it != pending_requests_.end();) {
            auto& [future, callback] = *it;
            
            if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                try {
                    auto response = future.get();
                    callback(response->success, response->message);
                    it = pending_requests_.erase(it);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception in async response: %s", e.what());
                    callback(false, "Exception occurred");
                    it = pending_requests_.erase(it);
                }
            } else {
                ++it;
            }
        }
    }
    
    rclcpp::Client<CalibrateWheels>::SharedPtr calibration_client_;
    rclcpp::TimerInterface::SharedPtr response_timer_;
    std::map<std::shared_future<CalibrateWheels::Response::SharedPtr>, 
             std::function<void(bool, const std::string&)>> pending_requests_;
};

// Uso del cliente asíncrono
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<AsyncServiceClient>();
    
    // Realizar calibración asíncrona
    client->calibrateWheelsAsync("full", [client](bool success, const std::string& message) {
        if (success) {
            RCLCPP_INFO(client->get_logger(), "Async calibration completed: %s", message.c_str());
        } else {
            RCLCPP_ERROR(client->get_logger(), "Async calibration failed: %s", message.c_str());
        }
    });
    
    // Continuar con otras tareas mientras se ejecuta la calibración
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
```

### Cliente Asíncrono en Python

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tadeo_ecar_interfaces.srv import CalibrateWheels
import asyncio
from concurrent.futures import ThreadPoolExecutor

class AsyncServiceClient(Node):
    def __init__(self):
        super().__init__('async_service_client')
        
        self.calibration_client = self.create_client(CalibrateWheels, 'calibrate_wheels')
        self.executor = ThreadPoolExecutor(max_workers=4)
        
    async def calibrate_wheels_async(self, calibration_type, reset_offsets=True):
        """Calibración asíncrona usando async/await"""
        
        # Esperar servicio
        if not self.calibration_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Calibration service not available')
        
        # Crear request
        request = CalibrateWheels.Request()
        request.calibration_type = calibration_type
        request.reset_offsets = reset_offsets
        request.wheel_mask = 0x0F
        request.max_steering_angle = 3.14159 / 4
        
        # Enviar request de forma asíncrona
        future = self.calibration_client.call_async(request)
        
        # Usar loop de asyncio para esperar
        loop = asyncio.get_event_loop()
        
        # Función que ejecuta spin_until_future_complete en thread separado
        def spin_until_complete():
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            return future.result()
        
        # Ejecutar en thread pool para no bloquear
        response = await loop.run_in_executor(self.executor, spin_until_complete)
        
        if response is None:
            raise RuntimeError('Service call timed out')
        
        return response
    
    def calibrate_wheels_callback_style(self, calibration_type, callback):
        """Calibración asíncrona usando callbacks"""
        
        if not self.calibration_client.wait_for_service(timeout_sec=5.0):
            callback(False, "Service not available", None)
            return
        
        request = CalibrateWheels.Request()
        request.calibration_type = calibration_type
        request.reset_offsets = True
        request.wheel_mask = 0x0F
        request.max_steering_angle = 3.14159 / 4
        
        future = self.calibration_client.call_async(request)
        
        # Usar thread para no bloquear el hilo principal
        def handle_response():
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            response = future.result()
            
            if response is not None:
                callback(response.success, response.message, response)
            else:
                callback(False, "Service call timed out", None)
        
        # Ejecutar en thread separado
        import threading
        thread = threading.Thread(target=handle_response)
        thread.daemon = True
        thread.start()

async def main_async():
    """Ejemplo de uso con async/await"""
    rclpy.init()
    client = AsyncServiceClient()
    
    try:
        # Calibración asíncrona
        print("Starting async calibration...")
        response = await client.calibrate_wheels_async("full")
        
        if response.success:
            print(f"Calibration successful: {response.message}")
            print(f"Quality: {response.calibration_quality:.2%}")
            print(f"Time: {response.calibration_time:.2f}s")
        else:
            print(f"Calibration failed: {response.message}")
            
    except Exception as e:
        print(f"Error during calibration: {e}")
    
    finally:
        client.destroy_node()
        rclpy.shutdown()

def main_callback():
    """Ejemplo de uso con callbacks"""
    rclpy.init()
    client = AsyncServiceClient()
    
    def calibration_callback(success, message, response):
        if success:
            print(f"Callback: Calibration successful - {message}")
            print(f"Quality: {response.calibration_quality:.2%}")
        else:
            print(f"Callback: Calibration failed - {message}")
    
    # Iniciar calibración con callback
    client.calibrate_wheels_callback_style("steering", calibration_callback)
    
    # Continuar con otras tareas
    print("Calibration started, continuing with other tasks...")
    
    # Spin por un tiempo para que se complete
    import time
    timeout = time.time() + 60  # 60 segundos max
    while rclpy.ok() and time.time() < timeout:
        rclpy.spin_once(client, timeout_sec=0.1)
        time.sleep(0.1)
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Ejemplo con async/await
    asyncio.run(main_async())
    
    # O ejemplo con callbacks
    # main_callback()
```

## Ejercicios Prácticos

### Ejercicio 1: Servicio de Configuración

Crear un servicio para configurar parámetros del eCar:

```cpp
// TODO: Implementar ConfigurationService
class ConfigurationService : public rclcpp::Node
{
public:
    ConfigurationService() : Node("configuration_service_node")
    {
        // TODO: Crear servicio /configure_robot
        // TODO: Validar parámetros de entrada
        // TODO: Aplicar configuración al sistema
        // TODO: Devolver confirmación
    }
};
```

### Ejercicio 2: Cliente de Emergency Stop

Crear un cliente que puede solicitar parada de emergencia:

```python
#!/usr/bin/env python3
# TODO: Implementar EmergencyClient
class EmergencyClient(Node):
    def __init__(self):
        super().__init__('emergency_client_node')
        # TODO: Crear cliente para servicio /emergency_stop
        # TODO: Implementar método emergency_stop()
        # TODO: Manejar timeout y errores
```

### Ejercicio 3: Integración con Sistema

```bash
# 1. Compilar servicios
colcon build --packages-select tadeo_ecar_interfaces tadeo_ecar_control

# 2. Ejecutar servicios
ros2 run tadeo_ecar_control calibration_service_node

# 3. En otra terminal, ejecutar cliente
ros2 run tadeo_ecar_control calibration_client_node

# 4. Verificar servicios disponibles
ros2 service list | grep tadeo_ecar

# 5. Llamar servicio manualmente
ros2 service call /calibrate_wheels tadeo_ecar_interfaces/srv/CalibrateWheels "{calibration_type: 'steering', reset_offsets: true, wheel_mask: 15, max_steering_angle: 0.785}"
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Concepto de servicios**: Comunicación síncrona request-response
2. **Cliente-Servidor**: Patrón desacoplado con confirmación
3. **Definición de servicios**: Servicios estándar y personalizados
4. **Implementación**: Ejemplos completos en C++ y Python
5. **Manejo de errores**: Estrategias robustas para fallos
6. **Servicios asíncronos**: Técnicas para evitar bloqueo

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes la diferencia entre servicios y tópicos
- [ ] Puedes crear servicios personalizados
- [ ] Sabes implementar clientes y servidores robustos
- [ ] Comprendes el manejo de errores y timeouts
- [ ] Puedes usar servicios de forma asíncrona
- [ ] Has probado los ejemplos con el sistema eCar

### Próximo Capítulo

En el Capítulo 6 estudiaremos:
- Acciones en ROS2
- Comunicación asíncrona para tareas largas
- Goals, feedback y results
- Cancelación de acciones
- Sistema de navegación del eCar con acciones

## Referencias

- [ROS2 Services Concepts](https://docs.ros.org/en/humble/Concepts/About-Services.html)
- [Service and Client C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)
- [Service and Client Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Custom Service Interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)