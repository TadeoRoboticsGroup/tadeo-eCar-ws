# Capítulo 15: Sistemas de Seguridad para eCar 4WD4WS

## Tabla de Contenidos

1. [Introducción a Seguridad Robótica](#introducción-a-seguridad-robótica)
2. [Arquitectura de Seguridad](#arquitectura-de-seguridad)
3. [Sistemas de Parada de Emergencia](#sistemas-de-parada-de-emergencia)
4. [Monitoreo de Estado](#monitoreo-de-estado)
5. [Gestión de Fallos](#gestión-de-fallos)
6. [Protocolos de Seguridad](#protocolos-de-seguridad)
7. [Implementación en ROS2](#implementación-en-ros2)
8. [Testing y Certificación](#testing-y-certificación)

## Introducción a Seguridad Robótica

### ¿Por qué Seguridad en Robótica?

La seguridad en robótica es crítica, especialmente en robots móviles como el eCar que operan en entornos compartidos con humanos. Un fallo puede resultar en:

```
Riesgos de Seguridad:
✗ Colisiones con personas u objetos
✗ Comportamientos impredecibles
✗ Daños al equipo o infraestructura
✗ Pérdida de datos críticos
✗ Violación de protocolos de seguridad

Beneficios de Sistemas de Seguridad:
✓ Operación confiable y predecible
✓ Protección de personas y equipos
✓ Cumplimiento de normativas
✓ Reducción de costos de mantenimiento
✓ Mayor aceptación del sistema
```

### Niveles de Seguridad

**1. Seguridad Funcional (Functional Safety)**
- Sistema opera correctamente bajo condiciones normales
- Manejo apropiado de entradas válidas
- Comportamiento determinístico

**2. Seguridad de Fallo (Fail-Safe)**
- Sistema entra en estado seguro ante fallos
- Detección temprana de anomalías
- Degradación controlada del servicio

**3. Seguridad Física (Physical Safety)**
- Protección contra daños físicos
- Detección de colisiones
- Paradas de emergencia

**4. Seguridad Cibernética (Cybersecurity)**
- Protección contra acceso no autorizado
- Comunicaciones seguras
- Integridad de datos

### Características Específicas del eCar 4WD4WS

**Ventajas de Seguridad:**
```cpp
// Capacidades omnidireccionales permiten maniobras de escape
void emergencyManeuver() {
    // Movimiento lateral inmediato sin necesidad de girar
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = -2.0;  // Escape lateral rápido
    cmd_vel.angular.z = 0.0;
}

// Rotación in-situ para mejor percepción
void emergencyScan() {
    // Girar en el lugar para analizar entorno
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 1.0;  // Scan rotacional
}
```

**Consideraciones Especiales:**
- Mayor complejidad mecánica (4 motores + 4 servos)
- Múltiples modos de fallo independientes
- Necesidad de coordinación entre sistemas de tracción y dirección

## Arquitectura de Seguridad

### Diseño de Sistemas de Seguridad

```
                 Safety Architecture eCar 4WD4WS
                              |
       ┌─────────────────────────────────────────────────────────┐
       |              Safety Supervisor                          |
       |   (Monitorea todos los subsistemas)                     |
       └─────────────────┬───────────────────────────────────────┘
                         |
       ┌─────────────────┼───────────────────────────────────────┐
       |                 |                                       |
  ┌────▼────┐    ┌───────▼────┐    ┌──────────▼────┐    ┌───────▼───┐
  │Emergency│    │Health      │    │Fault          │    │Watchdog   │
  │Stop     │    │Monitor     │    │Detection      │    │Timer      │
  └─────────┘    └────────────┘    └───────────────┘    └───────────┘
       |              |                      |                  |
  ┌────▼────┐   ┌─────▼────┐         ┌──────▼──────┐     ┌──────▼──────┐
  │Hardware │   │Software  │         │Sensor       │     │Communication│
  │E-Stop   │   │Monitor   │         │Validation   │     │Monitor      │
  └─────────┘   └──────────┘         └─────────────┘     └─────────────┘
```

### Principios de Diseño Seguro

**1. Redundancia**
```cpp
// Múltiples sensores para misma función
struct RedundantSensors {
    sensor_msgs::msg::LaserScan primary_lidar;
    sensor_msgs::msg::LaserScan secondary_lidar;
    std::vector<sensor_msgs::msg::Range> ultrasonic_array;
    sensor_msgs::msg::Image camera_feed;
    
    bool validateObstacleDetection() {
        // Verificar consistencia entre sensores
        return (primary_lidar.ranges.size() > 0 && 
                secondary_lidar.ranges.size() > 0 &&
                ultrasonic_array.size() >= 4);
    }
};
```

**2. Diversidad**
```cpp
// Diferentes tecnologías para misma función
class DiverseObstacleDetection {
private:
    LidarObstacleDetector lidar_detector_;      // Tecnología láser
    UltrasonicDetector ultrasonic_detector_;    // Tecnología ultrasónica
    VisionObstacleDetector vision_detector_;    // Tecnología visual
    
public:
    bool isPathClear() {
        // Al menos 2 de 3 tecnologías deben confirmar
        int confirmations = 0;
        if (lidar_detector_.isPathClear()) confirmations++;
        if (ultrasonic_detector_.isPathClear()) confirmations++;
        if (vision_detector_.isPathClear()) confirmations++;
        
        return confirmations >= 2;
    }
};
```

**3. Fail-Safe Design**
```cpp
// Comportamiento seguro por defecto
class SafeDefaultBehavior {
public:
    geometry_msgs::msg::Twist getSafeCommand() {
        geometry_msgs::msg::Twist safe_cmd;
        // Por defecto: parado
        safe_cmd.linear.x = 0.0;
        safe_cmd.linear.y = 0.0;
        safe_cmd.angular.z = 0.0;
        return safe_cmd;
    }
    
    void enterSafeMode() {
        // Deshabilitar movimiento
        publishStopCommand();
        // Activar luces de emergencia
        activateEmergencyLights();
        // Notificar operadores
        sendSafetyAlert();
    }
};
```

## Sistemas de Parada de Emergencia

### Emergency Stop Hardware

```cpp
// src/emergency_stop_manager.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

class EmergencyStopManager : public rclcpp::Node
{
public:
    EmergencyStopManager() : Node("emergency_stop_manager")
    {
        // Suscriptores para señales de E-Stop
        hardware_estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware_estop", 10,
            std::bind(&EmergencyStopManager::hardwareEstopCallback, this, std::placeholders::_1));
        
        software_estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/software_estop", 10,
            std::bind(&EmergencyStopManager::softwareEstopCallback, this, std::placeholders::_1));
        
        joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&EmergencyStopManager::joystickCallback, this, std::placeholders::_1));
        
        // Suscriptor para comandos de velocidad
        cmd_vel_input_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_input", 10,
            std::bind(&EmergencyStopManager::cmdVelCallback, this, std::placeholders::_1));
        
        // Publicadores
        cmd_vel_output_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        estop_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/estop_status", 10);
        
        diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);
        
        // Parámetros
        this->declare_parameter("estop_timeout", 1.0);  // segundos
        this->declare_parameter("enable_joystick_estop", true);
        this->declare_parameter("joystick_estop_button", 0);  // Botón A
        
        // Timer para monitoreo continuo
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&EmergencyStopManager::monitorSafety, this));
        
        // Estado inicial
        hardware_estop_active_ = false;
        software_estop_active_ = false;
        joystick_estop_active_ = false;
        last_heartbeat_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Emergency Stop Manager initialized");
    }

private:
    void hardwareEstopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        hardware_estop_active_ = msg->data;
        last_heartbeat_ = this->now();
        
        if (hardware_estop_active_) {
            RCLCPP_ERROR(this->get_logger(), "HARDWARE EMERGENCY STOP ACTIVATED");
            triggerEmergencyStop("Hardware E-Stop");
        } else {
            RCLCPP_INFO(this->get_logger(), "Hardware E-Stop released");
        }
        
        publishEstopStatus();
    }
    
    void softwareEstopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        software_estop_active_ = msg->data;
        
        if (software_estop_active_) {
            RCLCPP_WARN(this->get_logger(), "SOFTWARE EMERGENCY STOP ACTIVATED");
            triggerEmergencyStop("Software E-Stop");
        } else {
            RCLCPP_INFO(this->get_logger(), "Software E-Stop released");
        }
        
        publishEstopStatus();
    }
    
    void joystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!this->get_parameter("enable_joystick_estop").as_bool()) {
            return;
        }
        
        int estop_button = this->get_parameter("joystick_estop_button").as_int();
        
        if (estop_button < static_cast<int>(msg->buttons.size())) {
            bool button_pressed = msg->buttons[estop_button] == 1;
            
            if (button_pressed && !joystick_estop_active_) {
                joystick_estop_active_ = true;
                RCLCPP_WARN(this->get_logger(), "JOYSTICK EMERGENCY STOP ACTIVATED");
                triggerEmergencyStop("Joystick E-Stop");
            } else if (!button_pressed && joystick_estop_active_) {
                joystick_estop_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Joystick E-Stop released");
            }
        }
        
        publishEstopStatus();
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Filtrar comandos de velocidad según estado de E-Stop
        if (isEmergencyStopActive()) {
            // E-Stop activo: publicar comando de parada
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.linear.y = 0.0;
            stop_cmd.linear.z = 0.0;
            stop_cmd.angular.x = 0.0;
            stop_cmd.angular.y = 0.0;
            stop_cmd.angular.z = 0.0;
            
            cmd_vel_output_pub_->publish(stop_cmd);
            
            RCLCPP_DEBUG(this->get_logger(), "Command blocked by E-Stop");
        } else {
            // E-Stop inactivo: pasar comando sin modificar
            cmd_vel_output_pub_->publish(*msg);
        }
    }
    
    void monitorSafety()
    {
        // 1. Verificar timeout de heartbeat
        auto now = this->now();
        double timeout = this->get_parameter("estop_timeout").as_double();
        
        if ((now - last_heartbeat_).seconds() > timeout) {
            RCLCPP_ERROR(this->get_logger(), 
                        "EMERGENCY: Hardware heartbeat timeout (%.2fs)", 
                        (now - last_heartbeat_).seconds());
            
            // Activar E-Stop por timeout
            hardware_estop_active_ = true;
            triggerEmergencyStop("Heartbeat Timeout");
        }
        
        // 2. Publicar diagnósticos
        publishDiagnostics();
        
        // 3. Verificar estado general del sistema
        performSystemHealthCheck();
    }
    
    void triggerEmergencyStop(const std::string& reason)
    {
        // Log crítico
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP TRIGGERED: %s", reason.c_str());
        
        // Publicar comando de parada inmediata
        geometry_msgs::msg::Twist emergency_stop;
        // Todos los valores a cero (ya están por defecto)
        cmd_vel_output_pub_->publish(emergency_stop);
        
        // Enviar múltiples comandos de parada para asegurar recepción
        for (int i = 0; i < 5; ++i) {
            cmd_vel_output_pub_->publish(emergency_stop);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Notificar a otros sistemas
        notifyEmergencyStop(reason);
    }
    
    void notifyEmergencyStop(const std::string& reason)
    {
        // Publicar en topic de eventos de seguridad
        // (Implementar mensaje personalizado para eventos de seguridad)
        
        // Log adicional para auditoría
        RCLCPP_ERROR(this->get_logger(), 
                    "SAFETY EVENT: Emergency stop activated at %ld due to: %s", 
                    this->now().nanoseconds(), reason.c_str());
    }
    
    bool isEmergencyStopActive()
    {
        return hardware_estop_active_ || software_estop_active_ || joystick_estop_active_;
    }
    
    void publishEstopStatus()
    {
        std_msgs::msg::Bool status_msg;
        status_msg.data = isEmergencyStopActive();
        estop_status_pub_->publish(status_msg);
    }
    
    void publishDiagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = this->now();
        
        // Diagnóstico de E-Stop Hardware
        diagnostic_msgs::msg::DiagnosticStatus hw_estop_diag;
        hw_estop_diag.name = "Hardware E-Stop";
        hw_estop_diag.hardware_id = "estop_hw_001";
        
        if (hardware_estop_active_) {
            hw_estop_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            hw_estop_diag.message = "Hardware Emergency Stop ACTIVE";
        } else {
            hw_estop_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            hw_estop_diag.message = "Hardware Emergency Stop ready";
        }
        
        // Diagnóstico de timeout
        auto heartbeat_age = (this->now() - last_heartbeat_).seconds();
        diagnostic_msgs::msg::KeyValue heartbeat_kv;
        heartbeat_kv.key = "heartbeat_age";
        heartbeat_kv.value = std::to_string(heartbeat_age);
        hw_estop_diag.values.push_back(heartbeat_kv);
        
        diag_array.status.push_back(hw_estop_diag);
        
        // Diagnóstico de E-Stop Software
        diagnostic_msgs::msg::DiagnosticStatus sw_estop_diag;
        sw_estop_diag.name = "Software E-Stop";
        sw_estop_diag.hardware_id = "estop_sw_001";
        
        if (software_estop_active_) {
            sw_estop_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            sw_estop_diag.message = "Software Emergency Stop ACTIVE";
        } else {
            sw_estop_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            sw_estop_diag.message = "Software Emergency Stop ready";
        }
        
        diag_array.status.push_back(sw_estop_diag);
        
        diagnostic_pub_->publish(diag_array);
    }
    
    void performSystemHealthCheck()
    {
        // Verificaciones adicionales de salud del sistema
        
        // 1. Verificar memoria disponible
        checkMemoryUsage();
        
        // 2. Verificar CPU usage
        checkCPUUsage();
        
        // 3. Verificar conectividad de red
        checkNetworkConnectivity();
        
        // 4. Verificar estado de sensores críticos
        checkCriticalSensors();
    }
    
    void checkMemoryUsage()
    {
        // Implementar verificación de memoria
        // Si memoria < umbral crítico -> activar E-Stop software
    }
    
    void checkCPUUsage()
    {
        // Implementar verificación de CPU
        // Si CPU > 95% por tiempo prolongado -> alerta
    }
    
    void checkNetworkConnectivity()
    {
        // Verificar conectividad con sistemas críticos
    }
    
    void checkCriticalSensors()
    {
        // Verificar que sensores críticos estén respondiendo
    }
    
    // Miembros
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hardware_estop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr software_estop_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_input_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_output_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
    
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    
    // Estado
    bool hardware_estop_active_;
    bool software_estop_active_;
    bool joystick_estop_active_;
    rclcpp::Time last_heartbeat_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmergencyStopManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Monitoreo de Estado

### Health Monitor System

```cpp
// src/health_monitor.cpp
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

class HealthMonitor : public rclcpp::Node
{
public:
    HealthMonitor() : Node("health_monitor")
    {
        // Suscriptores para monitoreo
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state", 10,
            std::bind(&HealthMonitor::batteryCallback, this, std::placeholders::_1));
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&HealthMonitor::cmdVelCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&HealthMonitor::odomCallback, this, std::placeholders::_1));
        
        temperature_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motor_temperature", 10,
            std::bind(&HealthMonitor::temperatureCallback, this, std::placeholders::_1));
        
        // Publicadores
        health_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/health_diagnostics", 10);
        
        alert_pub_ = this->create_publisher<std_msgs::msg::String>("/health_alerts", 10);
        
        // Timer para chequeos periódicos
        health_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HealthMonitor::performHealthChecks, this));
        
        // Parámetros
        this->declare_parameter("battery_critical_voltage", 22.0);  // 24V system
        this->declare_parameter("battery_low_voltage", 23.0);
        this->declare_parameter("motor_critical_temp", 80.0);  // Celsius
        this->declare_parameter("motor_warning_temp", 70.0);
        this->declare_parameter("max_cmd_vel_age", 2.0);  // segundos
        this->declare_parameter("max_odom_age", 1.0);
        
        // Inicializar estado
        initializeHealthState();
        
        RCLCPP_INFO(this->get_logger(), "Health Monitor initialized");
    }

private:
    struct HealthState {
        // Batería
        double battery_voltage = 0.0;
        double battery_current = 0.0;
        double battery_percentage = 0.0;
        rclcpp::Time last_battery_update;
        
        // Motores
        double motor_temperature = 0.0;
        rclcpp::Time last_temperature_update;
        
        // Comunicación
        rclcpp::Time last_cmd_vel_update;
        rclcpp::Time last_odom_update;
        
        // Rendimiento
        double cpu_usage = 0.0;
        double memory_usage = 0.0;
        double disk_usage = 0.0;
        
        // Contadores de errores
        int battery_low_count = 0;
        int overtemp_count = 0;
        int communication_timeout_count = 0;
    };
    
    void initializeHealthState()
    {
        auto now = this->now();
        health_state_.last_battery_update = now;
        health_state_.last_temperature_update = now;
        health_state_.last_cmd_vel_update = now;
        health_state_.last_odom_update = now;
    }
    
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        health_state_.battery_voltage = msg->voltage;
        health_state_.battery_current = msg->current;
        health_state_.battery_percentage = msg->percentage * 100.0;
        health_state_.last_battery_update = this->now();
        
        // Verificar niveles críticos
        checkBatteryHealth();
    }
    
    void temperatureCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        health_state_.motor_temperature = msg->data;
        health_state_.last_temperature_update = this->now();
        
        checkTemperatureHealth();
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        health_state_.last_cmd_vel_update = this->now();
        
        // Verificar comandos válidos
        if (std::isnan(msg->linear.x) || std::isnan(msg->linear.y) || 
            std::isnan(msg->angular.z)) {
            RCLCPP_WARN(this->get_logger(), "Invalid cmd_vel received (NaN values)");
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        health_state_.last_odom_update = this->now();
        
        // Verificar datos de odometría válidos
        if (std::isnan(msg->pose.pose.position.x) || 
            std::isnan(msg->pose.pose.position.y)) {
            RCLCPP_WARN(this->get_logger(), "Invalid odometry received (NaN values)");
        }
    }
    
    void performHealthChecks()
    {
        auto now = this->now();
        
        // 1. Verificar timeouts de comunicación
        checkCommunicationTimeouts(now);
        
        // 2. Verificar rendimiento del sistema
        checkSystemPerformance();
        
        // 3. Verificar integridad de datos
        checkDataIntegrity();
        
        // 4. Publicar diagnósticos
        publishHealthDiagnostics();
        
        // 5. Verificar patrones de fallo
        analyzeFailurePatterns();
    }
    
    void checkBatteryHealth()
    {
        double critical_voltage = this->get_parameter("battery_critical_voltage").as_double();
        double low_voltage = this->get_parameter("battery_low_voltage").as_double();
        
        if (health_state_.battery_voltage < critical_voltage) {
            health_state_.battery_low_count++;
            
            if (health_state_.battery_low_count >= 3) {  // 3 lecturas consecutivas
                RCLCPP_ERROR(this->get_logger(), 
                           "CRITICAL: Battery voltage critically low: %.2fV", 
                           health_state_.battery_voltage);
                
                publishAlert("BATTERY_CRITICAL", 
                           "Battery voltage " + std::to_string(health_state_.battery_voltage) + 
                           "V below critical threshold " + std::to_string(critical_voltage) + "V");
            }
        } else if (health_state_.battery_voltage < low_voltage) {
            RCLCPP_WARN(this->get_logger(), 
                       "WARNING: Battery voltage low: %.2fV", 
                       health_state_.battery_voltage);
            
            publishAlert("BATTERY_LOW", 
                       "Battery voltage " + std::to_string(health_state_.battery_voltage) + 
                       "V below warning threshold " + std::to_string(low_voltage) + "V");
        } else {
            health_state_.battery_low_count = 0;  // Reset counter
        }
    }
    
    void checkTemperatureHealth()
    {
        double critical_temp = this->get_parameter("motor_critical_temp").as_double();
        double warning_temp = this->get_parameter("motor_warning_temp").as_double();
        
        if (health_state_.motor_temperature > critical_temp) {
            health_state_.overtemp_count++;
            
            RCLCPP_ERROR(this->get_logger(), 
                        "CRITICAL: Motor temperature too high: %.1f°C", 
                        health_state_.motor_temperature);
            
            publishAlert("MOTOR_OVERTEMP", 
                       "Motor temperature " + std::to_string(health_state_.motor_temperature) + 
                       "°C exceeds critical threshold " + std::to_string(critical_temp) + "°C");
            
            if (health_state_.overtemp_count >= 5) {
                // Activar protección térmica
                triggerThermalProtection();
            }
        } else if (health_state_.motor_temperature > warning_temp) {
            RCLCPP_WARN(this->get_logger(), 
                       "WARNING: Motor temperature elevated: %.1f°C", 
                       health_state_.motor_temperature);
        } else {
            health_state_.overtemp_count = 0;  // Reset counter
        }
    }
    
    void checkCommunicationTimeouts(const rclcpp::Time& now)
    {
        double max_cmd_vel_age = this->get_parameter("max_cmd_vel_age").as_double();
        double max_odom_age = this->get_parameter("max_odom_age").as_double();
        
        // Verificar timeout de cmd_vel
        double cmd_vel_age = (now - health_state_.last_cmd_vel_update).seconds();
        if (cmd_vel_age > max_cmd_vel_age) {
            health_state_.communication_timeout_count++;
            
            RCLCPP_WARN(this->get_logger(), 
                       "Communication timeout: cmd_vel age %.2fs > %.2fs", 
                       cmd_vel_age, max_cmd_vel_age);
            
            publishAlert("CMD_VEL_TIMEOUT", 
                       "No cmd_vel received for " + std::to_string(cmd_vel_age) + " seconds");
        }
        
        // Verificar timeout de odometría
        double odom_age = (now - health_state_.last_odom_update).seconds();
        if (odom_age > max_odom_age) {
            RCLCPP_WARN(this->get_logger(), 
                       "Communication timeout: odometry age %.2fs > %.2fs", 
                       odom_age, max_odom_age);
            
            publishAlert("ODOM_TIMEOUT", 
                       "No odometry received for " + std::to_string(odom_age) + " seconds");
        }
    }
    
    void checkSystemPerformance()
    {
        // Leer estadísticas del sistema
        updateSystemStats();
        
        // Verificar uso de CPU
        if (health_state_.cpu_usage > 90.0) {
            RCLCPP_WARN(this->get_logger(), 
                       "High CPU usage: %.1f%%", health_state_.cpu_usage);
            
            publishAlert("HIGH_CPU_USAGE", 
                       "CPU usage " + std::to_string(health_state_.cpu_usage) + "% > 90%");
        }
        
        // Verificar uso de memoria
        if (health_state_.memory_usage > 85.0) {
            RCLCPP_WARN(this->get_logger(), 
                       "High memory usage: %.1f%%", health_state_.memory_usage);
            
            publishAlert("HIGH_MEMORY_USAGE", 
                       "Memory usage " + std::to_string(health_state_.memory_usage) + "% > 85%");
        }
        
        // Verificar espacio en disco
        if (health_state_.disk_usage > 90.0) {
            RCLCPP_WARN(this->get_logger(), 
                       "Low disk space: %.1f%% used", health_state_.disk_usage);
            
            publishAlert("LOW_DISK_SPACE", 
                       "Disk usage " + std::to_string(health_state_.disk_usage) + "% > 90%");
        }
    }
    
    void updateSystemStats()
    {
        // Implementar lectura de /proc/stat, /proc/meminfo, etc.
        // Por simplicidad, valores simulados
        health_state_.cpu_usage = 45.0;   // Implementar lectura real
        health_state_.memory_usage = 60.0; // Implementar lectura real
        health_state_.disk_usage = 25.0;   // Implementar lectura real
    }
    
    void checkDataIntegrity()
    {
        // Verificar integridad de datos críticos
        
        // 1. Verificar rangos válidos
        if (health_state_.battery_voltage > 30.0 || health_state_.battery_voltage < 0.0) {
            publishAlert("INVALID_BATTERY_DATA", 
                       "Battery voltage out of valid range: " + 
                       std::to_string(health_state_.battery_voltage) + "V");
        }
        
        if (health_state_.motor_temperature > 150.0 || health_state_.motor_temperature < -20.0) {
            publishAlert("INVALID_TEMPERATURE_DATA", 
                       "Motor temperature out of valid range: " + 
                       std::to_string(health_state_.motor_temperature) + "°C");
        }
    }
    
    void triggerThermalProtection()
    {
        RCLCPP_ERROR(this->get_logger(), 
                    "THERMAL PROTECTION ACTIVATED - Reducing motor power");
        
        // Publicar comando de protección térmica
        publishAlert("THERMAL_PROTECTION_ACTIVE", 
                   "Motor power reduced due to overtemperature condition");
    }
    
    void analyzeFailurePatterns()
    {
        // Análisis de patrones de fallo para mantenimiento predictivo
        
        if (health_state_.battery_low_count > 10) {
            publishAlert("BATTERY_DEGRADATION_PATTERN", 
                       "Frequent battery low warnings may indicate battery degradation");
        }
        
        if (health_state_.overtemp_count > 15) {
            publishAlert("THERMAL_PATTERN", 
                       "Frequent overtemperature warnings may indicate cooling system issues");
        }
        
        if (health_state_.communication_timeout_count > 20) {
            publishAlert("COMMUNICATION_PATTERN", 
                       "Frequent communication timeouts may indicate network issues");
        }
    }
    
    void publishAlert(const std::string& alert_type, const std::string& message)
    {
        std_msgs::msg::String alert_msg;
        alert_msg.data = "[" + alert_type + "] " + message;
        alert_pub_->publish(alert_msg);
        
        // Log para auditoría
        RCLCPP_WARN(this->get_logger(), "HEALTH ALERT: %s", alert_msg.data.c_str());
    }
    
    void publishHealthDiagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = this->now();
        
        // Diagnóstico de batería
        diagnostic_msgs::msg::DiagnosticStatus battery_diag;
        battery_diag.name = "Battery System";
        battery_diag.hardware_id = "battery_001";
        
        double critical_voltage = this->get_parameter("battery_critical_voltage").as_double();
        
        if (health_state_.battery_voltage < critical_voltage) {
            battery_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            battery_diag.message = "Battery voltage critically low";
        } else if (health_state_.battery_voltage < this->get_parameter("battery_low_voltage").as_double()) {
            battery_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            battery_diag.message = "Battery voltage low";
        } else {
            battery_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            battery_diag.message = "Battery system healthy";
        }
        
        // Agregar valores de batería
        diagnostic_msgs::msg::KeyValue voltage_kv;
        voltage_kv.key = "voltage";
        voltage_kv.value = std::to_string(health_state_.battery_voltage);
        battery_diag.values.push_back(voltage_kv);
        
        diagnostic_msgs::msg::KeyValue current_kv;
        current_kv.key = "current";
        current_kv.value = std::to_string(health_state_.battery_current);
        battery_diag.values.push_back(current_kv);
        
        diagnostic_msgs::msg::KeyValue percentage_kv;
        percentage_kv.key = "percentage";
        percentage_kv.value = std::to_string(health_state_.battery_percentage);
        battery_diag.values.push_back(percentage_kv);
        
        diag_array.status.push_back(battery_diag);
        
        // Diagnóstico de temperatura
        diagnostic_msgs::msg::DiagnosticStatus temp_diag;
        temp_diag.name = "Motor Temperature";
        temp_diag.hardware_id = "motor_temp_001";
        
        double critical_temp = this->get_parameter("motor_critical_temp").as_double();
        
        if (health_state_.motor_temperature > critical_temp) {
            temp_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            temp_diag.message = "Motor temperature critical";
        } else if (health_state_.motor_temperature > this->get_parameter("motor_warning_temp").as_double()) {
            temp_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            temp_diag.message = "Motor temperature elevated";
        } else {
            temp_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            temp_diag.message = "Motor temperature normal";
        }
        
        diagnostic_msgs::msg::KeyValue temp_kv;
        temp_kv.key = "temperature";
        temp_kv.value = std::to_string(health_state_.motor_temperature);
        temp_diag.values.push_back(temp_kv);
        
        diag_array.status.push_back(temp_diag);
        
        // Publicar diagnósticos
        health_pub_->publish(diag_array);
    }
    
    // Miembros
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr temperature_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
    
    rclcpp::TimerBase::SharedPtr health_timer_;
    
    HealthState health_state_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HealthMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Protocolos de Seguridad

### Safety Protocol Manager

```python
#!/usr/bin/env python3
# src/safety_protocols.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, BatteryState
from diagnostic_msgs.msg import DiagnosticArray
import json
import time
from enum import Enum

class SafetyLevel(Enum):
    NORMAL = 0
    CAUTION = 1
    WARNING = 2
    CRITICAL = 3
    EMERGENCY = 4

class SafetyProtocolManager(Node):
    def __init__(self):
        super().__init__('safety_protocol_manager')
        
        # Suscriptores
        self.health_sub = self.create_subscription(
            String, '/health_alerts', self.health_alert_callback, 10)
        
        self.estop_sub = self.create_subscription(
            Bool, '/estop_status', self.estop_callback, 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        
        # Publicadores
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10)
        self.protocol_action_pub = self.create_publisher(String, '/protocol_actions', 10)
        self.cmd_vel_override_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        
        # Estado del sistema
        self.current_safety_level = SafetyLevel.NORMAL
        self.active_protocols = set()
        self.safety_history = []
        
        # Protocolos definidos
        self.define_safety_protocols()
        
        # Timer para evaluación continua
        self.evaluation_timer = self.create_wall_timer(0.5, self.evaluate_safety_level)
        
        # Parámetros
        self.declare_parameter('min_obstacle_distance', 0.5)
        self.declare_parameter('critical_battery_level', 20.0)
        self.declare_parameter('max_protocol_escalation_time', 30.0)
        
        self.get_logger().info('Safety Protocol Manager initialized')
    
    def define_safety_protocols(self):
        """Definir protocolos de seguridad estructurados"""
        
        self.safety_protocols = {
            'PROXIMITY_ALERT': {
                'level': SafetyLevel.CAUTION,
                'actions': ['reduce_speed', 'increase_sensor_frequency'],
                'escalation_time': 10.0,
                'escalation_protocol': 'PROXIMITY_WARNING'
            },
            
            'PROXIMITY_WARNING': {
                'level': SafetyLevel.WARNING,
                'actions': ['stop_forward_movement', 'activate_warning_lights'],
                'escalation_time': 5.0,
                'escalation_protocol': 'PROXIMITY_CRITICAL'
            },
            
            'PROXIMITY_CRITICAL': {
                'level': SafetyLevel.CRITICAL,
                'actions': ['emergency_stop', 'sound_alarm'],
                'escalation_time': 0.0,
                'escalation_protocol': None
            },
            
            'BATTERY_LOW': {
                'level': SafetyLevel.WARNING,
                'actions': ['limit_speed', 'find_charging_station'],
                'escalation_time': 60.0,
                'escalation_protocol': 'BATTERY_CRITICAL'
            },
            
            'BATTERY_CRITICAL': {
                'level': SafetyLevel.CRITICAL,
                'actions': ['emergency_return_to_base', 'conserve_power'],
                'escalation_time': 30.0,
                'escalation_protocol': 'POWER_EMERGENCY'
            },
            
            'POWER_EMERGENCY': {
                'level': SafetyLevel.EMERGENCY,
                'actions': ['immediate_shutdown_sequence', 'send_mayday'],
                'escalation_time': 0.0,
                'escalation_protocol': None
            },
            
            'SYSTEM_FAULT': {
                'level': SafetyLevel.WARNING,
                'actions': ['diagnostic_mode', 'limit_functionality'],
                'escalation_time': 15.0,
                'escalation_protocol': 'SYSTEM_CRITICAL'
            },
            
            'SYSTEM_CRITICAL': {
                'level': SafetyLevel.CRITICAL,
                'actions': ['safe_mode_only', 'request_maintenance'],
                'escalation_time': 0.0,
                'escalation_protocol': None
            },
            
            'COMMUNICATION_LOSS': {
                'level': SafetyLevel.WARNING,
                'actions': ['autonomous_safe_operation', 'attempt_reconnection'],
                'escalation_time': 20.0,
                'escalation_protocol': 'COMPLETE_ISOLATION'
            },
            
            'COMPLETE_ISOLATION': {
                'level': SafetyLevel.EMERGENCY,
                'actions': ['return_to_safe_zone', 'wait_for_manual_recovery'],
                'escalation_time': 0.0,
                'escalation_protocol': None
            }
        }
    
    def health_alert_callback(self, msg):
        """Procesar alertas de salud del sistema"""
        
        alert_data = msg.data
        self.get_logger().info(f'Health alert received: {alert_data}')
        
        # Parsear tipo de alerta
        if '[BATTERY_CRITICAL]' in alert_data:
            self.activate_protocol('BATTERY_CRITICAL')
        elif '[BATTERY_LOW]' in alert_data:
            self.activate_protocol('BATTERY_LOW')
        elif '[MOTOR_OVERTEMP]' in alert_data:
            self.activate_protocol('SYSTEM_FAULT')
        elif '[CMD_VEL_TIMEOUT]' in alert_data or '[ODOM_TIMEOUT]' in alert_data:
            self.activate_protocol('COMMUNICATION_LOSS')
        elif '[HIGH_CPU_USAGE]' in alert_data or '[HIGH_MEMORY_USAGE]' in alert_data:
            self.activate_protocol('SYSTEM_FAULT')
    
    def estop_callback(self, msg):
        """Procesar estado de emergency stop"""
        
        if msg.data:  # E-Stop activado
            self.get_logger().error('Emergency Stop activated - entering emergency mode')
            self.current_safety_level = SafetyLevel.EMERGENCY
            self.execute_emergency_procedures()
        else:
            self.get_logger().info('Emergency Stop released')
            # No cambiar automáticamente el nivel - requiere evaluación manual
    
    def scan_callback(self, msg):
        """Monitorear proximidad de obstáculos"""
        
        min_distance = self.get_parameter('min_obstacle_distance').value
        
        # Encontrar distancia mínima en sector frontal
        front_sector_size = len(msg.ranges) // 4  # 90 grados frontales
        front_start = len(msg.ranges) // 2 - front_sector_size // 2
        front_end = front_start + front_sector_size
        
        front_ranges = msg.ranges[front_start:front_end]
        valid_ranges = [r for r in front_ranges if msg.range_min <= r <= msg.range_max]
        
        if valid_ranges:
            min_front_distance = min(valid_ranges)
            
            if min_front_distance < min_distance * 0.5:  # Crítico
                self.activate_protocol('PROXIMITY_CRITICAL')
            elif min_front_distance < min_distance:  # Warning
                self.activate_protocol('PROXIMITY_WARNING')
            elif min_front_distance < min_distance * 2:  # Caution
                self.activate_protocol('PROXIMITY_ALERT')
    
    def battery_callback(self, msg):
        """Monitorear estado de batería"""
        
        battery_percentage = msg.percentage * 100
        critical_level = self.get_parameter('critical_battery_level').value
        
        if battery_percentage < critical_level * 0.5:  # 10% if critical is 20%
            self.activate_protocol('BATTERY_CRITICAL')
        elif battery_percentage < critical_level:
            self.activate_protocol('BATTERY_LOW')
    
    def activate_protocol(self, protocol_name):
        """Activar protocolo de seguridad específico"""
        
        if protocol_name not in self.safety_protocols:
            self.get_logger().error(f'Unknown safety protocol: {protocol_name}')
            return
        
        protocol = self.safety_protocols[protocol_name]
        
        # Verificar si ya está activo
        if protocol_name in self.active_protocols:
            return
        
        self.get_logger().warn(f'Activating safety protocol: {protocol_name}')
        
        # Agregar a protocolos activos
        self.active_protocols.add(protocol_name)
        
        # Ejecutar acciones del protocolo
        self.execute_protocol_actions(protocol_name, protocol['actions'])
        
        # Actualizar nivel de seguridad
        if protocol['level'].value > self.current_safety_level.value:
            self.current_safety_level = protocol['level']
        
        # Programar escalación si aplica
        if protocol['escalation_protocol']:
            escalation_time = protocol['escalation_time']
            self.create_timer(escalation_time, 
                            lambda: self.escalate_protocol(protocol_name))
        
        # Registrar en historial
        self.record_safety_event(protocol_name, 'ACTIVATED')
        
        # Publicar estado
        self.publish_safety_status()
    
    def execute_protocol_actions(self, protocol_name, actions):
        """Ejecutar acciones específicas del protocolo"""
        
        for action in actions:
            self.get_logger().info(f'Executing action: {action} for protocol: {protocol_name}')
            
            if action == 'reduce_speed':
                self.reduce_maximum_speed(0.5)  # 50% de velocidad máxima
            
            elif action == 'stop_forward_movement':
                self.stop_forward_movement()
            
            elif action == 'emergency_stop':
                self.execute_emergency_stop()
            
            elif action == 'limit_speed':
                self.reduce_maximum_speed(0.3)  # 30% de velocidad máxima
            
            elif action == 'emergency_return_to_base':
                self.initiate_return_to_base()
            
            elif action == 'conserve_power':
                self.activate_power_conservation()
            
            elif action == 'diagnostic_mode':
                self.enter_diagnostic_mode()
            
            elif action == 'safe_mode_only':
                self.enter_safe_mode()
            
            elif action == 'autonomous_safe_operation':
                self.enable_autonomous_safe_mode()
            
            elif action == 'return_to_safe_zone':
                self.return_to_safe_zone()
            
            elif action == 'send_mayday':
                self.send_mayday_signal()
            
            # Publicar acción ejecutada
            action_msg = String()
            action_msg.data = f'{protocol_name}:{action}'
            self.protocol_action_pub.publish(action_msg)
    
    def reduce_maximum_speed(self, factor):
        """Reducir velocidad máxima permitida"""
        
        self.get_logger().info(f'Reducing maximum speed by factor: {factor}')
        # Implementar override de cmd_vel
        # (En implementación real, coordinar con controlador de velocidad)
    
    def stop_forward_movement(self):
        """Detener movimiento hacia adelante"""
        
        self.get_logger().warn('Stopping forward movement due to proximity warning')
        # Implementar restricción de movimiento frontal
    
    def execute_emergency_stop(self):
        """Ejecutar parada de emergencia completa"""
        
        self.get_logger().error('Executing emergency stop')
        
        # Publicar comando de parada
        stop_cmd = Twist()
        for _ in range(10):  # Múltiples comandos para asegurar recepción
            self.cmd_vel_override_pub.publish(stop_cmd)
            time.sleep(0.01)
    
    def initiate_return_to_base(self):
        """Iniciar retorno a base"""
        
        self.get_logger().warn('Initiating emergency return to base')
        # Implementar navegación de retorno automático
    
    def activate_power_conservation(self):
        """Activar modo de conservación de energía"""
        
        self.get_logger().info('Activating power conservation mode')
        # Reducir frecuencia de sensores no críticos
        # Limitar potencia de motores
        # Reducir procesamiento de percepción
    
    def enter_diagnostic_mode(self):
        """Entrar en modo diagnóstico"""
        
        self.get_logger().info('Entering diagnostic mode')
        # Activar diagnósticos extendidos
        # Reducir funcionalidad operacional
    
    def enter_safe_mode(self):
        """Entrar en modo seguro"""
        
        self.get_logger().warn('Entering safe mode - limited functionality')
        # Solo funciones básicas de seguridad
        # Movimiento manual únicamente
    
    def send_mayday_signal(self):
        """Enviar señal de auxilio"""
        
        self.get_logger().error('MAYDAY: Robot requires immediate assistance')
        # Implementar notificación a operadores
        # Transmitir posición y estado
    
    def escalate_protocol(self, current_protocol):
        """Escalar a protocolo de mayor severidad"""
        
        if current_protocol not in self.active_protocols:
            return  # Protocolo ya desactivado
        
        protocol = self.safety_protocols[current_protocol]
        escalation = protocol['escalation_protocol']
        
        if escalation:
            self.get_logger().error(f'Escalating from {current_protocol} to {escalation}')
            self.activate_protocol(escalation)
    
    def evaluate_safety_level(self):
        """Evaluación continua del nivel de seguridad"""
        
        # Determinar nivel de seguridad actual basado en protocolos activos
        max_level = SafetyLevel.NORMAL
        
        for protocol_name in self.active_protocols:
            protocol_level = self.safety_protocols[protocol_name]['level']
            if protocol_level.value > max_level.value:
                max_level = protocol_level
        
        # Actualizar nivel si cambió
        if max_level != self.current_safety_level:
            old_level = self.current_safety_level
            self.current_safety_level = max_level
            
            self.get_logger().info(f'Safety level changed: {old_level.name} -> {max_level.name}')
            self.record_safety_event('LEVEL_CHANGE', f'{old_level.name}->{max_level.name}')
        
        # Publicar estado actual
        self.publish_safety_status()
    
    def record_safety_event(self, event_type, details):
        """Registrar evento de seguridad en historial"""
        
        event = {
            'timestamp': time.time(),
            'event_type': event_type,
            'details': details,
            'safety_level': self.current_safety_level.name,
            'active_protocols': list(self.active_protocols)
        }
        
        self.safety_history.append(event)
        
        # Mantener solo últimos 1000 eventos
        if len(self.safety_history) > 1000:
            self.safety_history.pop(0)
        
        self.get_logger().info(f'Safety event recorded: {event_type} - {details}')
    
    def publish_safety_status(self):
        """Publicar estado actual de seguridad"""
        
        status = {
            'safety_level': self.current_safety_level.name,
            'active_protocols': list(self.active_protocols),
            'timestamp': time.time()
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.safety_status_pub.publish(status_msg)
    
    def deactivate_protocol(self, protocol_name):
        """Desactivar protocolo de seguridad"""
        
        if protocol_name in self.active_protocols:
            self.active_protocols.remove(protocol_name)
            self.get_logger().info(f'Deactivated safety protocol: {protocol_name}')
            self.record_safety_event(protocol_name, 'DEACTIVATED')
    
    def execute_emergency_procedures(self):
        """Ejecutar procedimientos de emergencia completos"""
        
        self.get_logger().error('EXECUTING EMERGENCY PROCEDURES')
        
        # 1. Parada inmediata
        self.execute_emergency_stop()
        
        # 2. Activar todas las señales de alarma
        self.send_mayday_signal()
        
        # 3. Guardar estado para análisis post-incidente
        self.save_emergency_state()
        
        # 4. Entrar en modo de espera de rescate
        self.enter_rescue_wait_mode()
    
    def save_emergency_state(self):
        """Guardar estado completo del sistema para análisis"""
        
        emergency_state = {
            'timestamp': time.time(),
            'safety_level': self.current_safety_level.name,
            'active_protocols': list(self.active_protocols),
            'safety_history': self.safety_history[-50:],  # Últimos 50 eventos
            'emergency_trigger': 'EMERGENCY_STOP'
        }
        
        # En implementación real: guardar en archivo persistente
        self.get_logger().error(f'Emergency state saved: {len(self.safety_history)} events recorded')
    
    def enter_rescue_wait_mode(self):
        """Entrar en modo de espera de rescate"""
        
        self.get_logger().error('Entering rescue wait mode - manual intervention required')
        
        # Deshabilitar movimiento autónomo
        # Mantener solo sistemas de comunicación y seguridad
        # Enviar beacon de posición periódicamente

def main(args=None):
    rclpy.init(args=args)
    node = SafetyProtocolManager()
    
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

## Launch Files y Configuración

### Launch File Principal de Seguridad

```xml
<!-- launch/safety_system.launch.py -->
<launch>
  <!-- Argumentos -->
  <arg name="enable_hardware_estop" default="true"/>
  <arg name="enable_health_monitor" default="true"/>
  <arg name="enable_safety_protocols" default="true"/>
  <arg name="debug_mode" default="false"/>
  
  <!-- Emergency Stop Manager -->
  <group if="$(var enable_hardware_estop)">
    <node pkg="tadeo_ecar_safety" exec="emergency_stop_manager">
      <param name="estop_timeout" value="1.0"/>
      <param name="enable_joystick_estop" value="true"/>
      <param name="joystick_estop_button" value="0"/>
      <remap from="/cmd_vel_input" to="/cmd_vel_raw"/>
      <remap from="/cmd_vel" to="/cmd_vel_filtered"/>
    </node>
  </group>
  
  <!-- Health Monitor -->
  <group if="$(var enable_health_monitor)">
    <node pkg="tadeo_ecar_safety" exec="health_monitor">
      <param name="battery_critical_voltage" value="22.0"/>
      <param name="battery_low_voltage" value="23.0"/>
      <param name="motor_critical_temp" value="80.0"/>
      <param name="motor_warning_temp" value="70.0"/>
      <param name="max_cmd_vel_age" value="2.0"/>
      <param name="max_odom_age" value="1.0"/>
    </node>
  </group>
  
  <!-- Safety Protocol Manager -->
  <group if="$(var enable_safety_protocols)">
    <node pkg="tadeo_ecar_safety" exec="safety_protocols">
      <param name="min_obstacle_distance" value="0.5"/>
      <param name="critical_battery_level" value="20.0"/>
      <param name="max_protocol_escalation_time" value="30.0"/>
    </node>
  </group>
  
  <!-- Watchdog Timer -->
  <node pkg="tadeo_ecar_safety" exec="watchdog_timer">
    <param name="timeout_duration" value="5.0"/>
    <param name="critical_nodes" value="['emergency_stop_manager', 'health_monitor']"/>
  </node>
  
  <!-- Safety Dashboard (solo para debug) -->
  <group if="$(var debug_mode)">
    <node pkg="tadeo_ecar_safety" exec="safety_dashboard">
      <param name="update_rate" value="10.0"/>
    </node>
  </group>
  
</launch>
```

### Configuración YAML

```yaml
# config/safety_params.yaml

emergency_stop_manager:
  ros__parameters:
    estop_timeout: 1.0
    enable_joystick_estop: true
    joystick_estop_button: 0
    heartbeat_timeout: 2.0
    
health_monitor:
  ros__parameters:
    # Batería
    battery_critical_voltage: 22.0
    battery_low_voltage: 23.0
    battery_monitor_frequency: 1.0
    
    # Temperatura
    motor_critical_temp: 80.0
    motor_warning_temp: 70.0
    temp_monitor_frequency: 2.0
    
    # Comunicación
    max_cmd_vel_age: 2.0
    max_odom_age: 1.0
    
    # Sistema
    cpu_warning_threshold: 85.0
    cpu_critical_threshold: 95.0
    memory_warning_threshold: 80.0
    memory_critical_threshold: 90.0
    disk_warning_threshold: 85.0
    disk_critical_threshold: 95.0

safety_protocols:
  ros__parameters:
    # Proximidad
    min_obstacle_distance: 0.5
    proximity_check_frequency: 10.0
    
    # Batería
    critical_battery_level: 20.0
    low_battery_level: 30.0
    
    # Escalación
    max_protocol_escalation_time: 30.0
    auto_deescalation_enabled: false
    
    # Comunicación de emergencia
    mayday_repeat_interval: 10.0
    emergency_contact_enabled: true

watchdog_timer:
  ros__parameters:
    timeout_duration: 5.0
    critical_nodes:
      - "emergency_stop_manager"
      - "health_monitor"
      - "safety_protocols"
    check_frequency: 1.0
    auto_restart_enabled: true
```

Este capítulo cubre todos los aspectos fundamentales de seguridad para el eCar 4WD4WS, desde sistemas básicos de parada de emergencia hasta protocolos avanzados de gestión de fallos y monitoreo continuo de salud del sistema.