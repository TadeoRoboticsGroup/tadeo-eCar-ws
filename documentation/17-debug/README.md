# Capítulo 17: Debugging y Diagnóstico para eCar 4WD4WS

## Tabla de Contenidos

1. [Introducción al Debugging](#introducción-al-debugging)
2. [Herramientas de Debugging en ROS2](#herramientas-de-debugging-en-ros2)
3. [Debugging de Hardware](#debugging-de-hardware)
4. [Debugging de Comunicación](#debugging-de-comunicación)
5. [Debugging de Percepción](#debugging-de-percepción)
6. [Debugging de Navegación](#debugging-de-navegación)
7. [Debugging de Control](#debugging-de-control)
8. [Análisis de Logs y Métricas](#análisis-de-logs-y-métricas)
9. [Herramientas de Visualización](#herramientas-de-visualización)
10. [Estrategias de Resolución de Problemas](#estrategias-de-resolución-de-problemas)

## Introducción al Debugging

### ¿Qué es el Debugging en Robótica?

El debugging en robótica es el proceso sistemático de identificar, analizar y resolver problemas en sistemas robóticos complejos como el eCar 4WD4WS.

```
Debugging Robótico = Detección + Diagnóstico + Resolución

Tipos de Problemas Comunes:
✗ Hardware: sensores, actuadores, conectividad
✗ Software: lógica, algoritmos, configuración
✗ Comunicación: latencia, pérdida de mensajes, sincronización
✗ Integración: interacción entre subsistemas
✗ Rendimiento: velocidad, eficiencia, recursos

Metodología de Debugging:
1. Reproducir el problema
2. Aislar el subsistema afectado
3. Analizar datos y logs
4. Formular hipótesis
5. Probar soluciones
6. Validar la corrección
```

### Desafíos Específicos del eCar 4WD4WS

**1. Complejidad del Hardware**
- 4 motores + 4 servos requieren coordinación precisa
- Múltiples sensores con diferentes frecuencias
- Interferencias electromagnéticas entre componentes

**2. Comportamiento Emergente**
- Interacciones no obvias entre subsistemas
- Problemas que solo aparecen bajo condiciones específicas
- Dificultad para reproducir errores intermitentes

**3. Sistemas Distribuidos**
- Múltiples nodos ROS2 ejecutándose en paralelo
- Comunicación asíncrona compleja
- Dificultad para correlacionar eventos temporalmente

### Principios del Debugging Efectivo

**1. Observabilidad**
```cpp
// Instrumentación para observabilidad
class DebugInstrumentedNode : public rclcpp::Node {
public:
    DebugInstrumentedNode(const std::string& name) : Node(name) {
        // Logger configurado para diferentes niveles
        logger_ = this->get_logger();
        
        // Métricas de rendimiento
        performance_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/debug/" + name + "/performance", 10);
        
        // Estados internos
        state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/debug/" + name + "/state", 10);
    }
    
protected:
    void logWithContext(const std::string& message, 
                       const std::string& function_name = "",
                       const std::map<std::string, std::string>& context = {}) {
        
        std::ostringstream log_stream;
        log_stream << "[" << function_name << "] " << message;
        
        for (const auto& [key, value] : context) {
            log_stream << " | " << key << "=" << value;
        }
        
        RCLCPP_INFO(logger_, "%s", log_stream.str().c_str());
    }
};
```

**2. Reproducibilidad**
```cpp
// Grabación de estado para reproducir problemas
class StateRecorder {
public:
    void recordState(const std::string& component, 
                    const std::string& state_data) {
        StateEntry entry;
        entry.timestamp = std::chrono::high_resolution_clock::now();
        entry.component = component;
        entry.data = state_data;
        
        recorded_states_.push_back(entry);
        
        // Mantener solo últimas N entradas
        if (recorded_states_.size() > max_entries_) {
            recorded_states_.erase(recorded_states_.begin());
        }
    }
    
    void saveToFile(const std::string& filename) {
        // Guardar estados para análisis posterior
        std::ofstream file(filename);
        for (const auto& entry : recorded_states_) {
            file << entry.timestamp.time_since_epoch().count() << ","
                 << entry.component << ","
                 << entry.data << std::endl;
        }
    }
};
```

## Herramientas de Debugging en ROS2

### ROS2 CLI Tools para Debugging

```bash
#!/bin/bash
# debug_tools.sh - Herramientas básicas de debugging

echo "=== Herramientas de Debugging ROS2 para eCar ==="

# 1. Información general del sistema
echo "1. Estado general del sistema:"
echo "Nodos activos:"
ros2 node list

echo -e "\nTopics activos:"
ros2 topic list

echo -e "\nServicios disponibles:"
ros2 service list

# 2. Información detallada de nodos
echo -e "\n2. Información detallada de nodos críticos:"
critical_nodes=("motor_controller" "servo_controller" "emergency_stop_manager")

for node in "${critical_nodes[@]}"; do
    echo -e "\n--- Información de $node ---"
    if ros2 node list | grep -q "$node"; then
        echo "✓ Nodo activo"
        echo "Subscripciones:"
        ros2 node info "$node" | grep -A 10 "Subscriptions:"
        echo "Publicaciones:"
        ros2 node info "$node" | grep -A 10 "Publishers:"
    else
        echo "✗ Nodo NO encontrado"
    fi
done

# 3. Análisis de topics
echo -e "\n3. Análisis de topics críticos:"
critical_topics=("/cmd_vel" "/odom" "/scan" "/diagnostics")

for topic in "${critical_topics[@]}"; do
    echo -e "\n--- Análisis de $topic ---"
    if ros2 topic list | grep -q "$topic"; then
        echo "✓ Topic activo"
        echo "Tipo de mensaje:"
        ros2 topic type "$topic"
        echo "Frecuencia (5 segundos de medición):"
        timeout 5s ros2 topic hz "$topic" 2>/dev/null || echo "No hay datos"
        echo "Último mensaje:"
        timeout 2s ros2 topic echo "$topic" --once 2>/dev/null || echo "No hay mensajes"
    else
        echo "✗ Topic NO encontrado"
    fi
done

# 4. Análisis de TF
echo -e "\n4. Análisis de TF Tree:"
echo "Frames disponibles:"
ros2 run tf2_ros tf2_echo --help >/dev/null 2>&1 && {
    timeout 3s ros2 run tf2_tools view_frames 2>/dev/null || echo "Error al generar TF tree"
    
    echo "Test de transformación map -> base_link:"
    timeout 3s ros2 run tf2_ros tf2_echo map base_link 2>/dev/null || echo "Transformación no disponible"
}

# 5. Diagnósticos del sistema
echo -e "\n5. Diagnósticos del sistema:"
if ros2 topic list | grep -q "/diagnostics"; then
    echo "Último reporte de diagnósticos:"
    timeout 3s ros2 topic echo /diagnostics --once 2>/dev/null | head -20
else
    echo "No hay topic de diagnósticos disponible"
fi

echo -e "\n=== Debugging Tools Complete ==="
```

### RQT Tools para Debugging Visual

```bash
#!/bin/bash
# launch_debug_tools.sh

echo "Lanzando herramientas de debugging visual..."

# 1. RQT Graph - Visualizar conexiones entre nodos
echo "Lanzando RQT Graph..."
rqt_graph &

# 2. RQT Console - Visualizar logs
echo "Lanzando RQT Console..."
rqt_console &

# 3. RQT Plot - Gráficos en tiempo real
echo "Lanzando RQT Plot..."
rqt_plot &

# 4. RQT Topic Monitor
echo "Lanzando RQT Topic Monitor..."
rqt_topic &

# 5. RQT Robot Monitor - Para diagnósticos
echo "Lanzando RQT Robot Monitor..."
rqt_robot_monitor &

echo "Todas las herramientas de debugging visual lanzadas."
echo "Usar 'pkill -f rqt' para cerrar todas las herramientas."
```

### Logger Configurado para Debugging

```cpp
// src/debug_logger.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fstream>
#include <mutex>

class DebugLogger : public rclcpp::Node
{
public:
    DebugLogger() : Node("debug_logger")
    {
        // Configurar diferentes niveles de logging
        setupLoggingLevels();
        
        // Suscriptores para capturar información de debugging
        setupSubscribers();
        
        // Archivos de log
        setupLogFiles();
        
        // Timer para flush periódico
        flush_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&DebugLogger::flushLogs, this));
        
        RCLCPP_INFO(this->get_logger(), "Debug Logger initialized");
    }
    
    ~DebugLogger()
    {
        flushLogs();
        closeLogFiles();
    }

private:
    void setupLoggingLevels()
    {
        // Configurar logging a diferentes niveles
        auto ret = rcutils_logging_set_logger_level(
            this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set logging level");
        }
    }
    
    void setupSubscribers()
    {
        // Suscribirse a topics de debugging
        debug_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/debug/system", 10,
            std::bind(&DebugLogger::debugCallback, this, std::placeholders::_1));
        
        diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10,
            std::bind(&DebugLogger::diagnosticsCallback, this, std::placeholders::_1));
        
        performance_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/debug/performance", 10,
            std::bind(&DebugLogger::performanceCallback, this, std::placeholders::_1));
        
        error_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/debug/errors", 10,
            std::bind(&DebugLogger::errorCallback, this, std::placeholders::_1));
    }
    
    void setupLogFiles()
    {
        std::string log_dir = "/tmp/ecar_debug_logs";
        
        // Crear directorio si no existe
        std::filesystem::create_directories(log_dir);
        
        // Timestamp para archivos únicos
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream timestamp;
        timestamp << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        
        // Abrir archivos de log
        debug_log_.open(log_dir + "/debug_" + timestamp.str() + ".log");
        performance_log_.open(log_dir + "/performance_" + timestamp.str() + ".log");
        error_log_.open(log_dir + "/errors_" + timestamp.str() + ".log");
        diagnostics_log_.open(log_dir + "/diagnostics_" + timestamp.str() + ".log");
    }
    
    void debugCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        
        auto timestamp = this->now();
        debug_log_ << "[" << timestamp.seconds() << "] DEBUG: " 
                  << msg->data << std::endl;
        
        RCLCPP_DEBUG(this->get_logger(), "Logged debug message: %s", msg->data.c_str());
    }
    
    void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        
        auto timestamp = this->now();
        diagnostics_log_ << "[" << timestamp.seconds() << "] DIAGNOSTICS:" << std::endl;
        
        for (const auto& status : msg->status) {
            diagnostics_log_ << "  " << status.name << ": " << status.message 
                           << " (Level: " << static_cast<int>(status.level) << ")" << std::endl;
            
            for (const auto& value : status.values) {
                diagnostics_log_ << "    " << value.key << "=" << value.value << std::endl;
            }
        }
        
        diagnostics_log_ << std::endl;
    }
    
    void performanceCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        
        auto timestamp = this->now();
        performance_log_ << "[" << timestamp.seconds() << "] PERFORMANCE: " 
                        << msg->data << std::endl;
    }
    
    void errorCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        
        auto timestamp = this->now();
        error_log_ << "[" << timestamp.seconds() << "] ERROR: " 
                  << msg->data << std::endl;
        
        RCLCPP_ERROR(this->get_logger(), "Logged error: %s", msg->data.c_str());
    }
    
    void flushLogs()
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        
        debug_log_.flush();
        performance_log_.flush();
        error_log_.flush();
        diagnostics_log_.flush();
        
        RCLCPP_DEBUG(this->get_logger(), "Logs flushed to disk");
    }
    
    void closeLogFiles()
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        
        if (debug_log_.is_open()) debug_log_.close();
        if (performance_log_.is_open()) performance_log_.close();
        if (error_log_.is_open()) error_log_.close();
        if (diagnostics_log_.is_open()) diagnostics_log_.close();
    }
    
    // Miembros
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr performance_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr error_sub_;
    
    rclcpp::TimerBase::SharedPtr flush_timer_;
    
    std::ofstream debug_log_;
    std::ofstream performance_log_;
    std::ofstream error_log_;
    std::ofstream diagnostics_log_;
    
    std::mutex log_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DebugLogger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

## Debugging de Hardware

### Hardware Diagnostics Tool

```cpp
// src/hardware_diagnostics.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

class HardwareDiagnostics : public rclcpp::Node
{
public:
    HardwareDiagnostics() : Node("hardware_diagnostics")
    {
        // Suscriptores para datos de hardware
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&HardwareDiagnostics::jointStateCallback, this, std::placeholders::_1));
        
        motor_current_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/motor_currents", 10,
            std::bind(&HardwareDiagnostics::motorCurrentCallback, this, std::placeholders::_1));
        
        servo_feedback_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/servo_feedback", 10,
            std::bind(&HardwareDiagnostics::servoFeedbackCallback, this, std::placeholders::_1));
        
        // Publicadores
        hardware_diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/hardware_diagnostics", 10);
        
        debug_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/debug/hardware", 10);
        
        // Timer para diagnósticos periódicos
        diag_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HardwareDiagnostics::runDiagnostics, this));
        
        // Parámetros
        this->declare_parameter("max_motor_current", 5.0);  // Amperes
        this->declare_parameter("max_servo_error", 5.0);    // Grados
        this->declare_parameter("joint_timeout", 2.0);      // Segundos
        
        // Inicializar estado
        initializeHardwareState();
        
        RCLCPP_INFO(this->get_logger(), "Hardware Diagnostics initialized");
    }

private:
    struct MotorState {
        double position = 0.0;
        double velocity = 0.0;
        double current = 0.0;
        rclcpp::Time last_update;
        bool is_responsive = false;
        int error_count = 0;
    };
    
    struct ServoState {
        double commanded_position = 0.0;
        double actual_position = 0.0;
        double error = 0.0;
        rclcpp::Time last_update;
        bool is_responsive = false;
        int error_count = 0;
    };
    
    void initializeHardwareState()
    {
        // Inicializar estado de motores (4WD)
        motor_names_ = {"front_left", "front_right", "rear_left", "rear_right"};
        for (const auto& name : motor_names_) {
            MotorState state;
            state.last_update = this->now();
            motor_states_[name] = state;
        }
        
        // Inicializar estado de servos (4WS)
        servo_names_ = {"steering_fl", "steering_fr", "steering_rl", "steering_rr"};
        for (const auto& name : servo_names_) {
            ServoState state;
            state.last_update = this->now();
            servo_states_[name] = state;
        }
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto now = this->now();
        
        // Actualizar estado de motores y servos
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& joint_name = msg->name[i];
            
            // Verificar si es un motor
            auto motor_it = motor_states_.find(joint_name);
            if (motor_it != motor_states_.end()) {
                auto& motor = motor_it->second;
                
                if (i < msg->position.size()) motor.position = msg->position[i];
                if (i < msg->velocity.size()) motor.velocity = msg->velocity[i];
                
                motor.last_update = now;
                motor.is_responsive = true;
                motor.error_count = 0;
            }
            
            // Verificar si es un servo
            auto servo_it = servo_states_.find(joint_name);
            if (servo_it != servo_states_.end()) {
                auto& servo = servo_it->second;
                
                if (i < msg->position.size()) servo.actual_position = msg->position[i];
                
                servo.last_update = now;
                servo.is_responsive = true;
                servo.error_count = 0;
            }
        }
    }
    
    void motorCurrentCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Actualizar corrientes de motores
        if (msg->data.size() >= 4) {
            for (size_t i = 0; i < motor_names_.size() && i < msg->data.size(); ++i) {
                motor_states_[motor_names_[i]].current = msg->data[i];
            }
        }
    }
    
    void servoFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Actualizar comandos de servos
        if (msg->data.size() >= 4) {
            for (size_t i = 0; i < servo_names_.size() && i < msg->data.size(); ++i) {
                auto& servo = servo_states_[servo_names_[i]];
                servo.commanded_position = msg->data[i];
                servo.error = servo.commanded_position - servo.actual_position;
            }
        }
    }
    
    void runDiagnostics()
    {
        auto now = this->now();
        double timeout = this->get_parameter("joint_timeout").as_double();
        
        // Diagnosticar motores
        diagnosisMotors(now, timeout);
        
        // Diagnosticar servos
        diagnosisServos(now, timeout);
        
        // Publicar resultados
        publishDiagnostics();
        
        // Debugging detallado
        publishDebugInfo();
    }
    
    void diagnosisMotors(const rclcpp::Time& now, double timeout)
    {
        double max_current = this->get_parameter("max_motor_current").as_double();
        
        for (auto& [name, motor] : motor_states_) {
            // 1. Verificar timeout de comunicación
            double age = (now - motor.last_update).seconds();
            if (age > timeout) {
                motor.is_responsive = false;
                motor.error_count++;
                
                publishDebugMessage("Motor " + name + " communication timeout: " + 
                                  std::to_string(age) + "s");
            }
            
            // 2. Verificar corriente excesiva
            if (motor.current > max_current) {
                motor.error_count++;
                
                publishDebugMessage("Motor " + name + " overcurrent: " + 
                                  std::to_string(motor.current) + "A");
            }
            
            // 3. Verificar velocidad anómala
            if (std::abs(motor.velocity) > 10.0) {  // rad/s
                motor.error_count++;
                
                publishDebugMessage("Motor " + name + " excessive velocity: " + 
                                  std::to_string(motor.velocity) + " rad/s");
            }
            
            // 4. Verificar si el motor está bloqueado
            if (motor.current > max_current * 0.8 && std::abs(motor.velocity) < 0.1) {
                motor.error_count++;
                
                publishDebugMessage("Motor " + name + " appears to be blocked");
            }
        }
    }
    
    void diagnosisServos(const rclcpp::Time& now, double timeout)
    {
        double max_error = this->get_parameter("max_servo_error").as_double();
        
        for (auto& [name, servo] : servo_states_) {
            // 1. Verificar timeout de comunicación
            double age = (now - servo.last_update).seconds();
            if (age > timeout) {
                servo.is_responsive = false;
                servo.error_count++;
                
                publishDebugMessage("Servo " + name + " communication timeout: " + 
                                  std::to_string(age) + "s");
            }
            
            // 2. Verificar error de posición
            if (std::abs(servo.error) > max_error * M_PI / 180.0) {  // Convertir a radianes
                servo.error_count++;
                
                publishDebugMessage("Servo " + name + " position error: " + 
                                  std::to_string(servo.error * 180.0 / M_PI) + " degrees");
            }
            
            // 3. Verificar límites físicos
            double max_angle = M_PI / 3;  // ±60 grados
            if (std::abs(servo.actual_position) > max_angle) {
                servo.error_count++;
                
                publishDebugMessage("Servo " + name + " exceeds physical limits: " + 
                                  std::to_string(servo.actual_position * 180.0 / M_PI) + " degrees");
            }
        }
    }
    
    void publishDiagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = this->now();
        
        // Diagnóstico de motores
        for (const auto& [name, motor] : motor_states_) {
            diagnostic_msgs::msg::DiagnosticStatus motor_diag;
            motor_diag.name = "Motor " + name;
            motor_diag.hardware_id = "motor_" + name;
            
            if (!motor.is_responsive) {
                motor_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                motor_diag.message = "Motor not responsive";
            } else if (motor.error_count > 5) {
                motor_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                motor_diag.message = "Motor has recurring errors";
            } else {
                motor_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                motor_diag.message = "Motor operating normally";
            }
            
            // Agregar valores
            diagnostic_msgs::msg::KeyValue position_kv;
            position_kv.key = "position";
            position_kv.value = std::to_string(motor.position);
            motor_diag.values.push_back(position_kv);
            
            diagnostic_msgs::msg::KeyValue velocity_kv;
            velocity_kv.key = "velocity";
            velocity_kv.value = std::to_string(motor.velocity);
            motor_diag.values.push_back(velocity_kv);
            
            diagnostic_msgs::msg::KeyValue current_kv;
            current_kv.key = "current";
            current_kv.value = std::to_string(motor.current);
            motor_diag.values.push_back(current_kv);
            
            diag_array.status.push_back(motor_diag);
        }
        
        // Diagnóstico de servos
        for (const auto& [name, servo] : servo_states_) {
            diagnostic_msgs::msg::DiagnosticStatus servo_diag;
            servo_diag.name = "Servo " + name;
            servo_diag.hardware_id = "servo_" + name;
            
            if (!servo.is_responsive) {
                servo_diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                servo_diag.message = "Servo not responsive";
            } else if (servo.error_count > 3) {
                servo_diag.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                servo_diag.message = "Servo has positioning errors";
            } else {
                servo_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                servo_diag.message = "Servo operating normally";
            }
            
            // Agregar valores
            diagnostic_msgs::msg::KeyValue commanded_kv;
            commanded_kv.key = "commanded_position";
            commanded_kv.value = std::to_string(servo.commanded_position * 180.0 / M_PI);
            servo_diag.values.push_back(commanded_kv);
            
            diagnostic_msgs::msg::KeyValue actual_kv;
            actual_kv.key = "actual_position";
            actual_kv.value = std::to_string(servo.actual_position * 180.0 / M_PI);
            servo_diag.values.push_back(actual_kv);
            
            diagnostic_msgs::msg::KeyValue error_kv;
            error_kv.key = "position_error";
            error_kv.value = std::to_string(servo.error * 180.0 / M_PI);
            servo_diag.values.push_back(error_kv);
            
            diag_array.status.push_back(servo_diag);
        }
        
        hardware_diag_pub_->publish(diag_array);
    }
    
    void publishDebugInfo()
    {
        std::ostringstream debug_info;
        debug_info << "=== Hardware Status ===" << std::endl;
        
        // Estado de motores
        debug_info << "Motors:" << std::endl;
        for (const auto& [name, motor] : motor_states_) {
            debug_info << "  " << name << ": ";
            debug_info << "pos=" << std::fixed << std::setprecision(2) << motor.position;
            debug_info << ", vel=" << motor.velocity;
            debug_info << ", curr=" << motor.current << "A";
            debug_info << ", errors=" << motor.error_count;
            debug_info << std::endl;
        }
        
        // Estado de servos
        debug_info << "Servos:" << std::endl;
        for (const auto& [name, servo] : servo_states_) {
            debug_info << "  " << name << ": ";
            debug_info << "cmd=" << std::fixed << std::setprecision(1) 
                      << servo.commanded_position * 180.0 / M_PI << "°";
            debug_info << ", act=" << servo.actual_position * 180.0 / M_PI << "°";
            debug_info << ", err=" << servo.error * 180.0 / M_PI << "°";
            debug_info << ", errors=" << servo.error_count;
            debug_info << std::endl;
        }
        
        publishDebugMessage(debug_info.str());
    }
    
    void publishDebugMessage(const std::string& message)
    {
        std_msgs::msg::String debug_msg;
        debug_msg.data = message;
        debug_pub_->publish(debug_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
    }
    
    // Miembros
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_current_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr servo_feedback_sub_;
    
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr hardware_diag_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
    
    rclcpp::TimerBase::SharedPtr diag_timer_;
    
    std::vector<std::string> motor_names_;
    std::vector<std::string> servo_names_;
    std::map<std::string, MotorState> motor_states_;
    std::map<std::string, ServoState> servo_states_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareDiagnostics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Script de Debugging de Hardware

```bash
#!/bin/bash
# debug_hardware.sh

echo "=== Hardware Debugging para eCar 4WD4WS ==="

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Función para verificar hardware
check_hardware_component() {
    local component=$1
    local expected_value=$2
    local actual_value=$3
    local tolerance=$4
    
    if [ -z "$actual_value" ] || [ "$actual_value" = "null" ]; then
        echo -e "${RED}✗${NC} $component: No hay datos"
        return 1
    fi
    
    # Verificar si está dentro de tolerancia (para valores numéricos)
    if [[ $actual_value =~ ^[0-9.-]+$ ]] && [[ $expected_value =~ ^[0-9.-]+$ ]]; then
        local diff=$(echo "$actual_value - $expected_value" | bc -l 2>/dev/null)
        local abs_diff=$(echo "${diff#-}" | bc -l 2>/dev/null)
        
        if (( $(echo "$abs_diff <= $tolerance" | bc -l) )); then
            echo -e "${GREEN}✓${NC} $component: $actual_value (dentro de tolerancia ±$tolerance)"
            return 0
        else
            echo -e "${YELLOW}⚠${NC} $component: $actual_value (esperado: $expected_value ±$tolerance)"
            return 1
        fi
    else
        echo -e "${GREEN}✓${NC} $component: $actual_value"
        return 0
    fi
}

echo "1. Verificando conectividad de hardware..."

# Verificar que los nodos de hardware estén corriendo
hardware_nodes=("motor_controller" "servo_controller" "sensor_manager")
for node in "${hardware_nodes[@]}"; do
    if ros2 node list | grep -q "$node"; then
        echo -e "${GREEN}✓${NC} $node está corriendo"
    else
        echo -e "${RED}✗${NC} $node NO está corriendo"
    fi
done

echo -e "\n2. Verificando estado de motores..."

# Obtener estado actual de joints
if ros2 topic list | grep -q "/joint_states"; then
    echo "Obteniendo estado de motores..."
    
    # Capturar último mensaje de joint_states
    joint_data=$(timeout 3s ros2 topic echo /joint_states --once 2>/dev/null)
    
    if [ -n "$joint_data" ]; then
        echo "Datos de joints recibidos - analizando..."
        
        # Extraer nombres de joints
        joint_names=$(echo "$joint_data" | grep -A 20 "name:" | grep -o "'[^']*'" | tr -d "'")
        
        echo "Joints detectados:"
        echo "$joint_names"
        
        # Verificar motores específicos
        motors=("front_left_wheel" "front_right_wheel" "rear_left_wheel" "rear_right_wheel")
        for motor in "${motors[@]}"; do
            if echo "$joint_names" | grep -q "$motor"; then
                echo -e "${GREEN}✓${NC} Motor $motor detectado"
            else
                echo -e "${RED}✗${NC} Motor $motor NO detectado"
            fi
        done
    else
        echo -e "${RED}✗${NC} No se pudieron obtener datos de joint_states"
    fi
else
    echo -e "${RED}✗${NC} Topic /joint_states no disponible"
fi

echo -e "\n3. Verificando servos de dirección..."

if ros2 topic list | grep -q "/servo_feedback"; then
    echo "Obteniendo estado de servos..."
    
    servo_data=$(timeout 3s ros2 topic echo /servo_feedback --once 2>/dev/null)
    
    if [ -n "$servo_data" ]; then
        echo "Datos de servos recibidos"
        
        # Verificar que hay 4 servos
        servo_count=$(echo "$servo_data" | grep -o "data:" | wc -l)
        if [ "$servo_count" -eq 4 ]; then
            echo -e "${GREEN}✓${NC} Los 4 servos están reportando"
        else
            echo -e "${YELLOW}⚠${NC} Solo $servo_count servos reportando (esperado: 4)"
        fi
    else
        echo -e "${RED}✗${NC} No se pudieron obtener datos de servos"
    fi
else
    echo -e "${RED}✗${NC} Topic /servo_feedback no disponible"
fi

echo -e "\n4. Verificando sensores..."

# LiDAR
echo "Verificando LiDAR..."
if ros2 topic list | grep -q "/scan"; then
    scan_data=$(timeout 3s ros2 topic echo /scan --once 2>/dev/null)
    if [ -n "$scan_data" ]; then
        # Extraer información del scan
        range_min=$(echo "$scan_data" | grep "range_min:" | grep -o '[0-9.]*')
        range_max=$(echo "$scan_data" | grep "range_max:" | grep -o '[0-9.]*')
        ranges_count=$(echo "$scan_data" | grep -A 1000 "ranges:" | grep -o '[0-9.]*' | wc -l)
        
        echo -e "${GREEN}✓${NC} LiDAR funcionando"
        echo "  - Rango: $range_min m a $range_max m"
        echo "  - Puntos por scan: $ranges_count"
    else
        echo -e "${RED}✗${NC} LiDAR no responde"
    fi
else
    echo -e "${RED}✗${NC} Topic /scan no disponible"
fi

# IMU
echo -e "\nVerificando IMU..."
if ros2 topic list | grep -q "/imu"; then
    imu_data=$(timeout 3s ros2 topic echo /imu --once 2>/dev/null)
    if [ -n "$imu_data" ]; then
        echo -e "${GREEN}✓${NC} IMU funcionando"
        
        # Verificar aceleración cerca de la gravedad
        accel_z=$(echo "$imu_data" | grep -A 3 "linear_acceleration:" | grep "z:" | grep -o '[0-9.-]*')
        if [ -n "$accel_z" ]; then
            check_hardware_component "IMU aceleración Z" "9.81" "$accel_z" "2.0"
        fi
    else
        echo -e "${RED}✗${NC} IMU no responde"
    fi
else
    echo -e "${RED}✗${NC} Topic /imu no disponible"
fi

# Cámara
echo -e "\nVerificando cámara..."
if ros2 topic list | grep -q "/camera/image_raw"; then
    # Verificar que hay datos de imagen
    if timeout 3s ros2 topic hz /camera/image_raw >/dev/null 2>&1; then
        echo -e "${GREEN}✓${NC} Cámara publicando imágenes"
    else
        echo -e "${RED}✗${NC} Cámara no está publicando"
    fi
else
    echo -e "${RED}✗${NC} Topic /camera/image_raw no disponible"
fi

echo -e "\n5. Verificando diagnósticos de hardware..."

if ros2 topic list | grep -q "/hardware_diagnostics"; then
    diag_data=$(timeout 5s ros2 topic echo /hardware_diagnostics --once 2>/dev/null)
    
    if [ -n "$diag_data" ]; then
        echo "Análisis de diagnósticos de hardware:"
        
        # Contar errores y warnings
        error_count=$(echo "$diag_data" | grep -c "level: 3")
        warn_count=$(echo "$diag_data" | grep -c "level: 1")
        ok_count=$(echo "$diag_data" | grep -c "level: 0")
        
        echo "  - Estados OK: $ok_count"
        echo "  - Warnings: $warn_count"
        echo "  - Errores: $error_count"
        
        if [ "$error_count" -eq 0 ] && [ "$warn_count" -eq 0 ]; then
            echo -e "${GREEN}✓${NC} Todos los componentes de hardware están OK"
        elif [ "$error_count" -eq 0 ]; then
            echo -e "${YELLOW}⚠${NC} Hardware operacional con algunas advertencias"
        else
            echo -e "${RED}✗${NC} Hardware tiene errores críticos"
        fi
    else
        echo -e "${RED}✗${NC} No se pudieron obtener diagnósticos de hardware"
    fi
else
    echo -e "${RED}✗${NC} Topic /hardware_diagnostics no disponible"
fi

echo -e "\n6. Test de movimiento básico..."

echo "Enviando comando de test de motores..."
# Comando muy suave para test
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once >/dev/null 2>&1

sleep 2

echo "Enviando comando de parada..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once >/dev/null 2>&1

echo "Test de movimiento completado"

echo -e "\n7. Test de servos de dirección..."

echo "Enviando comando de test de dirección..."
# Pequeño movimiento de dirección
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once >/dev/null 2>&1

sleep 2

echo "Retornando a posición central..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once >/dev/null 2>&1

echo "Test de dirección completado"

echo -e "\n=== Hardware Debugging Complete ==="
echo "Revisar logs en /tmp/ecar_debug_logs/ para análisis detallado"
```

### Debugging de Percepción

```python
#!/usr/bin/env python3
# debug_perception.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import time
import json

class PerceptionDebugger(Node):
    def __init__(self):
        super().__init__('perception_debugger')
        
        self.bridge = CvBridge()
        
        # Suscriptores para datos de percepción
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.obstacles_sub = self.create_subscription(
            PointCloud2, '/obstacles', self.obstacles_callback, 10)
        
        self.detection_sub = self.create_subscription(
            PointStamped, '/camera/detections', self.detection_callback, 10)
        
        # Estado para debugging
        self.scan_stats = {
            'last_update': None,
            'frequency': 0.0,
            'range_stats': {},
            'error_count': 0
        }
        
        self.image_stats = {
            'last_update': None,
            'frequency': 0.0,
            'resolution': (0, 0),
            'brightness': 0.0,
            'error_count': 0
        }
        
        self.detection_stats = {
            'detections_per_minute': 0,
            'detection_history': [],
            'last_detection': None
        }
        
        # Timers para análisis
        self.analysis_timer = self.create_timer(5.0, self.analyze_perception_health)
        self.stats_timer = self.create_timer(1.0, self.update_frequency_stats)
        
        # Contadores para frecuencia
        self.scan_count = 0
        self.image_count = 0
        self.last_freq_check = time.time()
        
        self.get_logger().info('Perception Debugger initialized')
    
    def scan_callback(self, msg):
        """Analizar datos del LiDAR"""
        try:
            self.scan_count += 1
            self.scan_stats['last_update'] = time.time()
            
            # Analizar estadísticas del scan
            valid_ranges = [r for r in msg.ranges 
                          if msg.range_min <= r <= msg.range_max]
            
            if valid_ranges:
                self.scan_stats['range_stats'] = {
                    'min_range': min(valid_ranges),
                    'max_range': max(valid_ranges),
                    'mean_range': np.mean(valid_ranges),
                    'valid_points': len(valid_ranges),
                    'total_points': len(msg.ranges),
                    'validity_ratio': len(valid_ranges) / len(msg.ranges)
                }
                
                # Detectar problemas comunes
                self.detect_lidar_issues(msg, valid_ranges)
            
        except Exception as e:
            self.scan_stats['error_count'] += 1
            self.get_logger().error(f'Error processing scan: {str(e)}')
    
    def image_callback(self, msg):
        """Analizar datos de la cámara"""
        try:
            self.image_count += 1
            self.image_stats['last_update'] = time.time()
            
            # Convertir imagen para análisis
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Estadísticas básicas de la imagen
            self.image_stats['resolution'] = (cv_image.shape[1], cv_image.shape[0])
            
            # Analizar brillo promedio
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            self.image_stats['brightness'] = np.mean(gray)
            
            # Detectar problemas comunes
            self.detect_camera_issues(cv_image)
            
        except Exception as e:
            self.image_stats['error_count'] += 1
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def obstacles_callback(self, msg):
        """Analizar detección de obstáculos"""
        # Log básico de obstáculos detectados
        self.get_logger().debug(f'Obstacles detected: {msg.width} points')
    
    def detection_callback(self, msg):
        """Analizar detecciones de objetos"""
        detection_time = time.time()
        
        # Registrar detección
        self.detection_stats['detection_history'].append({
            'timestamp': detection_time,
            'position': [msg.point.x, msg.point.y, msg.point.z]
        })
        
        self.detection_stats['last_detection'] = detection_time
        
        # Mantener solo último minuto
        cutoff_time = detection_time - 60.0
        self.detection_stats['detection_history'] = [
            d for d in self.detection_stats['detection_history']
            if d['timestamp'] > cutoff_time
        ]
        
        # Calcular frecuencia de detecciones
        self.detection_stats['detections_per_minute'] = len(
            self.detection_stats['detection_history'])
    
    def detect_lidar_issues(self, msg, valid_ranges):
        """Detectar problemas específicos del LiDAR"""
        
        # 1. Muy pocos puntos válidos
        validity_ratio = len(valid_ranges) / len(msg.ranges)
        if validity_ratio < 0.5:
            self.get_logger().warn(
                f'Low LiDAR validity ratio: {validity_ratio:.2f}')
        
        # 2. Rangos sospechosamente uniformes (posible obstrucción)
        if valid_ranges:
            range_std = np.std(valid_ranges)
            if range_std < 0.1:
                self.get_logger().warn(
                    f'LiDAR readings very uniform, possible obstruction')
        
        # 3. Demasiados puntos en rango mínimo
        min_range_count = sum(1 for r in msg.ranges if abs(r - msg.range_min) < 0.1)
        if min_range_count > len(msg.ranges) * 0.3:
            self.get_logger().warn(
                f'Too many minimum range readings: {min_range_count}')
        
        # 4. Verificar simetría aproximada (robot en espacio abierto)
        mid_index = len(msg.ranges) // 2
        quarter_index = len(msg.ranges) // 4
        
        left_ranges = msg.ranges[:quarter_index]
        right_ranges = msg.ranges[-quarter_index:]
        
        left_valid = [r for r in left_ranges if msg.range_min <= r <= msg.range_max]
        right_valid = [r for r in right_ranges if msg.range_min <= r <= msg.range_max]
        
        if left_valid and right_valid:
            left_mean = np.mean(left_valid)
            right_mean = np.mean(right_valid)
            asymmetry = abs(left_mean - right_mean) / max(left_mean, right_mean)
            
            if asymmetry > 0.5:
                self.get_logger().debug(
                    f'LiDAR asymmetry detected: {asymmetry:.2f}')
    
    def detect_camera_issues(self, cv_image):
        """Detectar problemas específicos de la cámara"""
        
        h, w = cv_image.shape[:2]
        
        # 1. Imagen muy oscura o muy clara
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        
        if brightness < 30:
            self.get_logger().warn(f'Image very dark: brightness={brightness:.1f}')
        elif brightness > 220:
            self.get_logger().warn(f'Image very bright: brightness={brightness:.1f}')
        
        # 2. Imagen borrosa (bajo contraste)
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        if laplacian_var < 100:
            self.get_logger().warn(f'Image appears blurry: variance={laplacian_var:.1f}')
        
        # 3. Verificar si hay movimiento excesivo (motion blur)
        edges = cv2.Canny(gray, 50, 150)
        edge_ratio = np.sum(edges > 0) / (h * w)
        
        if edge_ratio < 0.01:
            self.get_logger().warn(f'Very few edges detected, possible motion blur')
        
        # 4. Verificar saturación de color
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        saturation = hsv[:,:,1]
        over_saturated = np.sum(saturation > 240) / (h * w)
        
        if over_saturated > 0.1:
            self.get_logger().warn(f'High color saturation: {over_saturated:.2f}')
    
    def update_frequency_stats(self):
        """Actualizar estadísticas de frecuencia"""
        current_time = time.time()
        time_diff = current_time - self.last_freq_check
        
        if time_diff >= 1.0:  # Actualizar cada segundo
            # Calcular frecuencias
            self.scan_stats['frequency'] = self.scan_count / time_diff
            self.image_stats['frequency'] = self.image_count / time_diff
            
            # Reset contadores
            self.scan_count = 0
            self.image_count = 0
            self.last_freq_check = current_time
    
    def analyze_perception_health(self):
        """Análisis periódico de salud de percepción"""
        current_time = time.time()
        
        self.get_logger().info("=== Perception Health Analysis ===")
        
        # Análisis de LiDAR
        if self.scan_stats['last_update']:
            scan_age = current_time - self.scan_stats['last_update']
            
            self.get_logger().info(f"LiDAR Status:")
            self.get_logger().info(f"  Frequency: {self.scan_stats['frequency']:.1f} Hz")
            self.get_logger().info(f"  Last update: {scan_age:.1f}s ago")
            self.get_logger().info(f"  Errors: {self.scan_stats['error_count']}")
            
            if 'range_stats' in self.scan_stats and self.scan_stats['range_stats']:
                stats = self.scan_stats['range_stats']
                self.get_logger().info(f"  Valid points: {stats['valid_points']}/{stats['total_points']}")
                self.get_logger().info(f"  Range: {stats['min_range']:.2f} - {stats['max_range']:.2f}m")
                self.get_logger().info(f"  Mean range: {stats['mean_range']:.2f}m")
            
            # Alertas
            if scan_age > 5.0:
                self.get_logger().error("LiDAR data is stale!")
            if self.scan_stats['frequency'] < 5.0:
                self.get_logger().warn("LiDAR frequency is low!")
        else:
            self.get_logger().error("No LiDAR data received!")
        
        # Análisis de cámara
        if self.image_stats['last_update']:
            image_age = current_time - self.image_stats['last_update']
            
            self.get_logger().info(f"Camera Status:")
            self.get_logger().info(f"  Frequency: {self.image_stats['frequency']:.1f} Hz")
            self.get_logger().info(f"  Last update: {image_age:.1f}s ago")
            self.get_logger().info(f"  Resolution: {self.image_stats['resolution']}")
            self.get_logger().info(f"  Brightness: {self.image_stats['brightness']:.1f}")
            self.get_logger().info(f"  Errors: {self.image_stats['error_count']}")
            
            # Alertas
            if image_age > 2.0:
                self.get_logger().error("Camera data is stale!")
            if self.image_stats['frequency'] < 10.0:
                self.get_logger().warn("Camera frequency is low!")
        else:
            self.get_logger().error("No camera data received!")
        
        # Análisis de detecciones
        detections_recent = len([d for d in self.detection_stats['detection_history']
                               if current_time - d['timestamp'] < 10.0])
        
        self.get_logger().info(f"Detection Status:")
        self.get_logger().info(f"  Detections per minute: {self.detection_stats['detections_per_minute']}")
        self.get_logger().info(f"  Recent detections (10s): {detections_recent}")
        
        if self.detection_stats['last_detection']:
            last_detection_age = current_time - self.detection_stats['last_detection']
            self.get_logger().info(f"  Last detection: {last_detection_age:.1f}s ago")
        
        self.get_logger().info("=== End Analysis ===")
    
    def save_debug_data(self, filename="/tmp/perception_debug.json"):
        """Guardar datos de debugging para análisis offline"""
        debug_data = {
            'timestamp': time.time(),
            'scan_stats': self.scan_stats,
            'image_stats': self.image_stats,
            'detection_stats': self.detection_stats
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(debug_data, f, indent=2, default=str)
            self.get_logger().info(f"Debug data saved to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save debug data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionDebugger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_debug_data()  # Guardar datos al salir
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Este capítulo proporciona un conjunto completo de herramientas y estrategias para debugging y diagnóstico del sistema eCar 4WD4WS, cubriendo desde hardware hasta percepción con herramientas específicas para cada subsistema.