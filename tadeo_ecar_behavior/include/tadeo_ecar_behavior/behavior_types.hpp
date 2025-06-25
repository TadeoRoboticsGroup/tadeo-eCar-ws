#ifndef TADEO_ECAR_BEHAVIOR_TYPES_HPP_
#define TADEO_ECAR_BEHAVIOR_TYPES_HPP_

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

namespace tadeo_ecar_behavior
{

// Estados del robot para comportamientos
enum class RobotState
{
    IDLE,                    // Robot en reposo
    INITIALIZING,           // Inicializando sistemas
    NAVIGATING,             // Navegando hacia objetivo
    EXPLORING,              // Explorando entorno
    PATROLLING,             // Patrullando área
    CHARGING,               // Cargando batería
    EMERGENCY,              // Estado de emergencia
    MAINTENANCE,            // Modo de mantenimiento
    MISSION_EXECUTING,      // Ejecutando misión específica
    WAITING,                // Esperando comandos
    RECOVERING,             // Recuperándose de fallo
    DOCKING,                // Acoplándose a estación
    UNKNOWN                 // Estado desconocido
};

// Tipos de comportamientos disponibles
enum class BehaviorType
{
    NAVIGATION,             // Comportamientos de navegación
    EXPLORATION,            // Comportamientos de exploración
    PATROL,                 // Comportamientos de patrullaje
    EMERGENCY,              // Comportamientos de emergencia
    MAINTENANCE,            // Comportamientos de mantenimiento
    MISSION,                // Comportamientos de misión
    INTERACTION,            // Comportamientos de interacción
    LEARNING,               // Comportamientos de aprendizaje
    RECOVERY,               // Comportamientos de recuperación
    IDLE                    // Comportamientos de reposo
};

// Prioridades de comportamientos
enum class BehaviorPriority
{
    EMERGENCY = 0,          // Máxima prioridad - emergencias
    SAFETY = 1,             // Alta prioridad - seguridad
    MISSION_CRITICAL = 2,   // Crítico para misión
    HIGH = 3,               // Alta prioridad
    NORMAL = 4,             // Prioridad normal
    LOW = 5,                // Baja prioridad
    BACKGROUND = 6          // Prioridad de fondo
};

// Estados de ejecución de comportamientos
enum class BehaviorStatus
{
    IDLE,                   // Comportamiento inactivo
    RUNNING,                // Ejecutándose actualmente
    SUCCESS,                // Completado exitosamente
    FAILURE,                // Falló en ejecución
    SUSPENDED,              // Suspendido temporalmente
    CANCELLED,              // Cancelado por usuario/sistema
    PREEMPTED,              // Interrumpido por comportamiento de mayor prioridad
    UNKNOWN                 // Estado desconocido
};

// Condiciones del entorno
struct EnvironmentCondition
{
    std::string name;               // Nombre de la condición
    bool is_active;                 // Si la condición está activa
    double confidence;              // Confianza en la condición (0.0-1.0)
    rclcpp::Time last_updated;     // Última actualización
    std::map<std::string, double> parameters; // Parámetros adicionales
    
    EnvironmentCondition() : is_active(false), confidence(0.0) {}
    EnvironmentCondition(const std::string& name_, bool active = false, double conf = 0.0) 
        : name(name_), is_active(active), confidence(conf) {}
};

// Contexto del robot
struct RobotContext
{
    RobotState current_state;                          // Estado actual
    geometry_msgs::msg::PoseStamped current_pose;     // Pose actual
    geometry_msgs::msg::Twist current_velocity;       // Velocidad actual
    double battery_level;                              // Nivel de batería (0.0-1.0)
    bool emergency_active;                             // Si hay emergencia activa
    bool sensors_healthy;                              // Si sensores están funcionando
    bool navigation_ready;                             // Si navegación está lista
    std::map<std::string, EnvironmentCondition> conditions; // Condiciones del entorno
    rclcpp::Time last_update;                         // Última actualización del contexto
    
    RobotContext() : 
        current_state(RobotState::IDLE), 
        battery_level(1.0), 
        emergency_active(false),
        sensors_healthy(true),
        navigation_ready(false) {}
};

// Parámetros de comportamiento
struct BehaviorParameters
{
    std::string name;                           // Nombre del comportamiento
    BehaviorType type;                          // Tipo de comportamiento
    BehaviorPriority priority;                  // Prioridad de ejecución
    double timeout_seconds;                     // Timeout en segundos
    int max_retries;                           // Máximo número de reintentos
    bool can_be_preempted;                     // Si puede ser interrumpido
    std::map<std::string, std::string> config; // Configuración adicional
    
    BehaviorParameters() : 
        type(BehaviorType::IDLE), 
        priority(BehaviorPriority::NORMAL),
        timeout_seconds(30.0),
        max_retries(3),
        can_be_preempted(true) {}
};

// Resultado de ejecución de comportamiento
struct BehaviorResult
{
    BehaviorStatus status;              // Estado final del comportamiento
    std::string message;                // Mensaje descriptivo del resultado
    double execution_time;              // Tiempo de ejecución en segundos
    bool was_preempted;                // Si fue interrumpido
    std::string failure_reason;        // Razón del fallo (si aplica)
    std::map<std::string, double> metrics; // Métricas de rendimiento
    
    BehaviorResult() : 
        status(BehaviorStatus::IDLE), 
        execution_time(0.0),
        was_preempted(false) {}
};

// Información de comportamiento activo
struct ActiveBehavior
{
    std::string behavior_id;            // ID único del comportamiento
    BehaviorParameters parameters;      // Parámetros del comportamiento
    BehaviorStatus current_status;      // Estado actual
    rclcpp::Time start_time;           // Tiempo de inicio
    rclcpp::Time last_update;          // Última actualización
    int retry_count;                   // Número de reintentos realizados
    
    ActiveBehavior() : current_status(BehaviorStatus::IDLE), retry_count(0) {}
};

// Métricas del sistema de comportamientos
struct BehaviorMetrics
{
    int total_behaviors_executed;       // Total de comportamientos ejecutados
    int successful_behaviors;           // Comportamientos exitosos
    int failed_behaviors;               // Comportamientos fallidos
    int preempted_behaviors;           // Comportamientos interrumpidos
    double average_execution_time;      // Tiempo promedio de ejecución
    double success_rate;               // Tasa de éxito (0.0-1.0)
    rclcpp::Time session_start_time;   // Inicio de la sesión
    
    BehaviorMetrics() : 
        total_behaviors_executed(0),
        successful_behaviors(0),
        failed_behaviors(0),
        preempted_behaviors(0),
        average_execution_time(0.0),
        success_rate(1.0) {}
    
    void updateMetrics() {
        if (total_behaviors_executed > 0) {
            success_rate = static_cast<double>(successful_behaviors) / total_behaviors_executed;
        }
    }
};

// Configuración del árbol de comportamientos
struct BehaviorTreeConfig
{
    std::string tree_file_path;         // Ruta al archivo XML del árbol
    std::string tree_name;              // Nombre del árbol de comportamientos
    double tick_frequency;              // Frecuencia de tick del árbol (Hz)
    bool auto_reload;                   // Recargar automáticamente cambios
    std::map<std::string, std::string> blackboard_values; // Valores del blackboard
    
    BehaviorTreeConfig() : 
        tick_frequency(10.0),
        auto_reload(false) {}
};

// Comando de comportamiento
struct BehaviorCommand
{
    std::string command_type;           // Tipo de comando (start, stop, pause, resume)
    std::string behavior_name;          // Nombre del comportamiento objetivo
    std::string tree_name;              // Nombre del árbol (opcional)
    std::map<std::string, std::string> parameters; // Parámetros del comando
    BehaviorPriority priority;          // Prioridad del comando
    rclcpp::Time timestamp;            // Timestamp del comando
    
    BehaviorCommand() : priority(BehaviorPriority::NORMAL) {}
};

// Evento del sistema
struct SystemEvent
{
    std::string event_type;             // Tipo de evento
    std::string source;                 // Fuente del evento
    std::string description;            // Descripción del evento
    BehaviorPriority severity;          // Severidad del evento
    rclcpp::Time timestamp;            // Timestamp del evento
    std::map<std::string, std::string> data; // Datos adicionales del evento
    
    SystemEvent() : severity(BehaviorPriority::NORMAL) {}
};

// Funciones de utilidad
inline std::string robotStateToString(RobotState state) {
    switch (state) {
        case RobotState::IDLE: return "IDLE";
        case RobotState::INITIALIZING: return "INITIALIZING";
        case RobotState::NAVIGATING: return "NAVIGATING";
        case RobotState::EXPLORING: return "EXPLORING";
        case RobotState::PATROLLING: return "PATROLLING";
        case RobotState::CHARGING: return "CHARGING";
        case RobotState::EMERGENCY: return "EMERGENCY";
        case RobotState::MAINTENANCE: return "MAINTENANCE";
        case RobotState::MISSION_EXECUTING: return "MISSION_EXECUTING";
        case RobotState::WAITING: return "WAITING";
        case RobotState::RECOVERING: return "RECOVERING";
        case RobotState::DOCKING: return "DOCKING";
        default: return "UNKNOWN";
    }
}

inline std::string behaviorTypeToString(BehaviorType type) {
    switch (type) {
        case BehaviorType::NAVIGATION: return "NAVIGATION";
        case BehaviorType::EXPLORATION: return "EXPLORATION";
        case BehaviorType::PATROL: return "PATROL";
        case BehaviorType::EMERGENCY: return "EMERGENCY";
        case BehaviorType::MAINTENANCE: return "MAINTENANCE";
        case BehaviorType::MISSION: return "MISSION";
        case BehaviorType::INTERACTION: return "INTERACTION";
        case BehaviorType::LEARNING: return "LEARNING";
        case BehaviorType::RECOVERY: return "RECOVERY";
        default: return "IDLE";
    }
}

inline std::string behaviorStatusToString(BehaviorStatus status) {
    switch (status) {
        case BehaviorStatus::IDLE: return "IDLE";
        case BehaviorStatus::RUNNING: return "RUNNING";
        case BehaviorStatus::SUCCESS: return "SUCCESS";
        case BehaviorStatus::FAILURE: return "FAILURE";
        case BehaviorStatus::SUSPENDED: return "SUSPENDED";
        case BehaviorStatus::CANCELLED: return "CANCELLED";
        case BehaviorStatus::PREEMPTED: return "PREEMPTED";
        default: return "UNKNOWN";
    }
}

inline std::string behaviorPriorityToString(BehaviorPriority priority) {
    switch (priority) {
        case BehaviorPriority::EMERGENCY: return "EMERGENCY";
        case BehaviorPriority::SAFETY: return "SAFETY";
        case BehaviorPriority::MISSION_CRITICAL: return "MISSION_CRITICAL";
        case BehaviorPriority::HIGH: return "HIGH";
        case BehaviorPriority::NORMAL: return "NORMAL";
        case BehaviorPriority::LOW: return "LOW";
        case BehaviorPriority::BACKGROUND: return "BACKGROUND";
        default: return "NORMAL";
    }
}

inline BehaviorPriority stringToBehaviorPriority(const std::string& priority_str) {
    if (priority_str == "EMERGENCY") return BehaviorPriority::EMERGENCY;
    if (priority_str == "SAFETY") return BehaviorPriority::SAFETY;
    if (priority_str == "MISSION_CRITICAL") return BehaviorPriority::MISSION_CRITICAL;
    if (priority_str == "HIGH") return BehaviorPriority::HIGH;
    if (priority_str == "LOW") return BehaviorPriority::LOW;
    if (priority_str == "BACKGROUND") return BehaviorPriority::BACKGROUND;
    return BehaviorPriority::NORMAL;
}

// Comparador para prioridades de comportamientos
inline bool hasHigherPriority(BehaviorPriority a, BehaviorPriority b) {
    return static_cast<int>(a) < static_cast<int>(b);
}

// Validaciones
inline bool isValidBehaviorName(const std::string& name) {
    return !name.empty() && name.length() <= 64 && 
           name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") == std::string::npos;
}

inline bool isEmergencyState(RobotState state) {
    return state == RobotState::EMERGENCY;
}

inline bool canPreempt(const BehaviorParameters& current, const BehaviorParameters& incoming) {
    return current.can_be_preempted && hasHigherPriority(incoming.priority, current.priority);
}

} // namespace tadeo_ecar_behavior

#endif // TADEO_ECAR_BEHAVIOR_TYPES_HPP_