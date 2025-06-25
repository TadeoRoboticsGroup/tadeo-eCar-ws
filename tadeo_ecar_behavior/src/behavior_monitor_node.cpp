#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include "tadeo_ecar_behavior/behavior_types.hpp"
#include <chrono>
#include <fstream>
#include <filesystem>
#include <memory>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <ctime>

namespace tadeo_ecar_behavior
{

class BehaviorMonitorNode : public rclcpp::Node
{
public:
    BehaviorMonitorNode() : Node("behavior_monitor_node")
    {
        loadParameters();
        initializeMonitor();
        
        // Suscriptores
        behavior_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "behavior_status", 10,
            std::bind(&BehaviorMonitorNode::behaviorStatusCallback, this, std::placeholders::_1));
        
        robot_state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_state", 10,
            std::bind(&BehaviorMonitorNode::robotStateCallback, this, std::placeholders::_1));
        
        behavior_metrics_sub_ = this->create_subscription<std_msgs::msg::String>(
            "behavior_metrics", 10,
            std::bind(&BehaviorMonitorNode::behaviorMetricsCallback, this, std::placeholders::_1));
        
        system_alerts_sub_ = this->create_subscription<std_msgs::msg::String>(
            "system_alerts", 10,
            std::bind(&BehaviorMonitorNode::systemAlertsCallback, this, std::placeholders::_1));
        
        // Publicadores
        behavior_performance_pub_ = this->create_publisher<std_msgs::msg::Float64>("behavior_performance", 10);
        behavior_diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>("behavior_diagnostics", 10);
        behavior_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("behavior_monitor_viz", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("behavior/monitor_health", 10);
        
        // Timer principal
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / monitor_frequency_)),
            std::bind(&BehaviorMonitorNode::monitorLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Behavior Monitor Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("monitor_frequency", 2.0);
        this->declare_parameter("enable_performance_logging", true);
        this->declare_parameter("log_directory", "/tmp/tadeo_behavior_monitor/");
        this->declare_parameter("performance_threshold", 0.7);
        this->declare_parameter("alert_threshold", 5);
        this->declare_parameter("base_frame", "base_link");
        
        monitor_frequency_ = this->get_parameter("monitor_frequency").as_double();
        enable_performance_logging_ = this->get_parameter("enable_performance_logging").as_bool();
        log_directory_ = this->get_parameter("log_directory").as_string();
        performance_threshold_ = this->get_parameter("performance_threshold").as_double();
        alert_threshold_ = this->get_parameter("alert_threshold").as_int();
        base_frame_ = this->get_parameter("base_frame").as_string();
    }
    
    void initializeMonitor()
    {
        // Inicializar métricas de monitoreo
        monitor_metrics_.session_start_time = this->now();
        monitor_metrics_.total_behaviors_monitored = 0;
        monitor_metrics_.performance_alerts = 0;
        monitor_metrics_.system_alerts = 0;
        monitor_metrics_.average_performance = 1.0;
        
        // Crear directorio de logs
        if (enable_performance_logging_) {
            try {
                std::filesystem::create_directories(log_directory_);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create log directory: %s", e.what());
                enable_performance_logging_ = false;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Behavior monitor initialized");
    }
    
    void monitorLoop()
    {
        // Analizar rendimiento del sistema de comportamientos
        analyzePerformance();
        
        // Detectar anomalías
        detectAnomalies();
        
        // Publicar métricas
        publishPerformanceMetrics();
        publishDiagnostics();
        publishVisualization();
        publishHealthStatus();
        
        // Logging si está habilitado
        if (enable_performance_logging_) {
            logMonitoringData();
        }
    }
    
    void analyzePerformance()
    {
        auto current_time = this->now();
        
        // Calcular rendimiento basado en métricas recientes
        double performance_score = 1.0;
        
        // Factor 1: Frecuencia de actualización de estado
        double state_freshness = calculateStateFreshness();
        
        // Factor 2: Tasa de éxito de comportamientos
        double success_rate = calculateSuccessRate();
        
        // Factor 3: Tiempo de respuesta del sistema
        double response_time = calculateResponseTime();
        
        // Combinar factores
        performance_score = (state_freshness * 0.3 + success_rate * 0.5 + response_time * 0.2);
        performance_score = std::clamp(performance_score, 0.0, 1.0);
        
        // Actualizar métricas
        monitor_metrics_.current_performance = performance_score;
        updateAveragePerformance(performance_score);
        
        // Verificar umbral de rendimiento
        if (performance_score < performance_threshold_) {
            monitor_metrics_.performance_alerts++;
            RCLCPP_WARN(this->get_logger(), 
                        "Performance below threshold: %.2f < %.2f", 
                        performance_score, performance_threshold_);
        }
    }
    
    double calculateStateFreshness()
    {
        auto current_time = this->now();
        
        // Verificar qué tan recientes son las actualizaciones de estado
        double behavior_freshness = 1.0;
        double state_freshness = 1.0;
        
        if (last_behavior_update_.seconds() > 0) {
            double behavior_age = (current_time - last_behavior_update_).seconds();
            behavior_freshness = std::max(0.0, 1.0 - (behavior_age / 10.0)); // 10 segundos máximo
        }
        
        if (last_state_update_.seconds() > 0) {
            double state_age = (current_time - last_state_update_).seconds();
            state_freshness = std::max(0.0, 1.0 - (state_age / 5.0)); // 5 segundos máximo
        }
        
        return (behavior_freshness + state_freshness) / 2.0;
    }
    
    double calculateSuccessRate()
    {
        // Analizar métricas de comportamientos para calcular tasa de éxito
        // Por ahora usamos un valor placeholder basado en alertas
        double base_rate = 1.0;
        
        if (monitor_metrics_.system_alerts > 0) {
            base_rate -= (monitor_metrics_.system_alerts * 0.1);
        }
        
        return std::clamp(base_rate, 0.0, 1.0);
    }
    
    double calculateResponseTime()
    {
        // Calcular tiempo de respuesta del sistema
        // Placeholder - en implementación real mediría latencia de respuesta
        return 0.9; // Asumir buen tiempo de respuesta por defecto
    }
    
    void updateAveragePerformance(double current_performance)
    {
        // Media móvil simple
        const double alpha = 0.1; // Factor de suavizado
        monitor_metrics_.average_performance = 
            alpha * current_performance + (1.0 - alpha) * monitor_metrics_.average_performance;
    }
    
    void detectAnomalies()
    {
        auto current_time = this->now();
        
        // Detectar anomalías en el comportamiento del sistema
        std::vector<std::string> anomalies;
        
        // Anomalía 1: No hay actualizaciones de estado por mucho tiempo
        if (last_state_update_.seconds() > 0) {
            double state_age = (current_time - last_state_update_).seconds();
            if (state_age > 30.0) {
                anomalies.push_back("No state updates for " + std::to_string(state_age) + " seconds");
            }
        }
        
        // Anomalía 2: Muchas alertas del sistema
        if (recent_alerts_.size() > alert_threshold_) {
            anomalies.push_back("High alert frequency: " + std::to_string(recent_alerts_.size()) + " alerts");
        }
        
        // Anomalía 3: Rendimiento consistentemente bajo
        if (monitor_metrics_.average_performance < performance_threshold_ * 0.8) {
            anomalies.push_back("Consistently low performance: " + 
                               std::to_string(monitor_metrics_.average_performance));
        }
        
        // Procesar anomalías detectadas
        for (const auto& anomaly : anomalies) {
            RCLCPP_WARN(this->get_logger(), "ANOMALY DETECTED: %s", anomaly.c_str());
            detected_anomalies_.push_back({current_time, anomaly});
        }
        
        // Limpiar anomalías antiguas (>1 hora)
        auto cutoff_time = current_time - rclcpp::Duration::from_nanoseconds(3600 * 1e9);
        detected_anomalies_.erase(
            std::remove_if(detected_anomalies_.begin(), detected_anomalies_.end(),
                [cutoff_time](const auto& anomaly) { 
                    return anomaly.first < cutoff_time; 
                }),
            detected_anomalies_.end());
    }
    
    void publishPerformanceMetrics()
    {
        std_msgs::msg::Float64 performance_msg;
        performance_msg.data = monitor_metrics_.current_performance;
        behavior_performance_pub_->publish(performance_msg);
    }
    
    void publishDiagnostics()
    {
        std_msgs::msg::String diagnostics_msg;
        
        std::ostringstream oss;
        oss << "Performance: " << std::fixed << std::setprecision(2) << monitor_metrics_.current_performance
            << " | Avg: " << monitor_metrics_.average_performance
            << " | Alerts: " << monitor_metrics_.system_alerts
            << " | Anomalies: " << detected_anomalies_.size()
            << " | Uptime: " << (this->now() - monitor_metrics_.session_start_time).seconds() << "s";
        
        diagnostics_msg.data = oss.str();
        behavior_diagnostics_pub_->publish(diagnostics_msg);
    }
    
    void publishVisualization()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Marcador de rendimiento
        visualization_msgs::msg::Marker performance_marker;
        performance_marker.header.frame_id = base_frame_;
        performance_marker.header.stamp = this->now();
        performance_marker.ns = "behavior_monitor";
        performance_marker.id = 0;
        performance_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        performance_marker.action = visualization_msgs::msg::Marker::ADD;
        
        performance_marker.pose.position.x = 0.0;
        performance_marker.pose.position.y = 0.0;
        performance_marker.pose.position.z = 2.0;
        performance_marker.pose.orientation.w = 1.0;
        
        performance_marker.text = "Performance: " + 
                                 std::to_string(static_cast<int>(monitor_metrics_.current_performance * 100)) + "%";
        performance_marker.scale.z = 0.3;
        
        // Color basado en rendimiento
        if (monitor_metrics_.current_performance > 0.8) {
            performance_marker.color.r = 0.0; performance_marker.color.g = 1.0; performance_marker.color.b = 0.0;
        } else if (monitor_metrics_.current_performance > 0.6) {
            performance_marker.color.r = 1.0; performance_marker.color.g = 1.0; performance_marker.color.b = 0.0;
        } else {
            performance_marker.color.r = 1.0; performance_marker.color.g = 0.0; performance_marker.color.b = 0.0;
        }
        performance_marker.color.a = 1.0;
        
        marker_array.markers.push_back(performance_marker);
        
        behavior_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        health_msg.component_name = "behavior_monitor";
        
        // Determinar estado de salud
        if (monitor_metrics_.current_performance < 0.3) {
            health_msg.status = "ERROR";
            health_msg.error_code = 21001;
            health_msg.error_message = "Critical performance degradation";
        } else if (monitor_metrics_.current_performance < performance_threshold_) {
            health_msg.status = "WARNING";
            health_msg.error_code = 21002;
            health_msg.error_message = "Performance below threshold";
        } else if (!detected_anomalies_.empty()) {
            health_msg.status = "WARNING";
            health_msg.error_code = 21003;
            health_msg.error_message = "Anomalies detected in behavior system";
        } else {
            health_msg.status = "OK";
            health_msg.error_code = 0;
            health_msg.error_message = "";
        }
        
        health_msg.cpu_usage = 2.0; // Placeholder
        health_msg.memory_usage = 4.0; // Placeholder
        health_msg.temperature = 33.0; // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    void logMonitoringData()
    {
        static auto last_log_time = this->now();
        auto current_time = this->now();
        
        // Hacer log cada 10 segundos
        if ((current_time - last_log_time).seconds() < 10.0) return;
        
        try {
            std::string log_filename = log_directory_ + "behavior_monitor_" +
                                      std::to_string(std::time(nullptr)) + ".log";
            
            std::ofstream log_file(log_filename, std::ios::app);
            
            log_file << current_time.seconds() << ","
                     << monitor_metrics_.current_performance << ","
                     << monitor_metrics_.average_performance << ","
                     << monitor_metrics_.system_alerts << ","
                     << detected_anomalies_.size() << ","
                     << current_robot_state_ << "\n";
            
            log_file.close();
            last_log_time = current_time;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to log monitoring data: %s", e.what());
        }
    }
    
    // Callbacks
    void behaviorStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_behavior_status_ = msg->data;
        last_behavior_update_ = this->now();
        monitor_metrics_.total_behaviors_monitored++;
    }
    
    void robotStateCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_robot_state_ = msg->data;
        last_state_update_ = this->now();
    }
    
    void behaviorMetricsCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_behavior_metrics_ = msg->data;
        // Aquí se podría parsear las métricas para análisis más detallado
    }
    
    void systemAlertsCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string alert = msg->data;
        auto current_time = this->now();
        
        recent_alerts_.push_back({current_time, alert});
        monitor_metrics_.system_alerts++;
        
        // Limpiar alertas antiguas (>5 minutos)
        auto cutoff_time = current_time - rclcpp::Duration::from_nanoseconds(300 * 1e9);
        recent_alerts_.erase(
            std::remove_if(recent_alerts_.begin(), recent_alerts_.end(),
                [cutoff_time](const auto& alert) { 
                    return alert.first < cutoff_time; 
                }),
            recent_alerts_.end());
        
        RCLCPP_INFO(this->get_logger(), "System alert received: %s", alert.c_str());
    }
    
    // Estructura para métricas de monitoreo
    struct MonitorMetrics {
        rclcpp::Time session_start_time;
        int total_behaviors_monitored;
        int performance_alerts;
        int system_alerts;
        double current_performance;
        double average_performance;
    };
    
    // Variables miembro
    double monitor_frequency_;
    bool enable_performance_logging_;
    std::string log_directory_;
    double performance_threshold_;
    int alert_threshold_;
    std::string base_frame_;
    
    // Estado actual
    std::string current_behavior_status_;
    std::string current_robot_state_;
    std::string current_behavior_metrics_;
    rclcpp::Time last_behavior_update_;
    rclcpp::Time last_state_update_;
    
    // Métricas de monitoreo
    MonitorMetrics monitor_metrics_;
    
    // Alertas y anomalías recientes
    std::vector<std::pair<rclcpp::Time, std::string>> recent_alerts_;
    std::vector<std::pair<rclcpp::Time, std::string>> detected_anomalies_;
    
    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_metrics_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_alerts_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr behavior_performance_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_diagnostics_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr behavior_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::TimerInterface::SharedPtr monitor_timer_;
};

} // namespace tadeo_ecar_behavior

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_behavior::BehaviorMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}