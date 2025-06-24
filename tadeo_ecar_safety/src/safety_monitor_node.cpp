#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tadeo_ecar_msgs/msg/safety_status.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_msgs/msg/robot_status.hpp>
#include <tadeo_ecar_interfaces/srv/emergency_stop.hpp>
#include "tadeo_ecar_safety/safety_types.hpp"
#include <cmath>
#include <map>
#include <vector>
#include <chrono>

namespace tadeo_ecar_safety
{

class SafetyMonitorNode : public rclcpp::Node
{
public:
    SafetyMonitorNode() : Node("safety_monitor_node")
    {
        loadParameters();
        initializeSafetyLimits();
        
        // Subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&SafetyMonitorNode::imuCallback, this, std::placeholders::_1));
        
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10,
            std::bind(&SafetyMonitorNode::batteryCallback, this, std::placeholders::_1));
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&SafetyMonitorNode::cmdVelCallback, this, std::placeholders::_1));
        
        robot_status_sub_ = this->create_subscription<tadeo_ecar_msgs::msg::RobotStatus>(
            "robot_status", 10,
            std::bind(&SafetyMonitorNode::robotStatusCallback, this, std::placeholders::_1));
        
        system_health_sub_ = this->create_subscription<tadeo_ecar_msgs::msg::SystemHealth>(
            "system_health", 10,
            std::bind(&SafetyMonitorNode::systemHealthCallback, this, std::placeholders::_1));
        
        // Publishers
        safety_status_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SafetyStatus>(
            "safety_monitor_status", 10);
        
        tilt_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("safety/tilt_angle", 10);
        lateral_accel_pub_ = this->create_publisher<std_msgs::msg::Float64>("safety/lateral_acceleration", 10);
        stability_factor_pub_ = this->create_publisher<std_msgs::msg::Float64>("safety/stability_factor", 10);
        battery_health_pub_ = this->create_publisher<std_msgs::msg::Float64>("safety/battery_health", 10);
        
        // Service client for emergency stop
        emergency_stop_client_ = this->create_client<tadeo_ecar_interfaces::srv::EmergencyStop>("emergency_stop");
        
        // Timer for safety monitoring
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / monitor_frequency_)),
            std::bind(&SafetyMonitorNode::monitorLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Safety Monitor Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("monitor.frequency", 50.0);
        this->declare_parameter("limits.max_tilt_angle", 0.524); // 30 degrees
        this->declare_parameter("limits.max_lateral_acceleration", 8.0);
        this->declare_parameter("limits.max_linear_velocity", 2.0);
        this->declare_parameter("limits.max_angular_velocity", 1.5);
        this->declare_parameter("limits.max_acceleration", 3.0);
        this->declare_parameter("limits.max_deceleration", -5.0);
        this->declare_parameter("battery.critical_voltage", 20.0);
        this->declare_parameter("battery.warning_voltage", 22.0);
        this->declare_parameter("battery.critical_percentage", 10.0);
        this->declare_parameter("battery.warning_percentage", 20.0);
        this->declare_parameter("stability.min_factor", 0.3);
        this->declare_parameter("stability.warning_factor", 0.5);
        this->declare_parameter("temperature.max_operating", 70.0);
        this->declare_parameter("temperature.critical", 85.0);
        
        monitor_frequency_ = this->get_parameter("monitor.frequency").as_double();
        
        safety_limits_.max_tilt_angle = this->get_parameter("limits.max_tilt_angle").as_double();
        safety_limits_.max_lateral_acceleration = this->get_parameter("limits.max_lateral_acceleration").as_double();
        safety_limits_.max_linear_velocity = this->get_parameter("limits.max_linear_velocity").as_double();
        safety_limits_.max_angular_velocity = this->get_parameter("limits.max_angular_velocity").as_double();
        safety_limits_.max_acceleration = this->get_parameter("limits.max_acceleration").as_double();
        safety_limits_.max_deceleration = this->get_parameter("limits.max_deceleration").as_double();
        
        critical_battery_voltage_ = this->get_parameter("battery.critical_voltage").as_double();
        warning_battery_voltage_ = this->get_parameter("battery.warning_voltage").as_double();
        critical_battery_percentage_ = this->get_parameter("battery.critical_percentage").as_double();
        warning_battery_percentage_ = this->get_parameter("battery.warning_percentage").as_double();
        
        min_stability_factor_ = this->get_parameter("stability.min_factor").as_double();
        warning_stability_factor_ = this->get_parameter("stability.warning_factor").as_double();
        
        max_operating_temperature_ = this->get_parameter("temperature.max_operating").as_double();
        critical_temperature_ = this->get_parameter("temperature.critical").as_double();
    }
    
    void initializeSafetyLimits()
    {
        current_tilt_angle_ = 0.0;
        current_lateral_acceleration_ = 0.0;
        current_stability_factor_ = 1.0;
        battery_health_factor_ = 1.0;
        
        last_velocity_time_ = this->now();
        previous_velocity_ = 0.0;
        current_acceleration_ = 0.0;
        
        // Initialize safety status
        current_safety_level_ = SafetyLevel::SAFE;
        active_warnings_.clear();
        
        RCLCPP_INFO(this->get_logger(), "Safety limits initialized");
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        current_imu_ = *msg;
        
        // Calculate tilt angle from orientation
        calculateTiltAngle();
        
        // Extract lateral acceleration
        current_lateral_acceleration_ = std::abs(msg->linear_acceleration.y);
        
        // Calculate stability factor
        calculateStabilityFactor();
        
        // Check IMU-based safety conditions
        checkStabilityLimits();
    }
    
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        current_battery_ = *msg;
        
        // Calculate battery health factor
        calculateBatteryHealth();
        
        // Check battery safety conditions
        checkBatteryLimits();
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_cmd_vel_ = *msg;
        
        // Check velocity limits
        checkVelocityLimits();
        
        // Calculate acceleration
        calculateAcceleration();
    }
    
    void robotStatusCallback(const tadeo_ecar_msgs::msg::RobotStatus::SharedPtr msg)
    {
        current_robot_status_ = *msg;
        
        // Check temperature limits
        checkTemperatureLimits();
        
        // Update overall system status
        updateSystemStatus();
    }
    
    void systemHealthCallback(const tadeo_ecar_msgs::msg::SystemHealth::SharedPtr msg)
    {
        // Store health information by component
        component_health_[msg->component_name] = *msg;
        
        // Check for critical system health issues
        checkSystemHealth(*msg);
    }
    
    void calculateTiltAngle()
    {
        // Extract roll and pitch from quaternion
        double x = current_imu_.orientation.x;
        double y = current_imu_.orientation.y;
        double z = current_imu_.orientation.z;
        double w = current_imu_.orientation.w;
        
        // Convert to Euler angles
        double roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        double pitch = asin(2.0 * (w * y - z * x));
        
        // Calculate overall tilt angle
        current_tilt_angle_ = sqrt(roll * roll + pitch * pitch);
    }
    
    void calculateStabilityFactor()
    {
        // Simple stability factor based on tilt angle and lateral acceleration
        double tilt_factor = 1.0 - (current_tilt_angle_ / safety_limits_.max_tilt_angle);
        double accel_factor = 1.0 - (current_lateral_acceleration_ / safety_limits_.max_lateral_acceleration);
        
        tilt_factor = std::clamp(tilt_factor, 0.0, 1.0);
        accel_factor = std::clamp(accel_factor, 0.0, 1.0);
        
        current_stability_factor_ = std::min(tilt_factor, accel_factor);
    }
    
    void calculateBatteryHealth()
    {
        if (current_battery_.voltage > 0) {
            // Simple battery health based on voltage and percentage
            double voltage_factor = std::clamp(
                (current_battery_.voltage - critical_battery_voltage_) / 
                (warning_battery_voltage_ - critical_battery_voltage_), 0.0, 1.0);
            
            double percentage_factor = std::clamp(
                (current_battery_.percentage - critical_battery_percentage_) / 
                (warning_battery_percentage_ - critical_battery_percentage_), 0.0, 1.0);
            
            battery_health_factor_ = std::min(voltage_factor, percentage_factor);
        }
    }
    
    void calculateAcceleration()
    {
        auto current_time = this->now();
        double dt = (current_time - last_velocity_time_).seconds();
        
        if (dt > 0.0 && dt < 1.0) { // Reasonable time step
            double current_velocity = current_robot_status_.current_speed;
            current_acceleration_ = (current_velocity - previous_velocity_) / dt;
            previous_velocity_ = current_velocity;
        }
        
        last_velocity_time_ = current_time;
    }
    
    void checkStabilityLimits()
    {
        // Check tilt angle
        if (current_tilt_angle_ > safety_limits_.max_tilt_angle) {
            addWarning("CRITICAL_TILT", "Robot tilt angle exceeds safety limit: " + 
                       std::to_string(current_tilt_angle_ * 180.0 / M_PI) + " degrees",
                       SafetyLevel::EMERGENCY);
        } else if (current_tilt_angle_ > safety_limits_.max_tilt_angle * 0.8) {
            addWarning("HIGH_TILT", "Robot tilt angle approaching limit",
                       SafetyLevel::WARNING);
        }
        
        // Check lateral acceleration
        if (current_lateral_acceleration_ > safety_limits_.max_lateral_acceleration) {
            addWarning("HIGH_LATERAL_ACCEL", "Lateral acceleration exceeds safety limit: " + 
                       std::to_string(current_lateral_acceleration_) + " m/s²",
                       SafetyLevel::CRITICAL);
        }
        
        // Check stability factor
        if (current_stability_factor_ < min_stability_factor_) {
            addWarning("LOW_STABILITY", "Vehicle stability factor critically low: " + 
                       std::to_string(current_stability_factor_),
                       SafetyLevel::EMERGENCY);
        } else if (current_stability_factor_ < warning_stability_factor_) {
            addWarning("STABILITY_WARNING", "Vehicle stability factor low",
                       SafetyLevel::WARNING);
        }
    }
    
    void checkBatteryLimits()
    {
        // Check voltage
        if (current_battery_.voltage < critical_battery_voltage_) {
            addWarning("CRITICAL_BATTERY_VOLTAGE", "Battery voltage critically low: " + 
                       std::to_string(current_battery_.voltage) + "V",
                       SafetyLevel::CRITICAL);
        } else if (current_battery_.voltage < warning_battery_voltage_) {
            addWarning("LOW_BATTERY_VOLTAGE", "Battery voltage low",
                       SafetyLevel::WARNING);
        }
        
        // Check percentage
        if (current_battery_.percentage < critical_battery_percentage_) {
            addWarning("CRITICAL_BATTERY_LEVEL", "Battery level critically low: " + 
                       std::to_string(current_battery_.percentage) + "%",
                       SafetyLevel::CRITICAL);
        } else if (current_battery_.percentage < warning_battery_percentage_) {
            addWarning("LOW_BATTERY_LEVEL", "Battery level low",
                       SafetyLevel::WARNING);
        }
    }
    
    void checkVelocityLimits()
    {
        // Check linear velocity
        double linear_speed = sqrt(current_cmd_vel_.linear.x * current_cmd_vel_.linear.x + 
                                   current_cmd_vel_.linear.y * current_cmd_vel_.linear.y);
        
        if (linear_speed > safety_limits_.max_linear_velocity) {
            addWarning("HIGH_LINEAR_VELOCITY", "Linear velocity exceeds limit: " + 
                       std::to_string(linear_speed) + " m/s",
                       SafetyLevel::WARNING);
        }
        
        // Check angular velocity
        if (std::abs(current_cmd_vel_.angular.z) > safety_limits_.max_angular_velocity) {
            addWarning("HIGH_ANGULAR_VELOCITY", "Angular velocity exceeds limit: " + 
                       std::to_string(current_cmd_vel_.angular.z) + " rad/s",
                       SafetyLevel::WARNING);
        }
        
        // Check acceleration
        if (current_acceleration_ > safety_limits_.max_acceleration) {
            addWarning("HIGH_ACCELERATION", "Acceleration exceeds limit: " + 
                       std::to_string(current_acceleration_) + " m/s²",
                       SafetyLevel::WARNING);
        } else if (current_acceleration_ < safety_limits_.max_deceleration) {
            addWarning("HIGH_DECELERATION", "Deceleration exceeds limit: " + 
                       std::to_string(current_acceleration_) + " m/s²",
                       SafetyLevel::WARNING);
        }
    }
    
    void checkTemperatureLimits()
    {
        if (current_robot_status_.system_temperature > critical_temperature_) {
            addWarning("CRITICAL_TEMPERATURE", "System temperature critically high: " + 
                       std::to_string(current_robot_status_.system_temperature) + "°C",
                       SafetyLevel::EMERGENCY);
        } else if (current_robot_status_.system_temperature > max_operating_temperature_) {
            addWarning("HIGH_TEMPERATURE", "System temperature high",
                       SafetyLevel::WARNING);
        }
    }
    
    void checkSystemHealth(const tadeo_ecar_msgs::msg::SystemHealth& health)
    {
        if (health.status == "ERROR" || health.status == "CRITICAL") {
            addWarning("SYSTEM_HEALTH_" + health.component_name, 
                       "Component health critical: " + health.component_name + " - " + health.error_message,
                       SafetyLevel::CRITICAL);
        }
    }
    
    void addWarning(const std::string& id, const std::string& message, SafetyLevel level)
    {
        EmergencyCondition condition;
        condition.type = EmergencyType::SYSTEM_OVERLOAD; // Generic type
        condition.description = message;
        condition.severity = level;
        condition.timestamp = std::chrono::steady_clock::now();
        condition.is_active = true;
        condition.source_component = "safety_monitor";
        
        active_warnings_[id] = condition;
        
        // Update overall safety level
        current_safety_level_ = std::max(current_safety_level_, level);
        
        // Trigger emergency stop if critical
        if (level == SafetyLevel::EMERGENCY) {
            triggerEmergencyStop(message);
        }
        
        RCLCPP_WARN(this->get_logger(), "Safety warning [%s]: %s", id.c_str(), message.c_str());
    }
    
    void triggerEmergencyStop(const std::string& reason)
    {
        if (!emergency_stop_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Emergency stop service not available");
            return;
        }
        
        auto request = std::make_shared<tadeo_ecar_interfaces::srv::EmergencyStop::Request>();
        request->activate = true;
        
        auto future = emergency_stop_client_->async_send_request(request);
        
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP TRIGGERED BY SAFETY MONITOR: %s", reason.c_str());
    }
    
    void updateSystemStatus()
    {
        // Calculate overall system health
        double overall_health = 1.0;
        int critical_components = 0;
        int total_components = component_health_.size();
        
        for (const auto& [name, health] : component_health_) {
            if (health.status == "ERROR" || health.status == "CRITICAL") {
                critical_components++;
            }
        }
        
        if (total_components > 0) {
            overall_health = 1.0 - (static_cast<double>(critical_components) / total_components);
        }
        
        // Combine with other health factors
        overall_health = std::min({overall_health, current_stability_factor_, battery_health_factor_});
        
        // Update safety level based on overall health
        if (overall_health < 0.3) {
            current_safety_level_ = SafetyLevel::EMERGENCY;
        } else if (overall_health < 0.6) {
            current_safety_level_ = SafetyLevel::CRITICAL;
        } else if (overall_health < 0.8) {
            current_safety_level_ = SafetyLevel::WARNING;
        } else {
            current_safety_level_ = SafetyLevel::SAFE;
        }
    }
    
    void monitorLoop()
    {
        // Clean up expired warnings
        cleanupExpiredWarnings();
        
        // Publish safety status
        publishSafetyStatus();
        
        // Publish individual metrics
        publishMetrics();
    }
    
    void cleanupExpiredWarnings()
    {
        auto now = std::chrono::steady_clock::now();
        auto it = active_warnings_.begin();
        
        while (it != active_warnings_.end()) {
            auto age = std::chrono::duration_cast<std::chrono::seconds>(now - it->second.timestamp).count();
            
            // Remove warnings older than 5 seconds
            if (age > 5) {
                it = active_warnings_.erase(it);
            } else {
                ++it;
            }
        }
        
        // Reset safety level if no active warnings
        if (active_warnings_.empty()) {
            current_safety_level_ = SafetyLevel::SAFE;
        }
    }
    
    void publishSafetyStatus()
    {
        auto safety_msg = tadeo_ecar_msgs::msg::SafetyStatus();
        safety_msg.header.stamp = this->now();
        safety_msg.header.frame_id = "base_link";
        
        safety_msg.emergency_stop = (current_safety_level_ == SafetyLevel::EMERGENCY);
        safety_msg.collision_warning = false; // Handled by collision avoidance
        safety_msg.stability_warning = (current_stability_factor_ < warning_stability_factor_);
        safety_msg.safety_override = false;
        
        safety_msg.closest_obstacle_distance = 0.0; // Not monitored by this node
        safety_msg.safety_zone_status = static_cast<int>(current_safety_level_);
        
        safety_msg.emergency_zone_clear = true; // Not monitored by this node
        safety_msg.warning_zone_clear = true;
        safety_msg.slow_zone_clear = true;
        
        safety_status_pub_->publish(safety_msg);
    }
    
    void publishMetrics()
    {
        // Publish tilt angle
        auto tilt_msg = std_msgs::msg::Float64();
        tilt_msg.data = current_tilt_angle_;
        tilt_angle_pub_->publish(tilt_msg);
        
        // Publish lateral acceleration
        auto accel_msg = std_msgs::msg::Float64();
        accel_msg.data = current_lateral_acceleration_;
        lateral_accel_pub_->publish(accel_msg);
        
        // Publish stability factor
        auto stability_msg = std_msgs::msg::Float64();
        stability_msg.data = current_stability_factor_;
        stability_factor_pub_->publish(stability_msg);
        
        // Publish battery health
        auto battery_msg = std_msgs::msg::Float64();
        battery_msg.data = battery_health_factor_;
        battery_health_pub_->publish(battery_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr system_health_sub_;
    
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SafetyStatus>::SharedPtr safety_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lateral_accel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stability_factor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr battery_health_pub_;
    
    rclcpp::Client<tadeo_ecar_interfaces::srv::EmergencyStop>::SharedPtr emergency_stop_client_;
    rclcpp::TimerInterface::SharedPtr monitor_timer_;
    
    // Parameters
    double monitor_frequency_;
    SafetyLimits safety_limits_;
    double critical_battery_voltage_;
    double warning_battery_voltage_;
    double critical_battery_percentage_;
    double warning_battery_percentage_;
    double min_stability_factor_;
    double warning_stability_factor_;
    double max_operating_temperature_;
    double critical_temperature_;
    
    // State variables
    sensor_msgs::msg::Imu current_imu_;
    sensor_msgs::msg::BatteryState current_battery_;
    geometry_msgs::msg::Twist current_cmd_vel_;
    tadeo_ecar_msgs::msg::RobotStatus current_robot_status_;
    std::map<std::string, tadeo_ecar_msgs::msg::SystemHealth> component_health_;
    
    double current_tilt_angle_;
    double current_lateral_acceleration_;
    double current_stability_factor_;
    double battery_health_factor_;
    double current_acceleration_;
    double previous_velocity_;
    rclcpp::Time last_velocity_time_;
    
    SafetyLevel current_safety_level_;
    std::map<std::string, EmergencyCondition> active_warnings_;
};

} // namespace tadeo_ecar_safety

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_safety::SafetyMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}