#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tadeo_ecar_msgs/msg/safety_status.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/srv/emergency_stop.hpp>
#include "tadeo_ecar_safety/safety_types.hpp"
#include <chrono>
#include <vector>
#include <memory>

namespace tadeo_ecar_safety
{

class EmergencyStopNode : public rclcpp::Node
{
public:
    EmergencyStopNode() : Node("emergency_stop_node")
    {
        loadParameters();
        initializeEmergencySystem();
        
        // Subscribers
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&EmergencyStopNode::joyCallback, this, std::placeholders::_1));
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_input", 10,
            std::bind(&EmergencyStopNode::cmdVelCallback, this, std::placeholders::_1));
        
        safety_status_sub_ = this->create_subscription<tadeo_ecar_msgs::msg::SafetyStatus>(
            "safety_status", 10,
            std::bind(&EmergencyStopNode::safetyStatusCallback, this, std::placeholders::_1));
        
        system_health_sub_ = this->create_subscription<tadeo_ecar_msgs::msg::SystemHealth>(
            "system_health", 10,
            std::bind(&EmergencyStopNode::systemHealthCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);
        emergency_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("emergency_stop_active", 10);
        emergency_reason_pub_ = this->create_publisher<std_msgs::msg::String>("emergency_reason", 10);
        safety_override_pub_ = this->create_publisher<std_msgs::msg::Bool>("safety_override", 10);
        
        // Services
        emergency_stop_service_ = this->create_service<tadeo_ecar_interfaces::srv::EmergencyStop>(
            "emergency_stop",
            std::bind(&EmergencyStopNode::emergencyStopService, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        // Timer for monitoring
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / monitor_frequency_)),
            std::bind(&EmergencyStopNode::monitorLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Emergency Stop Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("monitor.frequency", 20.0);
        this->declare_parameter("emergency.enable_joystick", true);
        this->declare_parameter("emergency.joystick_button", 0);
        this->declare_parameter("emergency.auto_reset_timeout", 5.0);
        this->declare_parameter("emergency.require_manual_reset", true);
        this->declare_parameter("safety.max_cmd_age", 1.0);
        this->declare_parameter("safety.enable_heartbeat", true);
        this->declare_parameter("safety.heartbeat_timeout", 2.0);
        
        monitor_frequency_ = this->get_parameter("monitor.frequency").as_double();
        enable_joystick_ = this->get_parameter("emergency.enable_joystick").as_bool();
        emergency_button_ = this->get_parameter("emergency.joystick_button").as_int();
        auto_reset_timeout_ = this->get_parameter("emergency.auto_reset_timeout").as_double();
        require_manual_reset_ = this->get_parameter("emergency.require_manual_reset").as_bool();
        max_cmd_age_ = this->get_parameter("safety.max_cmd_age").as_double();
        enable_heartbeat_ = this->get_parameter("safety.enable_heartbeat").as_bool();
        heartbeat_timeout_ = this->get_parameter("safety.heartbeat_timeout").as_double();
    }
    
    void initializeEmergencySystem()
    {
        emergency_active_ = false;
        safety_override_active_ = false;
        emergency_reason_ = "";
        last_cmd_time_ = this->now();
        last_heartbeat_time_ = this->now();
        emergency_triggered_time_ = this->now();
        
        // Initialize emergency conditions
        active_conditions_.clear();
        
        RCLCPP_INFO(this->get_logger(), "Emergency system ready - monitoring enabled");
    }
    
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!enable_joystick_ || msg->buttons.size() <= static_cast<size_t>(emergency_button_)) {
            return;
        }
        
        // Check emergency button press
        if (msg->buttons[emergency_button_] == 1) {
            if (!emergency_active_) {
                triggerEmergencyStop("Joystick emergency button pressed", EmergencyType::USER_EMERGENCY);
            }
        }
        
        // Check for reset combination (example: button 1 + button 2)
        if (msg->buttons.size() > 2 && msg->buttons[1] == 1 && msg->buttons[2] == 1) {
            if (emergency_active_) {
                resetEmergencyStop("Manual joystick reset");
            }
        }
        
        // Update heartbeat
        if (enable_heartbeat_) {
            last_heartbeat_time_ = this->now();
        }
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = this->now();
        current_cmd_vel_ = *msg;
        
        // Process command through safety filter
        processSafeCommand();
    }
    
    void safetyStatusCallback(const tadeo_ecar_msgs::msg::SafetyStatus::SharedPtr msg)
    {
        // Check for safety violations
        if (msg->emergency_stop) {
            triggerEmergencyStop("Safety system emergency", EmergencyType::SYSTEM_OVERLOAD);
        }
        
        if (msg->collision_warning && !emergency_active_) {
            RCLCPP_WARN(this->get_logger(), "Collision warning active");
        }
        
        if (msg->stability_warning && !emergency_active_) {
            RCLCPP_WARN(this->get_logger(), "Stability warning active");
        }
        
        // Update safety override
        safety_override_active_ = msg->safety_override;
    }
    
    void systemHealthCallback(const tadeo_ecar_msgs::msg::SystemHealth::SharedPtr msg)
    {
        // Check for critical system health issues
        if (msg->status == "ERROR" || msg->status == "CRITICAL") {
            std::string reason = "System health critical: " + msg->component_name + " - " + msg->error_message;
            
            // Add to active conditions if not already present
            bool found = false;
            for (auto& condition : active_conditions_) {
                if (condition.source_component == msg->component_name) {
                    condition.timestamp = std::chrono::steady_clock::now();
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                EmergencyCondition condition;
                condition.type = EmergencyType::SENSOR_FAILURE;
                condition.description = reason;
                condition.severity = SafetyLevel::CRITICAL;
                condition.timestamp = std::chrono::steady_clock::now();
                condition.is_active = true;
                condition.source_component = msg->component_name;
                
                active_conditions_.push_back(condition);
                
                if (!emergency_active_) {
                    triggerEmergencyStop(reason, EmergencyType::SENSOR_FAILURE);
                }
            }
        }
    }
    
    void emergencyStopService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::EmergencyStop::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::EmergencyStop::Response> response)
    {
        if (request->activate) {
            triggerEmergencyStop("Service call emergency stop", EmergencyType::USER_EMERGENCY);
            response->success = true;
            response->message = "Emergency stop activated";
        } else {
            resetEmergencyStop("Service call reset");
            response->success = true;
            response->message = "Emergency stop reset";
        }
        
        RCLCPP_INFO(this->get_logger(), "Emergency stop service called: %s", 
                    request->activate ? "ACTIVATE" : "RESET");
    }
    
    void triggerEmergencyStop(const std::string& reason, EmergencyType type)
    {
        if (emergency_active_) {
            return; // Already in emergency state
        }
        
        emergency_active_ = true;
        emergency_reason_ = reason;
        emergency_triggered_time_ = this->now();
        
        // Add emergency condition
        EmergencyCondition condition;
        condition.type = type;
        condition.description = reason;
        condition.severity = SafetyLevel::EMERGENCY;
        condition.timestamp = std::chrono::steady_clock::now();
        condition.is_active = true;
        condition.source_component = "emergency_stop";
        
        active_conditions_.push_back(condition);
        
        // Immediately stop the robot
        auto stop_cmd = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(stop_cmd);
        
        // Publish emergency status
        publishEmergencyStatus();
        
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP TRIGGERED: %s", reason.c_str());
    }
    
    void resetEmergencyStop(const std::string& reason)
    {
        if (!emergency_active_) {
            return; // Not in emergency state
        }
        
        // Check if manual reset is required and conditions are clear
        if (require_manual_reset_ && !active_conditions_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot reset: Active emergency conditions remain");
            return;
        }
        
        emergency_active_ = false;
        emergency_reason_ = "";
        active_conditions_.clear();
        
        // Publish reset status
        publishEmergencyStatus();
        
        RCLCPP_INFO(this->get_logger(), "EMERGENCY STOP RESET: %s", reason.c_str());
    }
    
    void processSafeCommand()
    {
        auto safe_cmd = current_cmd_vel_;
        
        // Check if emergency stop is active
        if (emergency_active_ && !safety_override_active_) {
            // Force stop command
            safe_cmd.linear.x = 0.0;
            safe_cmd.linear.y = 0.0;
            safe_cmd.linear.z = 0.0;
            safe_cmd.angular.x = 0.0;
            safe_cmd.angular.y = 0.0;
            safe_cmd.angular.z = 0.0;
        }
        
        // Publish safe command
        cmd_vel_pub_->publish(safe_cmd);
    }
    
    void monitorLoop()
    {
        auto current_time = this->now();
        
        // Check command timeout
        double cmd_age = (current_time - last_cmd_time_).seconds();
        if (cmd_age > max_cmd_age_ && !emergency_active_) {
            // Publish stop command on timeout
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);
        }
        
        // Check heartbeat timeout
        if (enable_heartbeat_) {
            double heartbeat_age = (current_time - last_heartbeat_time_).seconds();
            if (heartbeat_age > heartbeat_timeout_ && !emergency_active_) {
                triggerEmergencyStop("Heartbeat timeout - communication lost", 
                                     EmergencyType::COMMUNICATION_LOSS);
            }
        }
        
        // Check auto-reset conditions
        if (emergency_active_ && !require_manual_reset_) {
            double emergency_age = (current_time - emergency_triggered_time_).seconds();
            if (emergency_age > auto_reset_timeout_ && active_conditions_.empty()) {
                resetEmergencyStop("Auto-reset timeout");
            }
        }
        
        // Clean up expired conditions
        cleanupExpiredConditions();
        
        // Publish current status
        publishEmergencyStatus();
    }
    
    void cleanupExpiredConditions()
    {
        auto now = std::chrono::steady_clock::now();
        auto it = active_conditions_.begin();
        
        while (it != active_conditions_.end()) {
            auto age = std::chrono::duration_cast<std::chrono::seconds>(now - it->timestamp).count();
            
            // Remove conditions older than 5 seconds (configurable)
            if (age > 5) {
                it = active_conditions_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void publishEmergencyStatus()
    {
        // Publish emergency active status
        auto emergency_msg = std_msgs::msg::Bool();
        emergency_msg.data = emergency_active_;
        emergency_status_pub_->publish(emergency_msg);
        
        // Publish emergency reason
        auto reason_msg = std_msgs::msg::String();
        reason_msg.data = emergency_reason_;
        emergency_reason_pub_->publish(reason_msg);
        
        // Publish safety override status
        auto override_msg = std_msgs::msg::Bool();
        override_msg.data = safety_override_active_;
        safety_override_pub_->publish(override_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::SafetyStatus>::SharedPtr safety_status_sub_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr system_health_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emergency_reason_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_override_pub_;
    
    rclcpp::Service<tadeo_ecar_interfaces::srv::EmergencyStop>::SharedPtr emergency_stop_service_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    
    // Parameters
    double monitor_frequency_;
    bool enable_joystick_;
    int emergency_button_;
    double auto_reset_timeout_;
    bool require_manual_reset_;
    double max_cmd_age_;
    bool enable_heartbeat_;
    double heartbeat_timeout_;
    
    // State variables
    bool emergency_active_;
    bool safety_override_active_;
    std::string emergency_reason_;
    rclcpp::Time last_cmd_time_;
    rclcpp::Time last_heartbeat_time_;
    rclcpp::Time emergency_triggered_time_;
    geometry_msgs::msg::Twist current_cmd_vel_;
    std::vector<EmergencyCondition> active_conditions_;
};

} // namespace tadeo_ecar_safety

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_safety::EmergencyStopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}