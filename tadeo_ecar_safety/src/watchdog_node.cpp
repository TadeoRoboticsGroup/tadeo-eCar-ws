#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/srv/emergency_stop.hpp>
#include "tadeo_ecar_safety/safety_types.hpp"
#include <chrono>
#include <map>
#include <vector>
#include <string>

namespace tadeo_ecar_safety
{

struct ComponentWatchdog
{
    std::string name;
    std::chrono::steady_clock::time_point last_heartbeat;
    double timeout_seconds;
    bool is_critical;
    bool is_alive;
    std::string last_message;
};

class WatchdogNode : public rclcpp::Node
{
public:
    WatchdogNode() : Node("watchdog_node")
    {
        loadParameters();
        initializeWatchdogs();
        
        // Subscribers for component heartbeats
        setupComponentSubscribers();
        
        // Publishers
        watchdog_status_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "watchdog_status", 10);
        
        system_alive_pub_ = this->create_publisher<std_msgs::msg::Bool>("system_alive", 10);
        failed_components_pub_ = this->create_publisher<std_msgs::msg::String>("failed_components", 10);
        
        // Service client for emergency stop
        emergency_stop_client_ = this->create_client<tadeo_ecar_interfaces::srv::EmergencyStop>("emergency_stop");
        
        // Watchdog timer
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / watchdog_frequency_)),
            std::bind(&WatchdogNode::watchdogLoop, this));
        
        // Heartbeat timer (this node's own heartbeat)
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / heartbeat_frequency_)),
            std::bind(&WatchdogNode::publishHeartbeat, this));
        
        RCLCPP_INFO(this->get_logger(), "Watchdog Node initialized - monitoring %zu components", 
                    component_watchdogs_.size());
    }

private:
    void loadParameters()
    {
        this->declare_parameter("watchdog.frequency", 10.0);
        this->declare_parameter("heartbeat.frequency", 1.0);
        this->declare_parameter("timeouts.control_system", 2.0);
        this->declare_parameter("timeouts.perception_system", 3.0);
        this->declare_parameter("timeouts.navigation_system", 5.0);
        this->declare_parameter("timeouts.safety_system", 1.0);
        this->declare_parameter("timeouts.hardware_interface", 2.0);
        this->declare_parameter("critical_components.control", true);
        this->declare_parameter("critical_components.perception", false);
        this->declare_parameter("critical_components.navigation", false);
        this->declare_parameter("critical_components.safety", true);
        this->declare_parameter("critical_components.hardware", true);
        this->declare_parameter("emergency.enable_auto_stop", true);
        this->declare_parameter("emergency.critical_component_failures", 2);
        this->declare_parameter("emergency.total_failure_threshold", 0.7);
        
        watchdog_frequency_ = this->get_parameter("watchdog.frequency").as_double();
        heartbeat_frequency_ = this->get_parameter("heartbeat.frequency").as_double();
        
        // Component timeouts
        component_timeouts_["control_system"] = this->get_parameter("timeouts.control_system").as_double();
        component_timeouts_["perception_system"] = this->get_parameter("timeouts.perception_system").as_double();
        component_timeouts_["navigation_system"] = this->get_parameter("timeouts.navigation_system").as_double();
        component_timeouts_["safety_system"] = this->get_parameter("timeouts.safety_system").as_double();
        component_timeouts_["hardware_interface"] = this->get_parameter("timeouts.hardware_interface").as_double();
        
        // Critical component flags
        critical_components_["control"] = this->get_parameter("critical_components.control").as_bool();
        critical_components_["perception"] = this->get_parameter("critical_components.perception").as_bool();
        critical_components_["navigation"] = this->get_parameter("critical_components.navigation").as_bool();
        critical_components_["safety"] = this->get_parameter("critical_components.safety").as_bool();
        critical_components_["hardware"] = this->get_parameter("critical_components.hardware").as_bool();
        
        enable_auto_emergency_stop_ = this->get_parameter("emergency.enable_auto_stop").as_bool();
        critical_failure_threshold_ = this->get_parameter("emergency.critical_component_failures").as_int();
        total_failure_threshold_ = this->get_parameter("emergency.total_failure_threshold").as_double();
    }
    
    void initializeWatchdogs()
    {
        component_watchdogs_.clear();
        
        // Initialize watchdogs for each monitored component
        std::vector<std::string> components = {
            "wheel_controller", "four_wheel_steering_controller", "vehicle_dynamics",
            "camera_processor", "lidar_processor", "imu_processor", "sensor_fusion",
            "emergency_stop", "collision_avoidance", "safety_monitor",
            "localization", "slam", "planner", "navigator", "behavior_tree"
        };
        
        auto now = std::chrono::steady_clock::now();
        
        for (const auto& component : components) {
            ComponentWatchdog watchdog;
            watchdog.name = component;
            watchdog.last_heartbeat = now;
            
            // Assign timeout based on component type
            if (component.find("control") != std::string::npos || 
                component.find("wheel") != std::string::npos ||
                component.find("steering") != std::string::npos ||
                component.find("dynamics") != std::string::npos) {
                watchdog.timeout_seconds = component_timeouts_["control_system"];
                watchdog.is_critical = critical_components_["control"];
            } else if (component.find("camera") != std::string::npos || 
                       component.find("lidar") != std::string::npos ||
                       component.find("imu") != std::string::npos ||
                       component.find("sensor") != std::string::npos ||
                       component.find("perception") != std::string::npos) {
                watchdog.timeout_seconds = component_timeouts_["perception_system"];
                watchdog.is_critical = critical_components_["perception"];
            } else if (component.find("emergency") != std::string::npos || 
                       component.find("safety") != std::string::npos ||
                       component.find("collision") != std::string::npos) {
                watchdog.timeout_seconds = component_timeouts_["safety_system"];
                watchdog.is_critical = critical_components_["safety"];
            } else if (component.find("navigation") != std::string::npos || 
                       component.find("planner") != std::string::npos ||
                       component.find("navigator") != std::string::npos ||
                       component.find("behavior") != std::string::npos) {
                watchdog.timeout_seconds = component_timeouts_["navigation_system"];
                watchdog.is_critical = critical_components_["navigation"];
            } else {
                watchdog.timeout_seconds = component_timeouts_["hardware_interface"];
                watchdog.is_critical = critical_components_["hardware"];
            }
            
            watchdog.is_alive = true;
            watchdog.last_message = "Initialized";
            
            component_watchdogs_[component] = watchdog;
        }
        
        RCLCPP_INFO(this->get_logger(), "Initialized watchdogs for %zu components", component_watchdogs_.size());
    }
    
    void setupComponentSubscribers()
    {
        // Create subscribers for each component's health status
        for (const auto& [name, watchdog] : component_watchdogs_) {
            std::string topic_name = name + "/health";
            
            auto sub = this->create_subscription<tadeo_ecar_msgs::msg::SystemHealth>(
                topic_name, 10,
                [this, name](const tadeo_ecar_msgs::msg::SystemHealth::SharedPtr msg) {
                    this->componentHealthCallback(name, msg);
                });
            
            component_subscribers_[name] = sub;
        }
        
        // Also subscribe to generic health topics
        generic_health_sub_ = this->create_subscription<tadeo_ecar_msgs::msg::SystemHealth>(
            "system_health", 10,
            std::bind(&WatchdogNode::genericHealthCallback, this, std::placeholders::_1));
    }
    
    void componentHealthCallback(const std::string& component_name, 
                                 const tadeo_ecar_msgs::msg::SystemHealth::SharedPtr msg)
    {
        auto it = component_watchdogs_.find(component_name);
        if (it != component_watchdogs_.end()) {
            it->second.last_heartbeat = std::chrono::steady_clock::now();
            it->second.is_alive = (msg->status != "ERROR" && msg->status != "CRITICAL");
            it->second.last_message = msg->error_message;
            
            if (!it->second.is_alive) {
                RCLCPP_WARN(this->get_logger(), "Component %s reported unhealthy status: %s", 
                            component_name.c_str(), msg->status.c_str());
            }
        }
    }
    
    void genericHealthCallback(const tadeo_ecar_msgs::msg::SystemHealth::SharedPtr msg)
    {
        // Update watchdog for the component if it exists
        auto it = component_watchdogs_.find(msg->component_name);
        if (it != component_watchdogs_.end()) {
            it->second.last_heartbeat = std::chrono::steady_clock::now();
            it->second.is_alive = (msg->status != "ERROR" && msg->status != "CRITICAL");
            it->second.last_message = msg->error_message;
        }
    }
    
    void watchdogLoop()
    {
        auto now = std::chrono::steady_clock::now();
        
        // Check each component
        int failed_components = 0;
        int critical_failed_components = 0;
        int total_components = component_watchdogs_.size();
        std::vector<std::string> failed_component_names;
        
        for (auto& [name, watchdog] : component_watchdogs_) {
            auto time_since_heartbeat = std::chrono::duration_cast<std::chrono::seconds>(
                now - watchdog.last_heartbeat).count();
            
            bool component_alive = (time_since_heartbeat < watchdog.timeout_seconds) && watchdog.is_alive;
            
            if (!component_alive && watchdog.is_alive) {
                // Component just failed
                RCLCPP_ERROR(this->get_logger(), "Component %s failed (timeout: %ld seconds)", 
                             name.c_str(), time_since_heartbeat);
                watchdog.is_alive = false;
                watchdog.last_message = "Timeout or unhealthy status";
            } else if (component_alive && !watchdog.is_alive) {
                // Component recovered
                RCLCPP_INFO(this->get_logger(), "Component %s recovered", name.c_str());
                watchdog.is_alive = true;
            }
            
            if (!watchdog.is_alive) {
                failed_components++;
                failed_component_names.push_back(name);
                
                if (watchdog.is_critical) {
                    critical_failed_components++;
                }
            }
        }
        
        // Calculate failure rate
        double failure_rate = static_cast<double>(failed_components) / total_components;
        
        // Check for emergency conditions
        bool emergency_condition = false;
        std::string emergency_reason;
        
        if (critical_failed_components >= critical_failure_threshold_) {
            emergency_condition = true;
            emergency_reason = "Critical component failures: " + std::to_string(critical_failed_components);
        } else if (failure_rate > total_failure_threshold_) {
            emergency_condition = true;
            emergency_reason = "Total system failure rate: " + std::to_string(static_cast<int>(failure_rate * 100)) + "%";
        }
        
        // Trigger emergency stop if conditions are met
        if (emergency_condition && enable_auto_emergency_stop_) {
            triggerEmergencyStop(emergency_reason);
        }
        
        // Publish status
        publishWatchdogStatus(failed_components, total_components, failure_rate);
        publishSystemAlive(emergency_condition);
        publishFailedComponents(failed_component_names);
    }
    
    void triggerEmergencyStop(const std::string& reason)
    {
        static auto last_emergency_call = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        
        // Rate limit emergency stop calls (max once per 5 seconds)
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_emergency_call).count() < 5) {
            return;
        }
        
        if (!emergency_stop_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Emergency stop service not available");
            return;
        }
        
        auto request = std::make_shared<tadeo_ecar_interfaces::srv::EmergencyStop::Request>();
        request->activate = true;
        
        auto future = emergency_stop_client_->async_send_request(request);
        
        last_emergency_call = now;
        
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP TRIGGERED BY WATCHDOG: %s", reason.c_str());
    }
    
    void publishWatchdogStatus(int failed_components, int total_components, double failure_rate)
    {
        auto status_msg = tadeo_ecar_msgs::msg::SystemHealth();
        status_msg.header.stamp = this->now();
        status_msg.header.frame_id = "base_link";
        
        status_msg.component_name = "watchdog";
        
        if (failure_rate > total_failure_threshold_) {
            status_msg.status = "CRITICAL";
            status_msg.error_code = 5001;
            status_msg.error_message = "System failure rate critical: " + std::to_string(static_cast<int>(failure_rate * 100)) + "%";
        } else if (failed_components > 0) {
            status_msg.status = "WARNING";
            status_msg.error_code = 5002;
            status_msg.error_message = std::to_string(failed_components) + " components failed";
        } else {
            status_msg.status = "OK";
            status_msg.error_code = 0;
            status_msg.error_message = "";
        }
        
        status_msg.cpu_usage = 5.0; // Watchdog is lightweight
        status_msg.memory_usage = 10.0;
        status_msg.temperature = 40.0;
        
        watchdog_status_pub_->publish(status_msg);
    }
    
    void publishSystemAlive(bool emergency_condition)
    {
        auto alive_msg = std_msgs::msg::Bool();
        alive_msg.data = !emergency_condition;
        system_alive_pub_->publish(alive_msg);
    }
    
    void publishFailedComponents(const std::vector<std::string>& failed_components)
    {
        auto failed_msg = std_msgs::msg::String();
        
        if (failed_components.empty()) {
            failed_msg.data = "none";
        } else {
            failed_msg.data = "";
            for (size_t i = 0; i < failed_components.size(); ++i) {
                failed_msg.data += failed_components[i];
                if (i < failed_components.size() - 1) {
                    failed_msg.data += ", ";
                }
            }
        }
        
        failed_components_pub_->publish(failed_msg);
    }
    
    void publishHeartbeat()
    {
        // Publish this node's own heartbeat
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = "base_link";
        
        health_msg.component_name = "watchdog";
        health_msg.status = "OK";
        health_msg.error_code = 0;
        health_msg.error_message = "";
        health_msg.cpu_usage = 5.0;
        health_msg.memory_usage = 10.0;
        health_msg.temperature = 40.0;
        
        watchdog_status_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    std::map<std::string, rclcpp::Subscription<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr> component_subscribers_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr generic_health_sub_;
    
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr watchdog_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr system_alive_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr failed_components_pub_;
    
    rclcpp::Client<tadeo_ecar_interfaces::srv::EmergencyStop>::SharedPtr emergency_stop_client_;
    
    rclcpp::TimerInterface::SharedPtr watchdog_timer_;
    rclcpp::TimerInterface::SharedPtr heartbeat_timer_;
    
    // Parameters
    double watchdog_frequency_;
    double heartbeat_frequency_;
    std::map<std::string, double> component_timeouts_;
    std::map<std::string, bool> critical_components_;
    bool enable_auto_emergency_stop_;
    int critical_failure_threshold_;
    double total_failure_threshold_;
    
    // State variables
    std::map<std::string, ComponentWatchdog> component_watchdogs_;
};

} // namespace tadeo_ecar_safety

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_safety::WatchdogNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}