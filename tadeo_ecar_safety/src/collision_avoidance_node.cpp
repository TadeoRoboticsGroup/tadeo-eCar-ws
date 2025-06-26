#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tadeo_ecar_msgs/msg/safety_status.hpp>
#include <tadeo_ecar_msgs/msg/robot_status.hpp>
#include "tadeo_ecar_safety/safety_types.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace tadeo_ecar_safety
{

class CollisionAvoidanceNode : public rclcpp::Node
{
public:
    CollisionAvoidanceNode() : Node("collision_avoidance_node")
    {
        loadParameters();
        initializeSafetyZones();
        
        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_raw", 10,
            std::bind(&CollisionAvoidanceNode::cmdVelCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&CollisionAvoidanceNode::scanCallback, this, std::placeholders::_1));
        
        obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "perception/fused_obstacles", 10,
            std::bind(&CollisionAvoidanceNode::obstaclesCallback, this, std::placeholders::_1));
        
        robot_status_sub_ = this->create_subscription<tadeo_ecar_msgs::msg::RobotStatus>(
            "robot_status", 10,
            std::bind(&CollisionAvoidanceNode::robotStatusCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);
        safety_status_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SafetyStatus>("safety_status", 10);
        safety_zones_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("safety_zones", 10);
        collision_prediction_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("collision_prediction", 10);
        
        // Timer for safety monitoring
        safety_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / safety_frequency_)),
            std::bind(&CollisionAvoidanceNode::safetyLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Collision Avoidance Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("safety.frequency", 20.0);
        this->declare_parameter("zones.emergency_distance", 0.3);
        this->declare_parameter("zones.warning_distance", 1.0);
        this->declare_parameter("zones.slow_distance", 2.0);
        this->declare_parameter("zones.angular_range", 1.57); // 90 degrees
        this->declare_parameter("collision.prediction_time", 3.0);
        this->declare_parameter("collision.min_confidence", 0.7);
        this->declare_parameter("avoidance.enable_lateral", true);
        this->declare_parameter("avoidance.lateral_gain", 0.5);
        this->declare_parameter("avoidance.deceleration_gain", 0.8);
        this->declare_parameter("avoidance.min_velocity", 0.1);
        this->declare_parameter("robot.width", 1.0);
        this->declare_parameter("robot.length", 1.2);
        this->declare_parameter("robot.safety_margin", 0.2);
        
        safety_frequency_ = this->get_parameter("safety.frequency").as_double();
        emergency_distance_ = this->get_parameter("zones.emergency_distance").as_double();
        warning_distance_ = this->get_parameter("zones.warning_distance").as_double();
        slow_distance_ = this->get_parameter("zones.slow_distance").as_double();
        angular_range_ = this->get_parameter("zones.angular_range").as_double();
        prediction_time_ = this->get_parameter("collision.prediction_time").as_double();
        min_confidence_ = this->get_parameter("collision.min_confidence").as_double();
        enable_lateral_avoidance_ = this->get_parameter("avoidance.enable_lateral").as_bool();
        lateral_gain_ = this->get_parameter("avoidance.lateral_gain").as_double();
        deceleration_gain_ = this->get_parameter("avoidance.deceleration_gain").as_double();
        min_velocity_ = this->get_parameter("avoidance.min_velocity").as_double();
        robot_width_ = this->get_parameter("robot.width").as_double();
        robot_length_ = this->get_parameter("robot.length").as_double();
        safety_margin_ = this->get_parameter("robot.safety_margin").as_double();
    }
    
    void initializeSafetyZones()
    {
        safety_zones_.clear();
        
        // Emergency zone (immediate stop)
        SafetyZone emergency_zone;
        emergency_zone.name = "emergency";
        emergency_zone.min_distance = 0.0;
        emergency_zone.max_distance = emergency_distance_;
        emergency_zone.angular_range = angular_range_;
        emergency_zone.level = SafetyLevel::EMERGENCY;
        emergency_zone.is_active = true;
        safety_zones_.push_back(emergency_zone);
        
        // Warning zone (significant deceleration)
        SafetyZone warning_zone;
        warning_zone.name = "warning";
        warning_zone.min_distance = emergency_distance_;
        warning_zone.max_distance = warning_distance_;
        warning_zone.angular_range = angular_range_;
        warning_zone.level = SafetyLevel::CRITICAL;
        warning_zone.is_active = true;
        safety_zones_.push_back(warning_zone);
        
        // Slow zone (moderate deceleration)
        SafetyZone slow_zone;
        slow_zone.name = "slow";
        slow_zone.min_distance = warning_distance_;
        slow_zone.max_distance = slow_distance_;
        slow_zone.angular_range = angular_range_;
        slow_zone.level = SafetyLevel::WARNING;
        slow_zone.is_active = true;
        safety_zones_.push_back(slow_zone);
        
        RCLCPP_INFO(this->get_logger(), "Safety zones initialized: Emergency=%.2fm, Warning=%.2fm, Slow=%.2fm",
                    emergency_distance_, warning_distance_, slow_distance_);
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_cmd_vel_ = *msg;
        
        // Process command through collision avoidance
        processCollisionAvoidance();
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        current_scan_ = *msg;
        
        // Analyze scan for immediate obstacles
        analyzeLaserScan();
    }
    
    void obstaclesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        current_obstacles_.clear();
        
        for (const auto& marker : msg->markers) {
            if (marker.action == visualization_msgs::msg::Marker::ADD) {
                current_obstacles_.push_back(marker);
            }
        }
        
        // Update collision predictions
        updateCollisionPredictions();
    }
    
    void robotStatusCallback(const tadeo_ecar_msgs::msg::RobotStatus::SharedPtr msg)
    {
        current_robot_status_ = *msg;
        current_velocity_ = msg->current_speed;
    }
    
    void processCollisionAvoidance()
    {
        auto safe_cmd = target_cmd_vel_;
        
        // Check each safety zone
        SafetyLevel highest_threat = SafetyLevel::SAFE;
        double closest_obstacle_distance = std::numeric_limits<double>::max();
        
        for (const auto& zone : safety_zones_) {
            if (!zone.is_active) continue;
            
            bool obstacle_in_zone = checkObstacleInZone(zone, closest_obstacle_distance);
            
            if (obstacle_in_zone) {
                highest_threat = std::max(highest_threat, zone.level);
            }
        }
        
        // Apply safety modifications based on threat level
        switch (highest_threat) {
            case SafetyLevel::EMERGENCY:
                safe_cmd = applyEmergencyStop(safe_cmd);
                break;
            case SafetyLevel::CRITICAL:
                safe_cmd = applyHardDeceleration(safe_cmd, closest_obstacle_distance);
                break;
            case SafetyLevel::WARNING:
                safe_cmd = applyModerateDeceleration(safe_cmd, closest_obstacle_distance);
                break;
            case SafetyLevel::SAFE:
            default:
                // No modification needed
                break;
        }
        
        // Apply additional collision avoidance
        if (enable_lateral_avoidance_) {
            safe_cmd = applyLateralAvoidance(safe_cmd);
        }
        
        // Validate final command
        safe_cmd = validateCommand(safe_cmd);
        
        // Publish safe command
        cmd_vel_pub_->publish(safe_cmd);
        
        // Update safety status
        updateSafetyStatus(highest_threat, closest_obstacle_distance);
    }
    
    bool checkObstacleInZone(const SafetyZone& zone, double& closest_distance)
    {
        bool obstacle_found = false;
        double min_distance = std::numeric_limits<double>::max();
        
        // Check laser scan data
        if (!current_scan_.ranges.empty()) {
            for (size_t i = 0; i < current_scan_.ranges.size(); ++i) {
                double angle = current_scan_.angle_min + i * current_scan_.angle_increment;
                double range = current_scan_.ranges[i];
                
                // Skip invalid readings
                if (std::isnan(range) || std::isinf(range) || range <= 0) continue;
                
                // Check if point is within zone angular range
                if (std::abs(angle) <= zone.angular_range / 2.0) {
                    // Check if point is within zone distance range
                    if (range >= zone.min_distance && range <= zone.max_distance) {
                        obstacle_found = true;
                        min_distance = std::min(min_distance, range);
                    }
                }
            }
        }
        
        // Check tracked obstacles
        for (const auto& obstacle : current_obstacles_) {
            double distance = sqrt(obstacle.pose.position.x * obstacle.pose.position.x + 
                                   obstacle.pose.position.y * obstacle.pose.position.y);
            double angle = atan2(obstacle.pose.position.y, obstacle.pose.position.x);
            
            // Check if obstacle is within zone
            if (std::abs(angle) <= zone.angular_range / 2.0 && 
                distance >= zone.min_distance && distance <= zone.max_distance) {
                obstacle_found = true;
                min_distance = std::min(min_distance, distance);
            }
        }
        
        closest_distance = std::min(closest_distance, min_distance);
        return obstacle_found;
    }
    
    geometry_msgs::msg::Twist applyEmergencyStop(const geometry_msgs::msg::Twist& cmd)
    {
        auto stop_cmd = geometry_msgs::msg::Twist();
        // All velocities set to zero
        return stop_cmd;
    }
    
    geometry_msgs::msg::Twist applyHardDeceleration(const geometry_msgs::msg::Twist& cmd, double obstacle_distance)
    {
        auto safe_cmd = cmd;
        
        // Calculate deceleration factor based on distance
        double deceleration_factor = std::max(0.1, (obstacle_distance - emergency_distance_) / 
                                              (warning_distance_ - emergency_distance_));
        
        safe_cmd.linear.x *= deceleration_factor * deceleration_gain_;
        safe_cmd.linear.y *= deceleration_factor * deceleration_gain_;
        safe_cmd.angular.z *= deceleration_factor;
        
        return safe_cmd;
    }
    
    geometry_msgs::msg::Twist applyModerateDeceleration(const geometry_msgs::msg::Twist& cmd, double obstacle_distance)
    {
        auto safe_cmd = cmd;
        
        // Calculate deceleration factor based on distance
        double deceleration_factor = std::max(0.3, (obstacle_distance - warning_distance_) / 
                                              (slow_distance_ - warning_distance_));
        
        safe_cmd.linear.x *= deceleration_factor;
        safe_cmd.linear.y *= deceleration_factor;
        safe_cmd.angular.z *= deceleration_factor;
        
        return safe_cmd;
    }
    
    geometry_msgs::msg::Twist applyLateralAvoidance(const geometry_msgs::msg::Twist& cmd)
    {
        auto safe_cmd = cmd;
        
        // Find closest obstacle and calculate avoidance vector
        double closest_distance = std::numeric_limits<double>::max();
        double avoidance_angle = 0.0;
        
        for (const auto& obstacle : current_obstacles_) {
            double distance = sqrt(obstacle.pose.position.x * obstacle.pose.position.x + 
                                   obstacle.pose.position.y * obstacle.pose.position.y);
            
            if (distance < closest_distance && distance < slow_distance_) {
                closest_distance = distance;
                avoidance_angle = atan2(obstacle.pose.position.y, obstacle.pose.position.x);
            }
        }
        
        // Apply lateral avoidance if obstacle is close
        if (closest_distance < slow_distance_) {
            double avoidance_strength = lateral_gain_ * (slow_distance_ - closest_distance) / slow_distance_;
            
            // Add lateral component to avoid obstacle
            if (avoidance_angle > 0) {
                safe_cmd.linear.y -= avoidance_strength; // Move right to avoid left obstacle
            } else {
                safe_cmd.linear.y += avoidance_strength; // Move left to avoid right obstacle
            }
        }
        
        return safe_cmd;
    }
    
    geometry_msgs::msg::Twist validateCommand(const geometry_msgs::msg::Twist& cmd)
    {
        auto validated_cmd = cmd;
        
        // Ensure minimum velocity for stability
        if (std::abs(validated_cmd.linear.x) > 0 && std::abs(validated_cmd.linear.x) < min_velocity_) {
            validated_cmd.linear.x = (validated_cmd.linear.x > 0) ? min_velocity_ : -min_velocity_;
        }
        
        // Limit maximum velocities (should be handled by control layer, but double-check)
        validated_cmd.linear.x = std::clamp(validated_cmd.linear.x, -2.0, 2.0);
        validated_cmd.linear.y = std::clamp(validated_cmd.linear.y, -1.0, 1.0);
        validated_cmd.angular.z = std::clamp(validated_cmd.angular.z, -1.5, 1.5);
        
        return validated_cmd;
    }
    
    void analyzeLaserScan()
    {
        if (current_scan_.ranges.empty()) return;
        
        // Find minimum distance in forward sector
        double min_forward_distance = std::numeric_limits<double>::max();
        size_t forward_start = current_scan_.ranges.size() / 4;
        size_t forward_end = 3 * current_scan_.ranges.size() / 4;
        
        for (size_t i = forward_start; i < forward_end; ++i) {
            if (!std::isnan(current_scan_.ranges[i]) && !std::isinf(current_scan_.ranges[i])) {
                min_forward_distance = std::min(min_forward_distance, static_cast<double>(current_scan_.ranges[i]));
            }
        }
        
        scan_min_distance_ = min_forward_distance;
    }
    
    void updateCollisionPredictions()
    {
        current_predictions_.clear();
        
        for (const auto& obstacle : current_obstacles_) {
            CollisionPrediction prediction = predictCollision(obstacle);
            
            if (prediction.collision_predicted && prediction.confidence > min_confidence_) {
                current_predictions_.push_back(prediction);
                
                // Publish collision prediction
                auto point_msg = geometry_msgs::msg::PointStamped();
                point_msg.header.stamp = this->now();
                point_msg.header.frame_id = "base_link";
                point_msg.point = prediction.collision_point;
                collision_prediction_pub_->publish(point_msg);
            }
        }
    }
    
    CollisionPrediction predictCollision(const visualization_msgs::msg::Marker& obstacle)
    {
        CollisionPrediction prediction;
        prediction.collision_predicted = false;
        prediction.confidence = 0.0;
        
        // Simple collision prediction based on current trajectory
        double obstacle_distance = sqrt(obstacle.pose.position.x * obstacle.pose.position.x + 
                                        obstacle.pose.position.y * obstacle.pose.position.y);
        
        if (current_velocity_ > 0.1) {
            double time_to_obstacle = obstacle_distance / current_velocity_;
            
            if (time_to_obstacle < prediction_time_) {
                prediction.collision_predicted = true;
                prediction.time_to_collision = time_to_obstacle;
                prediction.collision_distance = obstacle_distance;
                prediction.collision_point = obstacle.pose.position;
                prediction.confidence = 1.0 - (time_to_obstacle / prediction_time_);
                prediction.obstacle_type = "detected_obstacle";
            }
        }
        
        return prediction;
    }
    
    void updateSafetyStatus(SafetyLevel threat_level, double closest_distance)
    {
        auto safety_msg = tadeo_ecar_msgs::msg::SafetyStatus();
        safety_msg.header.stamp = this->now();
        safety_msg.header.frame_id = "base_link";
        
        safety_msg.emergency_stop = (threat_level == SafetyLevel::EMERGENCY);
        safety_msg.collision_warning = (threat_level >= SafetyLevel::CRITICAL);
        safety_msg.stability_warning = false; // Handled by other nodes
        safety_msg.safety_override = false;
        
        safety_msg.closest_obstacle_distance = closest_distance;
        safety_msg.safety_zone_status = static_cast<int>(threat_level);
        
        // Set individual zone statuses
        safety_msg.emergency_zone_clear = !checkObstacleInZone(safety_zones_[0], closest_distance);
        safety_msg.warning_zone_clear = !checkObstacleInZone(safety_zones_[1], closest_distance);
        safety_msg.slow_zone_clear = !checkObstacleInZone(safety_zones_[2], closest_distance);
        
        safety_status_pub_->publish(safety_msg);
    }
    
    void safetyLoop()
    {
        publishSafetyZones();
    }
    
    void publishSafetyZones()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        for (size_t i = 0; i < safety_zones_.size(); ++i) {
            const auto& zone = safety_zones_[i];
            
            auto marker = visualization_msgs::msg::Marker();
            marker.header.stamp = this->now();
            marker.header.frame_id = "base_link";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = zone.max_distance / 2.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = zone.max_distance * 2.0;
            marker.scale.y = zone.max_distance * 2.0;
            marker.scale.z = 0.1;
            
            // Color based on zone level
            switch (zone.level) {
                case SafetyLevel::EMERGENCY:
                    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
                    break;
                case SafetyLevel::CRITICAL:
                    marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0;
                    break;
                case SafetyLevel::WARNING:
                    marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
                    break;
                default:
                    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
                    break;
            }
            marker.color.a = 0.3;
            
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            marker_array.markers.push_back(marker);
        }
        
        safety_zones_pub_->publish(marker_array);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SafetyStatus>::SharedPtr safety_status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr safety_zones_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr collision_prediction_pub_;
    
    rclcpp::TimerBase::SharedPtr safety_timer_;
    
    // Parameters
    double safety_frequency_;
    double emergency_distance_;
    double warning_distance_;
    double slow_distance_;
    double angular_range_;
    double prediction_time_;
    double min_confidence_;
    bool enable_lateral_avoidance_;
    double lateral_gain_;
    double deceleration_gain_;
    double min_velocity_;
    double robot_width_;
    double robot_length_;
    double safety_margin_;
    
    // State variables
    std::vector<SafetyZone> safety_zones_;
    geometry_msgs::msg::Twist target_cmd_vel_;
    sensor_msgs::msg::LaserScan current_scan_;
    std::vector<visualization_msgs::msg::Marker> current_obstacles_;
    std::vector<CollisionPrediction> current_predictions_;
    tadeo_ecar_msgs::msg::RobotStatus current_robot_status_;
    double current_velocity_;
    double scan_min_distance_;
};

} // namespace tadeo_ecar_safety

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_safety::CollisionAvoidanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}