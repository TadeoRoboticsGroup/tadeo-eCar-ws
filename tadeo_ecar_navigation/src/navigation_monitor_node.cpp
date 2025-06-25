#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include "tadeo_ecar_navigation/navigation_types.hpp"
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <limits>

namespace tadeo_ecar_navigation
{

class NavigationMonitorNode : public rclcpp::Node
{
public:
    NavigationMonitorNode() : Node("navigation_monitor_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeMonitor();
        
        // Subscribers
        navigation_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "navigation_status", 10,
            std::bind(&NavigationMonitorNode::navigationStatusCallback, this, std::placeholders::_1));
        
        mission_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "mission_status", 10,
            std::bind(&NavigationMonitorNode::missionStatusCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&NavigationMonitorNode::odomCallback, this, std::placeholders::_1));
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&NavigationMonitorNode::cmdVelCallback, this, std::placeholders::_1));
        
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&NavigationMonitorNode::laserCallback, this, std::placeholders::_1));
        
        costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
            "global_costmap/costmap", 10,
            std::bind(&NavigationMonitorNode::costmapCallback, this, std::placeholders::_1));
        
        current_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "current_goal", 10,
            std::bind(&NavigationMonitorNode::currentGoalCallback, this, std::placeholders::_1));
        
        // Publishers
        performance_metrics_pub_ = this->create_publisher<std_msgs::msg::String>("navigation_metrics", 10);
        safety_alerts_pub_ = this->create_publisher<std_msgs::msg::String>("safety_alerts", 10);
        navigation_quality_pub_ = this->create_publisher<std_msgs::msg::Float64>("navigation_quality", 10);
        monitor_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("monitor_visualization", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("navigation/monitor_health", 10);
        
        // Timers
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / monitor_frequency_)),
            std::bind(&NavigationMonitorNode::monitorLoop, this));
        
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&NavigationMonitorNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Navigation Monitor Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("monitor_frequency", 10.0);
        this->declare_parameter("safety_distance_threshold", 0.5);
        this->declare_parameter("velocity_threshold", 0.1);
        this->declare_parameter("angular_velocity_threshold", 0.05);
        this->declare_parameter("goal_distance_threshold", 5.0);
        this->declare_parameter("costmap_analysis_enabled", true);
        this->declare_parameter("performance_logging_enabled", true);
        this->declare_parameter("safety_monitoring_enabled", true);
        this->declare_parameter("data_logging_path", "/tmp/tadeo_navigation_logs/");
        this->declare_parameter("alert_threshold_high", 0.8);
        this->declare_parameter("alert_threshold_medium", 0.6);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
        
        monitor_frequency_ = this->get_parameter("monitor_frequency").as_double();
        safety_distance_threshold_ = this->get_parameter("safety_distance_threshold").as_double();
        velocity_threshold_ = this->get_parameter("velocity_threshold").as_double();
        angular_velocity_threshold_ = this->get_parameter("angular_velocity_threshold").as_double();
        goal_distance_threshold_ = this->get_parameter("goal_distance_threshold").as_double();
        costmap_analysis_enabled_ = this->get_parameter("costmap_analysis_enabled").as_bool();
        performance_logging_enabled_ = this->get_parameter("performance_logging_enabled").as_bool();
        safety_monitoring_enabled_ = this->get_parameter("safety_monitoring_enabled").as_bool();
        data_logging_path_ = this->get_parameter("data_logging_path").as_string();
        alert_threshold_high_ = this->get_parameter("alert_threshold_high").as_double();
        alert_threshold_medium_ = this->get_parameter("alert_threshold_medium").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
    }
    
    void initializeMonitor()
    {
        // Initialize monitoring state
        current_navigation_state_ = NavigationState::IDLE;
        current_mission_state_ = "IDLE";
        
        // Initialize metrics
        navigation_quality_score_ = 1.0;
        obstacle_proximity_score_ = 1.0;
        path_deviation_score_ = 1.0;
        velocity_smoothness_score_ = 1.0;
        
        // Initialize flags
        goal_available_ = false;
        odom_available_ = false;
        laser_available_ = false;
        costmap_available_ = false;
        
        // Initialize timing
        last_odom_time_ = this->now();
        last_laser_time_ = this->now();
        last_cmd_vel_time_ = this->now();
        
        // Create logging directory
        try {
            std::filesystem::create_directories(data_logging_path_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create logging directory: %s", e.what());
        }
        
        // Initialize velocity history for smoothness analysis
        velocity_history_.reserve(velocity_history_size_);
        angular_velocity_history_.reserve(velocity_history_size_);
        
        RCLCPP_INFO(this->get_logger(), "Navigation monitor initialized");
    }
    
    void navigationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string status = msg->data;
        
        // Parse navigation status (state|mode)
        size_t separator = status.find('|');
        if (separator != std::string::npos) {
            std::string state_str = status.substr(0, separator);
            std::string mode_str = status.substr(separator + 1);
            
            // Update navigation state
            if (state_str == "IDLE") {
                current_navigation_state_ = NavigationState::IDLE;
            } else if (state_str == "NAVIGATING") {
                current_navigation_state_ = NavigationState::NAVIGATING;
            } else if (state_str == "WAYPOINT_REACHED") {
                current_navigation_state_ = NavigationState::WAYPOINT_REACHED;
                navigation_metrics_.waypoints_reached++;
            } else if (state_str == "MISSION_COMPLETED") {
                current_navigation_state_ = NavigationState::MISSION_COMPLETED;
                navigation_metrics_.missions_completed++;
            } else if (state_str == "MISSION_FAILED") {
                current_navigation_state_ = NavigationState::MISSION_FAILED;
                navigation_metrics_.navigation_failures++;
            } else if (state_str == "EMERGENCY_STOP") {
                current_navigation_state_ = NavigationState::EMERGENCY_STOP;
                handleEmergencyAlert();
            } else if (state_str == "RECOVERY") {
                current_navigation_state_ = NavigationState::RECOVERY;
                recovery_events_++;
            }
            
            // Update mode
            current_navigation_mode_ = mode_str;
        }
        
        last_navigation_status_time_ = this->now();
    }
    
    void missionStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_mission_state_ = msg->data;
        last_mission_status_time_ = this->now();
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        current_pose_.theta = 2.0 * atan2(qz, qw);
        current_pose_.timestamp = msg->header.stamp;
        
        current_velocity_ = msg->twist.twist;
        
        odom_available_ = true;
        last_odom_time_ = msg->header.stamp;
        
        // Update navigation metrics
        updateNavigationMetrics();
        
        // Analyze path deviation if goal is available
        if (goal_available_) {
            analyzePathDeviation();
        }
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_cmd_vel_ = *msg;
        last_cmd_vel_time_ = this->now();
        
        // Update velocity history for smoothness analysis
        updateVelocityHistory(msg->linear.x, msg->angular.z);
        
        // Analyze velocity smoothness
        analyzeVelocitySmoothness();
    }
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        current_laser_scan_ = *msg;
        laser_available_ = true;
        last_laser_time_ = msg->header.stamp;
        
        // Analyze obstacle proximity
        if (safety_monitoring_enabled_) {
            analyzeObstacleProximity();
        }
    }
    
    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        current_costmap_ = *msg;
        costmap_available_ = true;
        last_costmap_time_ = msg->header.stamp;
        
        // Analyze costmap data
        if (costmap_analysis_enabled_) {
            analyzeCostmap();
        }
    }
    
    void currentGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_goal_.x = msg->pose.position.x;
        current_goal_.y = msg->pose.position.y;
        
        double qw = msg->pose.orientation.w;
        double qz = msg->pose.orientation.z;
        current_goal_.theta = 2.0 * atan2(qz, qw);
        current_goal_.timestamp = msg->header.stamp;
        
        goal_available_ = true;
        last_goal_time_ = msg->header.stamp;
    }
    
    void monitorLoop()
    {
        // Check data freshness
        checkDataFreshness();
        
        // Calculate overall navigation quality
        calculateNavigationQuality();
        
        // Perform safety monitoring
        if (safety_monitoring_enabled_) {
            performSafetyMonitoring();
        }
        
        // Update performance metrics
        updatePerformanceMetrics();
        
        // Publish monitoring data
        publishPerformanceMetrics();
        publishNavigationQuality();
        publishMonitorVisualization();
        
        // Log data if enabled
        if (performance_logging_enabled_) {
            logNavigationData();
        }
    }
    
    void checkDataFreshness()
    {
        auto current_time = this->now();
        double timeout_threshold = 2.0; // seconds
        
        data_freshness_.odom_fresh = (current_time - last_odom_time_).seconds() < timeout_threshold;
        data_freshness_.laser_fresh = (current_time - last_laser_time_).seconds() < timeout_threshold;
        data_freshness_.cmd_vel_fresh = (current_time - last_cmd_vel_time_).seconds() < timeout_threshold;
        data_freshness_.costmap_fresh = (current_time - last_costmap_time_).seconds() < timeout_threshold;
        data_freshness_.goal_fresh = goal_available_ && (current_time - last_goal_time_).seconds() < timeout_threshold;
        
        // Alert for stale data
        if (!data_freshness_.odom_fresh) {
            publishSafetyAlert("WARNING", "Odometry data is stale");
        }
        if (!data_freshness_.laser_fresh && laser_available_) {
            publishSafetyAlert("WARNING", "Laser scan data is stale");
        }
    }
    
    void updateNavigationMetrics()
    {
        if (!odom_available_) return;
        
        auto current_time = this->now();
        static auto last_update_time = current_time;
        static Pose2D last_pose = current_pose_;
        
        double dt = (current_time - last_update_time).seconds();
        if (dt < 0.1) return; // Update at most every 100ms
        
        // Update distance traveled
        double distance_increment = current_pose_.distance(last_pose);
        navigation_metrics_.total_distance_traveled += distance_increment;
        
        // Update time elapsed
        navigation_metrics_.total_time_elapsed += dt;
        
        // Calculate average velocity
        if (navigation_metrics_.total_time_elapsed > 0) {
            navigation_metrics_.average_velocity = navigation_metrics_.total_distance_traveled / 
                                                  navigation_metrics_.total_time_elapsed;
        }
        
        // Update success rate
        navigation_metrics_.updateSuccessRate();
        
        last_update_time = current_time;
        last_pose = current_pose_;
    }
    
    void analyzePathDeviation()
    {
        if (!goal_available_ || !odom_available_) return;
        
        // Calculate distance to goal
        double distance_to_goal = current_pose_.distance(current_goal_);
        
        // Simple path deviation analysis (assumes straight line to goal)
        // In a real implementation, this would use the planned path
        double max_acceptable_deviation = 2.0; // meters
        
        if (distance_to_goal > goal_distance_threshold_) {
            path_deviation_score_ = std::max(0.0, 1.0 - (distance_to_goal / max_acceptable_deviation));
        } else {
            path_deviation_score_ = 1.0;
        }
        
        path_deviation_score_ = std::clamp(path_deviation_score_, 0.0, 1.0);
    }
    
    void updateVelocityHistory(double linear_vel, double angular_vel)
    {
        // Add new velocity data
        velocity_history_.push_back(linear_vel);
        angular_velocity_history_.push_back(angular_vel);
        
        // Keep history size limited
        if (velocity_history_.size() > velocity_history_size_) {
            velocity_history_.erase(velocity_history_.begin());
        }
        if (angular_velocity_history_.size() > velocity_history_size_) {
            angular_velocity_history_.erase(angular_velocity_history_.begin());
        }
    }
    
    void analyzeVelocitySmoothness()
    {
        if (velocity_history_.size() < 5) return; // Need some data
        
        // Calculate velocity smoothness based on acceleration changes
        double linear_smoothness = 0.0;
        double angular_smoothness = 0.0;
        
        for (size_t i = 2; i < velocity_history_.size(); ++i) {
            // Calculate acceleration (change in velocity)
            double linear_accel = velocity_history_[i] - velocity_history_[i-1];
            double angular_accel = angular_velocity_history_[i] - angular_velocity_history_[i-1];
            
            // Calculate jerk (change in acceleration)
            double prev_linear_accel = velocity_history_[i-1] - velocity_history_[i-2];
            double prev_angular_accel = angular_velocity_history_[i-1] - angular_velocity_history_[i-2];
            
            double linear_jerk = std::abs(linear_accel - prev_linear_accel);
            double angular_jerk = std::abs(angular_accel - prev_angular_accel);
            
            linear_smoothness += linear_jerk;
            angular_smoothness += angular_jerk;
        }
        
        // Normalize smoothness score (lower jerk = higher smoothness)
        double max_acceptable_jerk = 2.0;
        linear_smoothness /= (velocity_history_.size() - 2);
        angular_smoothness /= (velocity_history_.size() - 2);
        
        double combined_smoothness = (linear_smoothness + angular_smoothness) / 2.0;
        velocity_smoothness_score_ = std::max(0.0, 1.0 - (combined_smoothness / max_acceptable_jerk));
        velocity_smoothness_score_ = std::clamp(velocity_smoothness_score_, 0.0, 1.0);
    }
    
    void analyzeObstacleProximity()
    {
        if (!laser_available_) return;
        
        double min_distance = std::numeric_limits<double>::max();
        
        // Find minimum distance to obstacles in front sector
        int front_sector_start = current_laser_scan_.ranges.size() * 0.25; // 90 degrees left
        int front_sector_end = current_laser_scan_.ranges.size() * 0.75;   // 90 degrees right
        
        for (int i = front_sector_start; i < front_sector_end; ++i) {
            double range = current_laser_scan_.ranges[i];
            if (range > current_laser_scan_.range_min && range < current_laser_scan_.range_max) {
                min_distance = std::min(min_distance, range);
            }
        }
        
        // Calculate proximity score
        if (min_distance < safety_distance_threshold_) {
            obstacle_proximity_score_ = min_distance / safety_distance_threshold_;
            
            // Publish safety alert for close obstacles
            if (min_distance < safety_distance_threshold_ * 0.5) {
                publishSafetyAlert("HIGH", "Obstacle very close: " + std::to_string(min_distance) + "m");
            } else {
                publishSafetyAlert("MEDIUM", "Obstacle detected: " + std::to_string(min_distance) + "m");
            }
        } else {
            obstacle_proximity_score_ = 1.0;
        }
        
        obstacle_proximity_score_ = std::clamp(obstacle_proximity_score_, 0.0, 1.0);
    }
    
    void analyzeCostmap()
    {
        // Simplified costmap analysis
        // In a real implementation, this would analyze the costmap around the robot
        if (!costmap_available_) return;
        
        // For now, just check if costmap data is available
        costmap_quality_score_ = costmap_available_ ? 1.0 : 0.0;
    }
    
    void calculateNavigationQuality()
    {
        // Weighted combination of different quality metrics
        double obstacle_weight = 0.3;
        double path_weight = 0.3;
        double smoothness_weight = 0.2;
        double data_freshness_weight = 0.2;
        
        // Calculate data freshness score
        double freshness_score = 0.0;
        int fresh_count = 0;
        
        if (data_freshness_.odom_fresh) { freshness_score += 1.0; fresh_count++; }
        if (data_freshness_.laser_fresh || !laser_available_) { freshness_score += 1.0; fresh_count++; }
        if (data_freshness_.cmd_vel_fresh) { freshness_score += 1.0; fresh_count++; }
        if (data_freshness_.costmap_fresh || !costmap_available_) { freshness_score += 1.0; fresh_count++; }
        
        if (fresh_count > 0) {
            freshness_score /= fresh_count;
        }
        
        // Calculate overall quality score
        navigation_quality_score_ = (obstacle_proximity_score_ * obstacle_weight +
                                   path_deviation_score_ * path_weight +
                                   velocity_smoothness_score_ * smoothness_weight +
                                   freshness_score * data_freshness_weight);
        
        navigation_quality_score_ = std::clamp(navigation_quality_score_, 0.0, 1.0);
    }
    
    void performSafetyMonitoring()
    {
        // Check for emergency conditions
        if (current_navigation_state_ == NavigationState::EMERGENCY_STOP) {
            return; // Already in emergency state
        }
        
        // Check velocity limits
        double linear_speed = std::abs(current_velocity_.linear.x);
        double angular_speed = std::abs(current_velocity_.angular.z);
        
        if (linear_speed > velocity_threshold_ * 10.0) { // Excessive speed
            publishSafetyAlert("HIGH", "Excessive linear velocity: " + std::to_string(linear_speed));
        }
        
        if (angular_speed > angular_velocity_threshold_ * 20.0) { // Excessive angular speed
            publishSafetyAlert("HIGH", "Excessive angular velocity: " + std::to_string(angular_speed));
        }
        
        // Check for stuck condition
        static auto last_movement_time = this->now();
        if (linear_speed < velocity_threshold_ && angular_speed < angular_velocity_threshold_) {
            if (current_navigation_state_ == NavigationState::NAVIGATING) {
                double stuck_duration = (this->now() - last_movement_time).seconds();
                if (stuck_duration > 10.0) { // Stuck for 10 seconds
                    publishSafetyAlert("MEDIUM", "Robot appears to be stuck");
                }
            }
        } else {
            last_movement_time = this->now();
        }
    }
    
    void updatePerformanceMetrics()
    {
        performance_metrics_.navigation_quality = navigation_quality_score_;
        performance_metrics_.obstacle_proximity = obstacle_proximity_score_;
        performance_metrics_.path_deviation = path_deviation_score_;
        performance_metrics_.velocity_smoothness = velocity_smoothness_score_;
        performance_metrics_.data_freshness = data_freshness_.odom_fresh && 
                                             data_freshness_.cmd_vel_fresh;
        performance_metrics_.recovery_events = recovery_events_;
        performance_metrics_.navigation_metrics = navigation_metrics_;
    }
    
    void handleEmergencyAlert()
    {
        emergency_events_++;
        publishSafetyAlert("CRITICAL", "Emergency stop activated");
        
        // Log emergency event
        if (performance_logging_enabled_) {
            logEmergencyEvent();
        }
    }
    
    void publishSafetyAlert(const std::string& level, const std::string& message)
    {
        std_msgs::msg::String alert_msg;
        alert_msg.data = level + ": " + message;
        safety_alerts_pub_->publish(alert_msg);
        
        // Log based on severity
        if (level == "CRITICAL") {
            RCLCPP_ERROR(this->get_logger(), "SAFETY ALERT - %s: %s", level.c_str(), message.c_str());
        } else if (level == "HIGH") {
            RCLCPP_WARN(this->get_logger(), "SAFETY ALERT - %s: %s", level.c_str(), message.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "SAFETY ALERT - %s: %s", level.c_str(), message.c_str());
        }
    }
    
    void publishPerformanceMetrics()
    {
        std_msgs::msg::String metrics_msg;
        
        std::ostringstream oss;
        oss << "Quality: " << std::fixed << std::setprecision(2) << navigation_quality_score_
            << " | Obstacles: " << obstacle_proximity_score_
            << " | Path: " << path_deviation_score_
            << " | Smoothness: " << velocity_smoothness_score_
            << " | Waypoints: " << navigation_metrics_.waypoints_reached
            << " | Missions: " << navigation_metrics_.missions_completed
            << " | Failures: " << navigation_metrics_.navigation_failures
            << " | Success Rate: " << navigation_metrics_.success_rate << "%"
            << " | Recovery: " << recovery_events_
            << " | Emergency: " << emergency_events_;
        
        metrics_msg.data = oss.str();
        performance_metrics_pub_->publish(metrics_msg);
    }
    
    void publishNavigationQuality()
    {
        std_msgs::msg::Float64 quality_msg;
        quality_msg.data = navigation_quality_score_;
        navigation_quality_pub_->publish(quality_msg);
    }
    
    void publishMonitorVisualization()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Quality indicator marker
        visualization_msgs::msg::Marker quality_marker;
        quality_marker.header.frame_id = base_frame_;
        quality_marker.header.stamp = this->now();
        quality_marker.ns = "navigation_monitor";
        quality_marker.id = 0;
        quality_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        quality_marker.action = visualization_msgs::msg::Marker::ADD;
        
        quality_marker.pose.position.x = 0.0;
        quality_marker.pose.position.y = 0.0;
        quality_marker.pose.position.z = 2.0;
        quality_marker.pose.orientation.w = 1.0;
        
        quality_marker.text = "Quality: " + std::to_string(static_cast<int>(navigation_quality_score_ * 100)) + "%";
        quality_marker.scale.z = 0.5;
        
        // Color based on quality
        if (navigation_quality_score_ > alert_threshold_high_) {
            quality_marker.color.r = 0.0; quality_marker.color.g = 1.0; quality_marker.color.b = 0.0;
        } else if (navigation_quality_score_ > alert_threshold_medium_) {
            quality_marker.color.r = 1.0; quality_marker.color.g = 1.0; quality_marker.color.b = 0.0;
        } else {
            quality_marker.color.r = 1.0; quality_marker.color.g = 0.0; quality_marker.color.b = 0.0;
        }
        quality_marker.color.a = 1.0;
        
        marker_array.markers.push_back(quality_marker);
        
        monitor_viz_pub_->publish(marker_array);
    }
    
    void logNavigationData()
    {
        static auto last_log_time = this->now();
        auto current_time = this->now();
        
        // Log every 5 seconds
        if ((current_time - last_log_time).seconds() < 5.0) return;
        
        try {
            std::string log_filename = data_logging_path_ + "navigation_log_" +
                                      std::to_string(std::time(nullptr)) + ".csv";
            
            std::ofstream log_file(log_filename, std::ios::app);
            
            // Write header if file is new
            if (log_file.tellp() == 0) {
                log_file << "timestamp,quality_score,obstacle_score,path_score,smoothness_score,"
                         << "waypoints_reached,missions_completed,failures,success_rate,"
                         << "recovery_events,emergency_events,navigation_state,mission_state\n";
            }
            
            log_file << current_time.seconds() << ","
                     << navigation_quality_score_ << ","
                     << obstacle_proximity_score_ << ","
                     << path_deviation_score_ << ","
                     << velocity_smoothness_score_ << ","
                     << navigation_metrics_.waypoints_reached << ","
                     << navigation_metrics_.missions_completed << ","
                     << navigation_metrics_.navigation_failures << ","
                     << navigation_metrics_.success_rate << ","
                     << recovery_events_ << ","
                     << emergency_events_ << ","
                     << stateToString(current_navigation_state_) << ","
                     << current_mission_state_ << "\n";
            
            log_file.close();
            last_log_time = current_time;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to log navigation data: %s", e.what());
        }
    }
    
    void logEmergencyEvent()
    {
        try {
            std::string emergency_log = data_logging_path_ + "emergency_log.txt";
            std::ofstream log_file(emergency_log, std::ios::app);
            
            auto current_time = this->now();
            log_file << "EMERGENCY EVENT - " << current_time.seconds() 
                     << " | State: " << stateToString(current_navigation_state_)
                     << " | Mission: " << current_mission_state_
                     << " | Quality: " << navigation_quality_score_ << "\n";
            
            log_file.close();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to log emergency event: %s", e.what());
        }
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        health_msg.component_name = "navigation_monitor";
        
        // Determine health status based on quality score
        if (navigation_quality_score_ > alert_threshold_high_) {
            health_msg.status = "OK";
            health_msg.error_code = 0;
            health_msg.error_message = "";
        } else if (navigation_quality_score_ > alert_threshold_medium_) {
            health_msg.status = "WARNING";
            health_msg.error_code = 18001;
            health_msg.error_message = "Navigation quality degraded";
        } else {
            health_msg.status = "ERROR";
            health_msg.error_code = 18002;
            health_msg.error_message = "Poor navigation quality";
        }
        
        // Check for data freshness issues
        if (!data_freshness_.odom_fresh) {
            health_msg.status = "ERROR";
            health_msg.error_code = 18003;
            health_msg.error_message = "Stale odometry data";
        }
        
        health_msg.cpu_usage = 8.0; // Placeholder
        health_msg.memory_usage = 6.0; // Placeholder
        health_msg.temperature = 32.0; // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_sub_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr performance_metrics_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_alerts_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr navigation_quality_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr monitor_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::TimerInterface::SharedPtr monitor_timer_;
    rclcpp::TimerInterface::SharedPtr health_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double monitor_frequency_;
    double safety_distance_threshold_;
    double velocity_threshold_;
    double angular_velocity_threshold_;
    double goal_distance_threshold_;
    bool costmap_analysis_enabled_;
    bool performance_logging_enabled_;
    bool safety_monitoring_enabled_;
    std::string data_logging_path_;
    double alert_threshold_high_;
    double alert_threshold_medium_;
    std::string base_frame_;
    std::string map_frame_;
    
    // Monitoring state
    NavigationState current_navigation_state_;
    std::string current_mission_state_;
    std::string current_navigation_mode_;
    
    // Current data
    Pose2D current_pose_;
    Pose2D current_goal_;
    geometry_msgs::msg::Twist current_velocity_;
    geometry_msgs::msg::Twist current_cmd_vel_;
    sensor_msgs::msg::LaserScan current_laser_scan_;
    nav2_msgs::msg::Costmap current_costmap_;
    
    // Quality metrics
    double navigation_quality_score_;
    double obstacle_proximity_score_;
    double path_deviation_score_;
    double velocity_smoothness_score_;
    double costmap_quality_score_;
    
    // Navigation metrics
    NavigationMetrics navigation_metrics_;
    
    // Performance metrics structure
    struct PerformanceMetrics {
        double navigation_quality;
        double obstacle_proximity;
        double path_deviation;
        double velocity_smoothness;
        bool data_freshness;
        int recovery_events;
        NavigationMetrics navigation_metrics;
    } performance_metrics_;
    
    // Data freshness tracking
    struct DataFreshness {
        bool odom_fresh;
        bool laser_fresh;
        bool cmd_vel_fresh;
        bool costmap_fresh;
        bool goal_fresh;
    } data_freshness_;
    
    // Flags
    bool goal_available_;
    bool odom_available_;
    bool laser_available_;
    bool costmap_available_;
    
    // Event counters
    int recovery_events_ = 0;
    int emergency_events_ = 0;
    
    // Timing
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_laser_time_;
    rclcpp::Time last_cmd_vel_time_;
    rclcpp::Time last_costmap_time_;
    rclcpp::Time last_goal_time_;
    rclcpp::Time last_navigation_status_time_;
    rclcpp::Time last_mission_status_time_;
    
    // Velocity history for smoothness analysis
    std::vector<double> velocity_history_;
    std::vector<double> angular_velocity_history_;
    static constexpr size_t velocity_history_size_ = 50;
};

} // namespace tadeo_ecar_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_navigation::NavigationMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}