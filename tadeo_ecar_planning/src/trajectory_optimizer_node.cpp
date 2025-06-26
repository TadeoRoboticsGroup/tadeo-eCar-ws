#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include "tadeo_ecar_planning/planning_types.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>

namespace tadeo_ecar_planning
{

class TrajectoryOptimizerNode : public rclcpp::Node
{
public:
    TrajectoryOptimizerNode() : Node("trajectory_optimizer_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeOptimizer();
        
        // Subscribers
        raw_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "raw_path", 10,
            std::bind(&TrajectoryOptimizerNode::rawPathCallback, this, std::placeholders::_1));
        
        // Publishers
        optimized_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("optimized_path", 10);
        velocity_profile_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity_profile", 10);
        trajectory_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("optimized_trajectory_markers", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("planning/optimizer_health", 10);
        
        // Timers
        optimization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / optimization_frequency_)),
            std::bind(&TrajectoryOptimizerNode::optimizationLoop, this));
        
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&TrajectoryOptimizerNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Trajectory Optimizer Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("optimization_frequency", 5.0);
        this->declare_parameter("max_velocity", 2.0);
        this->declare_parameter("max_acceleration", 1.0);
        this->declare_parameter("max_deceleration", 2.0);
        this->declare_parameter("max_jerk", 2.0);
        this->declare_parameter("max_curvature", 2.0);
        this->declare_parameter("smoothing_weight", 1.0);
        this->declare_parameter("curvature_weight", 10.0);
        this->declare_parameter("velocity_weight", 1.0);
        this->declare_parameter("acceleration_weight", 5.0);
        this->declare_parameter("jerk_weight", 10.0);
        this->declare_parameter("enable_velocity_optimization", true);
        this->declare_parameter("enable_path_smoothing", true);
        this->declare_parameter("enable_curvature_optimization", true);
        this->declare_parameter("optimization_iterations", 100);
        this->declare_parameter("convergence_threshold", 1e-6);
        this->declare_parameter("wheelbase", 0.3);
        this->declare_parameter("track_width", 0.25);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
        
        optimization_frequency_ = this->get_parameter("optimization_frequency").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        max_acceleration_ = this->get_parameter("max_acceleration").as_double();
        max_deceleration_ = this->get_parameter("max_deceleration").as_double();
        max_jerk_ = this->get_parameter("max_jerk").as_double();
        max_curvature_ = this->get_parameter("max_curvature").as_double();
        smoothing_weight_ = this->get_parameter("smoothing_weight").as_double();
        curvature_weight_ = this->get_parameter("curvature_weight").as_double();
        velocity_weight_ = this->get_parameter("velocity_weight").as_double();
        acceleration_weight_ = this->get_parameter("acceleration_weight").as_double();
        jerk_weight_ = this->get_parameter("jerk_weight").as_double();
        enable_velocity_optimization_ = this->get_parameter("enable_velocity_optimization").as_bool();
        enable_path_smoothing_ = this->get_parameter("enable_path_smoothing").as_bool();
        enable_curvature_optimization_ = this->get_parameter("enable_curvature_optimization").as_bool();
        optimization_iterations_ = this->get_parameter("optimization_iterations").as_int();
        convergence_threshold_ = this->get_parameter("convergence_threshold").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
    }
    
    void initializeOptimizer()
    {
        // Initialize vehicle constraints
        constraints_.max_velocity = max_velocity_;
        constraints_.max_acceleration = max_acceleration_;
        constraints_.max_deceleration = max_deceleration_;
        constraints_.wheelbase = wheelbase_;
        constraints_.track_width = track_width_;
        constraints_.max_steering_angle = atan(wheelbase_ * max_curvature_);
        
        // Initialize state
        raw_path_available_ = false;
        optimization_in_progress_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Optimizer initialized with max_velocity: %.2fm/s, max_acceleration: %.2fm/s²",
                    max_velocity_, max_acceleration_);
    }
    
    void rawPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            raw_path_available_ = false;
            return;
        }
        
        // Convert to internal path representation
        raw_path_.clear();
        raw_path_.frame_id = msg->header.frame_id;
        raw_path_.timestamp = msg->header.stamp;
        
        for (const auto& pose_stamped : msg->poses) {
            PathPoint point;
            point.pose.x = pose_stamped.pose.position.x;
            point.pose.y = pose_stamped.pose.position.y;
            
            // Convert quaternion to theta
            double qw = pose_stamped.pose.orientation.w;
            double qz = pose_stamped.pose.orientation.z;
            point.pose.theta = 2.0 * atan2(qz, qw);
            
            raw_path_.points.push_back(point);
        }
        
        raw_path_available_ = true;
        last_path_time_ = msg->header.stamp;
        should_optimize_ = true;
        
        RCLCPP_DEBUG(this->get_logger(), "Raw path updated with %zu points", raw_path_.size());
    }
    
    void optimizationLoop()
    {
        if (!should_optimize_ || optimization_in_progress_ || !raw_path_available_) return;
        
        optimizeTrajectory();
    }
    
    void optimizeTrajectory()
    {
        if (optimization_in_progress_ || raw_path_.empty()) return;
        
        optimization_in_progress_ = true;
        should_optimize_ = false;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        optimized_path_ = raw_path_;
        
        // Step 1: Path smoothing
        if (enable_path_smoothing_) {
            smoothPath(optimized_path_);
        }
        
        // Step 2: Calculate curvature
        calculateCurvature(optimized_path_);
        
        // Step 3: Curvature optimization
        if (enable_curvature_optimization_) {
            optimizeCurvature(optimized_path_);
        }
        
        // Step 4: Velocity profile optimization
        if (enable_velocity_optimization_) {
            optimizeVelocityProfile(optimized_path_);
        }
        
        // Step 5: Time parameterization
        parameterizeTime(optimized_path_);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Publish results
        publishOptimizedPath();
        publishVelocityProfile();
        publishTrajectoryVisualization();
        
        optimization_in_progress_ = false;
        last_optimization_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Trajectory optimized in %ldms (%zu points)",
                    duration.count(), optimized_path_.size());
    }
    
    void smoothPath(Path& path)
    {
        if (path.size() < 3) return;
        
        // Gradient descent smoothing
        for (int iter = 0; iter < optimization_iterations_; ++iter) {
            bool converged = true;
            
            for (size_t i = 1; i < path.size() - 1; ++i) {
                Pose2D old_pose = path[i].pose;
                
                // Calculate smoothing gradient
                double grad_x = smoothing_weight_ * (2.0 * path[i].pose.x - path[i-1].pose.x - path[i+1].pose.x);
                double grad_y = smoothing_weight_ * (2.0 * path[i].pose.y - path[i-1].pose.y - path[i+1].pose.y);
                
                // Update position
                double learning_rate = 0.01;
                path[i].pose.x -= learning_rate * grad_x;
                path[i].pose.y -= learning_rate * grad_y;
                
                // Check convergence
                double change = sqrt(pow(path[i].pose.x - old_pose.x, 2) + pow(path[i].pose.y - old_pose.y, 2));
                if (change > convergence_threshold_) {
                    converged = false;
                }
            }
            
            if (converged) {
                RCLCPP_DEBUG(this->get_logger(), "Path smoothing converged after %d iterations", iter);
                break;
            }
        }
        
        // Recalculate orientations after smoothing
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].pose.x - path[i-1].pose.x;
            double dy = path[i].pose.y - path[i-1].pose.y;
            path[i].pose.theta = atan2(dy, dx);
        }
    }
    
    void calculateCurvature(Path& path)
    {
        if (path.size() < 3) return;
        
        // Calculate curvature for each point
        for (size_t i = 1; i < path.size() - 1; ++i) {
            path[i].curvature = calculateCurvature(path[i-1].pose, path[i].pose, path[i+1].pose);
        }
        
        // Set boundary curvatures
        if (path.size() >= 2) {
            path[0].curvature = path[1].curvature;
            path[path.size()-1].curvature = path[path.size()-2].curvature;
        }
    }
    
    void optimizeCurvature(Path& path)
    {
        if (path.size() < 3) return;
        
        // Optimize path to reduce curvature while maintaining smoothness
        for (int iter = 0; iter < optimization_iterations_ / 2; ++iter) {
            bool converged = true;
            
            for (size_t i = 2; i < path.size() - 2; ++i) {
                Pose2D old_pose = path[i].pose;
                
                // Calculate curvature gradient
                double current_curvature = abs(path[i].curvature);
                if (current_curvature > max_curvature_) {
                    // Reduce curvature by adjusting position
                    double adjustment_factor = 0.1;
                    
                    // Move point towards the line connecting neighbors
                    double line_x = (path[i-1].pose.x + path[i+1].pose.x) / 2.0;
                    double line_y = (path[i-1].pose.y + path[i+1].pose.y) / 2.0;
                    
                    path[i].pose.x += adjustment_factor * (line_x - path[i].pose.x);
                    path[i].pose.y += adjustment_factor * (line_y - path[i].pose.y);
                    
                    // Check convergence
                    double change = sqrt(pow(path[i].pose.x - old_pose.x, 2) + pow(path[i].pose.y - old_pose.y, 2));
                    if (change > convergence_threshold_) {
                        converged = false;
                    }
                }
            }
            
            // Recalculate curvature after adjustment
            calculateCurvature(path);
            
            if (converged) {
                RCLCPP_DEBUG(this->get_logger(), "Curvature optimization converged after %d iterations", iter);
                break;
            }
        }
    }
    
    void optimizeVelocityProfile(Path& path)
    {
        if (path.empty()) return;
        
        // Forward pass: limit velocity based on curvature and acceleration
        for (size_t i = 0; i < path.size(); ++i) {
            // Curvature-limited velocity
            double curvature_velocity = max_velocity_;
            if (abs(path[i].curvature) > 1e-6) {
                curvature_velocity = std::min(max_velocity_, sqrt(max_acceleration_ / abs(path[i].curvature)));
            }
            
            path[i].speed = std::min(max_velocity_, curvature_velocity);
        }
        
        // Backward pass: enforce deceleration limits
        for (int i = static_cast<int>(path.size()) - 2; i >= 0; --i) {
            double distance = path[i].pose.distance(path[i+1].pose);
            if (distance > 1e-6) {
                double max_velocity_from_next = sqrt(path[i+1].speed * path[i+1].speed + 2 * max_deceleration_ * distance);
                path[i].speed = std::min(path[i].speed, max_velocity_from_next);
            }
        }
        
        // Forward pass: enforce acceleration limits
        for (size_t i = 1; i < path.size(); ++i) {
            double distance = path[i-1].pose.distance(path[i].pose);
            if (distance > 1e-6) {
                double max_velocity_from_prev = sqrt(path[i-1].speed * path[i-1].speed + 2 * max_acceleration_ * distance);
                path[i].speed = std::min(path[i].speed, max_velocity_from_prev);
            }
        }
        
        // Smooth velocity profile
        for (int iter = 0; iter < 10; ++iter) {
            for (size_t i = 1; i < path.size() - 1; ++i) {
                double smoothed_speed = (path[i-1].speed + 2.0 * path[i].speed + path[i+1].speed) / 4.0;
                path[i].speed = smoothed_speed;
            }
        }
    }
    
    void parameterizeTime(Path& path)
    {
        if (path.empty()) return;
        
        path[0].time_from_start = 0.0;
        
        for (size_t i = 1; i < path.size(); ++i) {
            double distance = path[i-1].pose.distance(path[i].pose);
            double avg_speed = (path[i-1].speed + path[i].speed) / 2.0;
            
            if (avg_speed > 1e-6) {
                double dt = distance / avg_speed;
                path[i].time_from_start = path[i-1].time_from_start + dt;
            } else {
                path[i].time_from_start = path[i-1].time_from_start + 0.1; // Default time step
            }
        }
        
        // Calculate total time
        if (!path.empty()) {
            optimized_path_.total_time = path.back().time_from_start;
        }
    }
    
    void publishOptimizedPath()
    {
        if (optimized_path_.empty()) return;
        
        nav_msgs::msg::Path ros_path;
        ros_path.header.stamp = this->now();
        ros_path.header.frame_id = map_frame_;
        
        for (const auto& point : optimized_path_.points) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = ros_path.header;
            pose_stamped.pose.position.x = point.pose.x;
            pose_stamped.pose.position.y = point.pose.y;
            pose_stamped.pose.position.z = 0.0;
            
            // Convert theta to quaternion
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = sin(point.pose.theta / 2.0);
            pose_stamped.pose.orientation.w = cos(point.pose.theta / 2.0);
            
            ros_path.poses.push_back(pose_stamped);
        }
        
        optimized_path_pub_->publish(ros_path);
    }
    
    void publishVelocityProfile()
    {
        if (optimized_path_.empty()) return;
        
        // Publish velocity at current time or first point
        geometry_msgs::msg::TwistStamped velocity_msg;
        velocity_msg.header.stamp = this->now();
        velocity_msg.header.frame_id = base_frame_;
        
        // Use first point's velocity
        velocity_msg.twist.linear.x = optimized_path_[0].speed;
        velocity_msg.twist.angular.z = 0.0; // Will be calculated by local planner
        
        velocity_profile_pub_->publish(velocity_msg);
    }
    
    void publishTrajectoryVisualization()
    {
        if (optimized_path_.empty()) return;
        
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Optimized path
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = map_frame_;
        path_marker.header.stamp = this->now();
        path_marker.ns = "optimized_trajectory";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        
        for (const auto& point : optimized_path_.points) {
            geometry_msgs::msg::Point p;
            p.x = point.pose.x;
            p.y = point.pose.y;
            p.z = 0.1;
            path_marker.points.push_back(p);
        }
        
        path_marker.scale.x = 0.04;
        path_marker.color.r = 0.0;
        path_marker.color.g = 0.0;
        path_marker.color.b = 1.0;
        path_marker.color.a = 1.0;
        
        marker_array.markers.push_back(path_marker);
        
        // Velocity visualization (colored by speed)
        for (size_t i = 0; i < optimized_path_.size(); i += 5) { // Sample every 5th point
            visualization_msgs::msg::Marker vel_marker;
            vel_marker.header.frame_id = map_frame_;
            vel_marker.header.stamp = this->now();
            vel_marker.ns = "velocity_profile";
            vel_marker.id = i;
            vel_marker.type = visualization_msgs::msg::Marker::SPHERE;
            vel_marker.action = visualization_msgs::msg::Marker::ADD;
            
            vel_marker.pose.position.x = optimized_path_[i].pose.x;
            vel_marker.pose.position.y = optimized_path_[i].pose.y;
            vel_marker.pose.position.z = 0.2;
            vel_marker.pose.orientation.w = 1.0;
            
            double size = 0.1 + 0.2 * (optimized_path_[i].speed / max_velocity_);
            vel_marker.scale.x = size;
            vel_marker.scale.y = size;
            vel_marker.scale.z = size;
            
            // Color based on speed (blue = slow, red = fast)
            double speed_ratio = optimized_path_[i].speed / max_velocity_;
            vel_marker.color.r = speed_ratio;
            vel_marker.color.g = 0.5;
            vel_marker.color.b = 1.0 - speed_ratio;
            vel_marker.color.a = 0.8;
            
            marker_array.markers.push_back(vel_marker);
        }
        
        // Curvature visualization
        for (size_t i = 0; i < optimized_path_.size(); i += 10) { // Sample every 10th point
            if (abs(optimized_path_[i].curvature) > 0.1) { // Only show significant curvature
                visualization_msgs::msg::Marker curve_marker;
                curve_marker.header.frame_id = map_frame_;
                curve_marker.header.stamp = this->now();
                curve_marker.ns = "curvature";
                curve_marker.id = i;
                curve_marker.type = visualization_msgs::msg::Marker::CYLINDER;
                curve_marker.action = visualization_msgs::msg::Marker::ADD;
                
                curve_marker.pose.position.x = optimized_path_[i].pose.x;
                curve_marker.pose.position.y = optimized_path_[i].pose.y;
                curve_marker.pose.position.z = 0.0;
                curve_marker.pose.orientation.w = 1.0;
                
                double curvature_ratio = std::min(1.0, abs(optimized_path_[i].curvature) / max_curvature_);
                curve_marker.scale.x = 0.1 * curvature_ratio;
                curve_marker.scale.y = 0.1 * curvature_ratio;
                curve_marker.scale.z = 0.05;
                
                curve_marker.color.r = 1.0;
                curve_marker.color.g = 1.0 - curvature_ratio;
                curve_marker.color.b = 0.0;
                curve_marker.color.a = 0.6;
                
                marker_array.markers.push_back(curve_marker);
            }
        }
        
        trajectory_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        // Remove component_name field - not part of SystemHealth message
        
        // Check system health
        auto current_time = this->now();
        bool path_timeout = raw_path_available_ && (current_time - last_path_time_).seconds() > 5.0;
        
        if (path_timeout) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(13001);
            health_msg.error_messages.push_back("Raw path timeout");
        } else if (optimization_in_progress_) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(13002);
            health_msg.error_messages.push_back("Optimization in progress");
        } else if (!raw_path_available_) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(13003);
            health_msg.error_messages.push_back("Waiting for raw path");
        } else {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(0);
            health_msg.error_messages.push_back("");
        }
        
        // Replace with cpu_temperature = 20.0; // See SystemHealth.msg // Placeholder
        // Replace with appropriate field - memory_usage not in SystemHealth.msg // Placeholder
        // Replace with specific temperature fields: cpu_temperature, gpu_temperature, motor_temperature // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_path_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_profile_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::TimerBase::SharedPtr optimization_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double optimization_frequency_;
    double max_velocity_;
    double max_acceleration_;
    double max_deceleration_;
    double max_jerk_;
    double max_curvature_;
    double smoothing_weight_;
    double curvature_weight_;
    double velocity_weight_;
    double acceleration_weight_;
    double jerk_weight_;
    bool enable_velocity_optimization_;
    bool enable_path_smoothing_;
    bool enable_curvature_optimization_;
    int optimization_iterations_;
    double convergence_threshold_;
    double wheelbase_;
    double track_width_;
    std::string base_frame_;
    std::string map_frame_;
    
    // Optimization state
    Path raw_path_;
    Path optimized_path_;
    VehicleConstraints constraints_;
    
    bool raw_path_available_;
    bool optimization_in_progress_;
    bool should_optimize_;
    
    rclcpp::Time last_path_time_;
    rclcpp::Time last_optimization_time_;
};

} // namespace tadeo_ecar_planning

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_planning::TrajectoryOptimizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}