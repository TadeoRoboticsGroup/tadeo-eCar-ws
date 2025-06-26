#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/action/navigate_to_goal.hpp>
#include "tadeo_ecar_planning/planning_types.hpp"
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>

namespace tadeo_ecar_planning
{

class PathManagerNode : public rclcpp::Node
{
public:
    PathManagerNode() : Node("path_manager_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeManager();
        
        // Subscribers
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_path", 10,
            std::bind(&PathManagerNode::globalPathCallback, this, std::placeholders::_1));
        
        optimized_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "optimized_path", 10,
            std::bind(&PathManagerNode::optimizedPathCallback, this, std::placeholders::_1));
        
        local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "local_path", 10,
            std::bind(&PathManagerNode::localPathCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&PathManagerNode::odomCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "move_base_simple/goal", 10,
            std::bind(&PathManagerNode::goalCallback, this, std::placeholders::_1));
        
        // Publishers
        final_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("final_path", 10);
        navigation_status_pub_ = this->create_publisher<std_msgs::msg::String>("navigation_status", 10);
        path_progress_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("path_progress", 10);
        planning_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("planning_visualization", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("planning/manager_health", 10);
        
        // Services
        navigation_service_ = this->create_service<tadeo_ecar_interfaces::srv::NavigateToGoal>(
            "navigate_to_goal",
            std::bind(&PathManagerNode::navigationService, this, std::placeholders::_1, std::placeholders::_2));
        
        // Timers
        management_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / management_frequency_)),
            std::bind(&PathManagerNode::managementLoop, this));
        
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PathManagerNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Path Manager Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("management_frequency", 10.0);
        this->declare_parameter("goal_tolerance", 0.3);
        this->declare_parameter("path_timeout", 5.0);
        this->declare_parameter("replanning_threshold", 1.0);
        this->declare_parameter("max_planning_attempts", 3);
        this->declare_parameter("enable_path_monitoring", true);
        this->declare_parameter("enable_progress_tracking", true);
        this->declare_parameter("enable_automatic_replanning", true);
        this->declare_parameter("path_deviation_threshold", 2.0);
        this->declare_parameter("stuck_detection_time", 10.0);
        this->declare_parameter("stuck_distance_threshold", 0.1);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
        
        management_frequency_ = this->get_parameter("management_frequency").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        path_timeout_ = this->get_parameter("path_timeout").as_double();
        replanning_threshold_ = this->get_parameter("replanning_threshold").as_double();
        max_planning_attempts_ = this->get_parameter("max_planning_attempts").as_int();
        enable_path_monitoring_ = this->get_parameter("enable_path_monitoring").as_bool();
        enable_progress_tracking_ = this->get_parameter("enable_progress_tracking").as_bool();
        enable_automatic_replanning_ = this->get_parameter("enable_automatic_replanning").as_bool();
        path_deviation_threshold_ = this->get_parameter("path_deviation_threshold").as_double();
        stuck_detection_time_ = this->get_parameter("stuck_detection_time").as_double();
        stuck_distance_threshold_ = this->get_parameter("stuck_distance_threshold").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
    }
    
    void initializeManager()
    {
        // Initialize navigation state
        navigation_state_ = NavigationState::IDLE;
        planning_attempts_ = 0;
        
        // Initialize path tracking
        path_progress_ = 0.0;
        closest_point_index_ = 0;
        
        // Initialize availability flags
        global_path_available_ = false;
        optimized_path_available_ = false;
        local_path_available_ = false;
        current_pose_available_ = false;
        goal_set_ = false;
        
        // Initialize timing
        last_movement_time_ = this->now();
        last_position_ = Pose2D(0, 0, 0);
        
        RCLCPP_INFO(this->get_logger(), "Path manager initialized");
    }
    
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        global_path_ = convertRosPathToInternal(*msg);
        global_path_available_ = !msg->poses.empty();
        last_global_path_time_ = msg->header.stamp;
        
        if (global_path_available_) {
            RCLCPP_INFO(this->get_logger(), "Global path received with %zu points", global_path_.size());
            updateNavigationState(NavigationState::GLOBAL_PLANNING_SUCCESS);
        }
    }
    
    void optimizedPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        optimized_path_ = convertRosPathToInternal(*msg);
        optimized_path_available_ = !msg->poses.empty();
        last_optimized_path_time_ = msg->header.stamp;
        
        if (optimized_path_available_) {
            RCLCPP_DEBUG(this->get_logger(), "Optimized path received with %zu points", optimized_path_.size());
            updateFinalPath();
        }
    }
    
    void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        local_path_ = convertRosPathToInternal(*msg);
        local_path_available_ = !msg->poses.empty();
        last_local_path_time_ = msg->header.stamp;
        
        RCLCPP_DEBUG(this->get_logger(), "Local path received with %zu points", local_path_.size());
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        current_pose_.theta = 2.0 * atan2(qz, qw);
        current_pose_.timestamp = msg->header.stamp;
        
        current_pose_available_ = true;
        last_odom_time_ = msg->header.stamp;
        
        // Update movement detection
        updateMovementDetection();
        
        // Update path progress
        if (enable_progress_tracking_) {
            updatePathProgress();
        }
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_.x = msg->pose.position.x;
        goal_pose_.y = msg->pose.position.y;
        
        double qw = msg->pose.orientation.w;
        double qz = msg->pose.orientation.z;
        goal_pose_.theta = 2.0 * atan2(qz, qw);
        
        goal_set_ = true;
        planning_attempts_ = 0;
        
        updateNavigationState(NavigationState::PLANNING_REQUESTED);
        
        RCLCPP_INFO(this->get_logger(), "New navigation goal: (%.2f, %.2f, %.2f)", 
                    goal_pose_.x, goal_pose_.y, goal_pose_.theta);
    }
    
    void managementLoop()
    {
        if (!current_pose_available_) return;
        
        // Check for timeouts
        checkTimeouts();
        
        // Monitor navigation progress
        if (enable_path_monitoring_) {
            monitorNavigation();
        }
        
        // Check if goal is reached
        if (goal_set_ && isGoalReached()) {
            updateNavigationState(NavigationState::GOAL_REACHED);
            goal_set_ = false;
        }
        
        // Handle automatic replanning
        if (enable_automatic_replanning_ && shouldReplan()) {
            requestReplanning();
        }
        
        // Update final path and publish
        updateFinalPath();
        publishNavigationStatus();
        publishPathProgress();
        publishPlanningVisualization();
    }
    
    void checkTimeouts()
    {
        auto current_time = this->now();
        
        if (global_path_available_ && (current_time - last_global_path_time_).seconds() > path_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Global path timeout");
            global_path_available_ = false;
        }
        
        if (optimized_path_available_ && (current_time - last_optimized_path_time_).seconds() > path_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Optimized path timeout");
            optimized_path_available_ = false;
        }
        
        if (local_path_available_ && (current_time - last_local_path_time_).seconds() > path_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Local path timeout");
            local_path_available_ = false;
        }
    }
    
    void monitorNavigation()
    {
        // Check if robot is stuck
        if (isRobotStuck()) {
            RCLCPP_WARN(this->get_logger(), "Robot appears to be stuck, requesting replanning");
            updateNavigationState(NavigationState::STUCK);
            requestReplanning();
        }
        
        // Check path deviation
        if (final_path_available_ && isDeviatingFromPath()) {
            RCLCPP_WARN(this->get_logger(), "Significant path deviation detected");
            if (enable_automatic_replanning_) {
                requestReplanning();
            }
        }
    }
    
    bool isGoalReached()
    {
        if (!goal_set_ || !current_pose_available_) return false;
        
        double distance = current_pose_.distance(goal_pose_);
        double angle_diff = current_pose_.angleDifference(goal_pose_);
        
        return (distance < goal_tolerance_ && angle_diff < 0.2);
    }
    
    bool shouldReplan()
    {
        // Replan if no path available after timeout
        if (goal_set_ && !global_path_available_ && 
            (this->now() - last_global_path_time_).seconds() > path_timeout_) {
            return true;
        }
        
        // Replan if planning failed multiple times
        if (navigation_state_ == NavigationState::PLANNING_FAILED && 
            planning_attempts_ < max_planning_attempts_) {
            return true;
        }
        
        return false;
    }
    
    void requestReplanning()
    {
        if (planning_attempts_ < max_planning_attempts_) {
            planning_attempts_++;
            updateNavigationState(NavigationState::REPLANNING);
            
            // Clear old paths
            global_path_available_ = false;
            optimized_path_available_ = false;
            
            RCLCPP_INFO(this->get_logger(), "Requesting replanning (attempt %d/%d)", 
                        planning_attempts_, max_planning_attempts_);
        } else {
            updateNavigationState(NavigationState::PLANNING_FAILED);
            RCLCPP_ERROR(this->get_logger(), "Max planning attempts reached, navigation failed");
        }
    }
    
    bool isRobotStuck()
    {
        if (!current_pose_available_) return false;
        
        double distance_moved = current_pose_.distance(last_position_);
        double time_elapsed = (this->now() - last_movement_time_).seconds();
        
        if (distance_moved > stuck_distance_threshold_) {
            last_movement_time_ = this->now();
            last_position_ = current_pose_;
            return false;
        }
        
        return time_elapsed > stuck_detection_time_;
    }
    
    bool isDeviatingFromPath()
    {
        if (!final_path_available_ || final_path_.empty()) return false;
        
        double min_distance = std::numeric_limits<double>::max();
        
        for (const auto& point : final_path_.points) {
            double dist = current_pose_.distance(point.pose);
            min_distance = std::min(min_distance, dist);
        }
        
        return min_distance > path_deviation_threshold_;
    }
    
    void updateMovementDetection()
    {
        if (!current_pose_available_) return;
        
        double distance_moved = current_pose_.distance(last_position_);
        
        if (distance_moved > stuck_distance_threshold_) {
            last_movement_time_ = this->now();
            last_position_ = current_pose_;
        }
    }
    
    void updatePathProgress()
    {
        if (!final_path_available_ || final_path_.empty()) return;
        
        // Find closest point on path
        double min_distance = std::numeric_limits<double>::max();
        size_t closest_index = 0;
        
        for (size_t i = closest_point_index_; i < final_path_.size(); ++i) {
            double dist = current_pose_.distance(final_path_[i].pose);
            if (dist < min_distance) {
                min_distance = dist;
                closest_index = i;
            }
        }
        
        closest_point_index_ = closest_index;
        path_progress_ = static_cast<double>(closest_index) / final_path_.size();
        
        RCLCPP_DEBUG(this->get_logger(), "Path progress: %.1f%%", path_progress_ * 100.0);
    }
    
    void updateFinalPath()
    {
        // Priority: optimized > global > local
        if (optimized_path_available_) {
            final_path_ = optimized_path_;
            final_path_available_ = true;
        } else if (global_path_available_) {
            final_path_ = global_path_;
            final_path_available_ = true;
        } else if (local_path_available_) {
            final_path_ = local_path_;
            final_path_available_ = true;
        } else {
            final_path_available_ = false;
        }
        
        if (final_path_available_) {
            publishFinalPath();
        }
    }
    
    void updateNavigationState(NavigationState new_state)
    {
        if (navigation_state_ != new_state) {
            RCLCPP_INFO(this->get_logger(), "Navigation state: %s -> %s", 
                        stateToString(navigation_state_).c_str(), stateToString(new_state).c_str());
            navigation_state_ = new_state;
        }
    }
    
    Path convertRosPathToInternal(const nav_msgs::msg::Path& ros_path)
    {
        Path path;
        path.frame_id = ros_path.header.frame_id;
        path.timestamp = ros_path.header.stamp;
        
        for (const auto& pose_stamped : ros_path.poses) {
            PathPoint point;
            point.pose.x = pose_stamped.pose.position.x;
            point.pose.y = pose_stamped.pose.position.y;
            
            double qw = pose_stamped.pose.orientation.w;
            double qz = pose_stamped.pose.orientation.z;
            point.pose.theta = 2.0 * atan2(qz, qw);
            
            path.points.push_back(point);
        }
        
        return path;
    }
    
    void publishFinalPath()
    {
        if (!final_path_available_) return;
        
        nav_msgs::msg::Path ros_path;
        ros_path.header.stamp = this->now();
        ros_path.header.frame_id = map_frame_;
        
        for (const auto& point : final_path_.points) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = ros_path.header;
            pose_stamped.pose.position.x = point.pose.x;
            pose_stamped.pose.position.y = point.pose.y;
            pose_stamped.pose.position.z = 0.0;
            
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = sin(point.pose.theta / 2.0);
            pose_stamped.pose.orientation.w = cos(point.pose.theta / 2.0);
            
            ros_path.poses.push_back(pose_stamped);
        }
        
        final_path_pub_->publish(ros_path);
    }
    
    void publishNavigationStatus()
    {
        std_msgs::msg::String status_msg;
        status_msg.data = stateToString(navigation_state_);
        navigation_status_pub_->publish(status_msg);
    }
    
    void publishPathProgress()
    {
        if (!final_path_available_ || final_path_.empty()) return;
        
        geometry_msgs::msg::PoseStamped progress_msg;
        progress_msg.header.stamp = this->now();
        progress_msg.header.frame_id = map_frame_;
        
        if (closest_point_index_ < final_path_.size()) {
            const auto& point = final_path_[closest_point_index_];
            progress_msg.pose.position.x = point.pose.x;
            progress_msg.pose.position.y = point.pose.y;
            progress_msg.pose.position.z = 0.0;
            
            progress_msg.pose.orientation.x = 0.0;
            progress_msg.pose.orientation.y = 0.0;
            progress_msg.pose.orientation.z = sin(point.pose.theta / 2.0);
            progress_msg.pose.orientation.w = cos(point.pose.theta / 2.0);
        }
        
        path_progress_pub_->publish(progress_msg);
    }
    
    void publishPlanningVisualization()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Current goal marker
        if (goal_set_) {
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = map_frame_;
            goal_marker.header.stamp = this->now();
            goal_marker.ns = "planning_manager";
            goal_marker.id = 0;
            goal_marker.type = visualization_msgs::msg::Marker::ARROW;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            
            goal_marker.pose.position.x = goal_pose_.x;
            goal_marker.pose.position.y = goal_pose_.y;
            goal_marker.pose.position.z = 0.5;
            goal_marker.pose.orientation.x = 0.0;
            goal_marker.pose.orientation.y = 0.0;
            goal_marker.pose.orientation.z = sin(goal_pose_.theta / 2.0);
            goal_marker.pose.orientation.w = cos(goal_pose_.theta / 2.0);
            
            goal_marker.scale.x = 1.0;
            goal_marker.scale.y = 0.2;
            goal_marker.scale.z = 0.2;
            goal_marker.color.r = 1.0;
            goal_marker.color.g = 0.0;
            goal_marker.color.b = 0.0;
            goal_marker.color.a = 1.0;
            
            marker_array.markers.push_back(goal_marker);
        }
        
        // Navigation state text
        visualization_msgs::msg::Marker state_marker;
        state_marker.header.frame_id = map_frame_;
        state_marker.header.stamp = this->now();
        state_marker.ns = "planning_manager";
        state_marker.id = 1;
        state_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        state_marker.action = visualization_msgs::msg::Marker::ADD;
        
        state_marker.pose.position.x = current_pose_.x;
        state_marker.pose.position.y = current_pose_.y + 1.0;
        state_marker.pose.position.z = 1.0;
        state_marker.pose.orientation.w = 1.0;
        
        state_marker.text = stateToString(navigation_state_);
        state_marker.scale.z = 0.3;
        state_marker.color.r = 1.0;
        state_marker.color.g = 1.0;
        state_marker.color.b = 1.0;
        state_marker.color.a = 1.0;
        
        marker_array.markers.push_back(state_marker);
        
        planning_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        // Remove component_name field - not part of SystemHealth message
        
        // Check system health based on navigation state
        switch (navigation_state_) {
            case NavigationState::IDLE:
                // Use appropriate status enum instead of string status
                health_msg.error_codes.push_back(0);
                health_msg.error_messages.push_back("Ready for navigation");
                break;
            
            case NavigationState::PLANNING_REQUESTED:
            case NavigationState::REPLANNING:
                // Use appropriate status enum instead of string status
                health_msg.error_codes.push_back(14001);
                health_msg.error_messages.push_back("Planning in progress");
                break;
            
            case NavigationState::NAVIGATING:
                // Use appropriate status enum instead of string status
                health_msg.error_codes.push_back(0);
                health_msg.error_messages.push_back("Navigating to goal");
                break;
            
            case NavigationState::GOAL_REACHED:
                // Use appropriate status enum instead of string status
                health_msg.error_codes.push_back(0);
                health_msg.error_messages.push_back("Goal reached");
                break;
            
            case NavigationState::STUCK:
                // Use appropriate status enum instead of string status
                health_msg.error_codes.push_back(14002);
                health_msg.error_messages.push_back("Robot stuck");
                break;
            
            case NavigationState::PLANNING_FAILED:
                // Use appropriate status enum instead of string status
                health_msg.error_codes.push_back(14003);
                health_msg.error_messages.push_back("Planning failed");
                break;
            
            default:
                // Use appropriate status enum instead of string status
                health_msg.error_codes.push_back(14999);
                health_msg.error_messages.push_back("Unknown state");
                break;
        }
        
        // Replace with cpu_temperature = 15.0; // See SystemHealth.msg // Placeholder
        // Replace with appropriate field - memory_usage not in SystemHealth.msg // Placeholder
        // Replace with specific temperature fields: cpu_temperature, gpu_temperature, motor_temperature // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    void navigationService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::NavigateToGoal::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::NavigateToGoal::Response> response)
    {
        try {
            // Set goal from service request
            goal_pose_.x = request->goal.pose.position.x;
            goal_pose_.y = request->goal.pose.position.y;
            
            double qw = request->goal.pose.orientation.w;
            double qz = request->goal.pose.orientation.z;
            goal_pose_.theta = 2.0 * atan2(qz, qw);
            
            goal_set_ = true;
            planning_attempts_ = 0;
            
            updateNavigationState(NavigationState::PLANNING_REQUESTED);
            
            response->success = true;
            response->message = "Navigation goal accepted";
            
            RCLCPP_INFO(this->get_logger(), "Navigation service called: goal (%.2f, %.2f, %.2f)",
                        goal_pose_.x, goal_pose_.y, goal_pose_.theta);
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Navigation failed: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Navigation service failed: %s", e.what());
        }
    }
    
    enum class NavigationState {
        IDLE,
        PLANNING_REQUESTED,
        GLOBAL_PLANNING_SUCCESS,
        NAVIGATING,
        GOAL_REACHED,
        STUCK,
        REPLANNING,
        PLANNING_FAILED
    };
    
    std::string stateToString(NavigationState state)
    {
        switch (state) {
            case NavigationState::IDLE: return "IDLE";
            case NavigationState::PLANNING_REQUESTED: return "PLANNING_REQUESTED";
            case NavigationState::GLOBAL_PLANNING_SUCCESS: return "GLOBAL_PLANNING_SUCCESS";
            case NavigationState::NAVIGATING: return "NAVIGATING";
            case NavigationState::GOAL_REACHED: return "GOAL_REACHED";
            case NavigationState::STUCK: return "STUCK";
            case NavigationState::REPLANNING: return "REPLANNING";
            case NavigationState::PLANNING_FAILED: return "PLANNING_FAILED";
            default: return "UNKNOWN";
        }
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr optimized_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr final_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr path_progress_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planning_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::Service<tadeo_ecar_interfaces::srv::NavigateToGoal>::SharedPtr navigation_service_;
    
    rclcpp::TimerBase::SharedPtr management_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double management_frequency_;
    double goal_tolerance_;
    double path_timeout_;
    double replanning_threshold_;
    int max_planning_attempts_;
    bool enable_path_monitoring_;
    bool enable_progress_tracking_;
    bool enable_automatic_replanning_;
    double path_deviation_threshold_;
    double stuck_detection_time_;
    double stuck_distance_threshold_;
    std::string base_frame_;
    std::string map_frame_;
    
    // Navigation state
    NavigationState navigation_state_;
    int planning_attempts_;
    
    // Paths
    Path global_path_;
    Path optimized_path_;
    Path local_path_;
    Path final_path_;
    
    // Poses
    Pose2D current_pose_;
    Pose2D goal_pose_;
    Pose2D last_position_;
    
    // Path tracking
    double path_progress_;
    size_t closest_point_index_;
    
    // Availability flags
    bool global_path_available_;
    bool optimized_path_available_;
    bool local_path_available_;
    bool final_path_available_;
    bool current_pose_available_;
    bool goal_set_;
    
    // Timing
    rclcpp::Time last_global_path_time_;
    rclcpp::Time last_optimized_path_time_;
    rclcpp::Time last_local_path_time_;
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_movement_time_;
};

} // namespace tadeo_ecar_planning

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_planning::PathManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}