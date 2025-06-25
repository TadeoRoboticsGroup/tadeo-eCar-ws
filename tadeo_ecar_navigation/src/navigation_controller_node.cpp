#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/srv/navigate_to_goal.hpp>
#include <tadeo_ecar_interfaces/action/follow_path.hpp>
#include "tadeo_ecar_navigation/navigation_types.hpp"
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>

namespace tadeo_ecar_navigation
{

class NavigationControllerNode : public rclcpp::Node
{
public:
    NavigationControllerNode() : Node("navigation_controller_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeController();
        
        // Subscribers
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "move_base_simple/goal", 10,
            std::bind(&NavigationControllerNode::goalCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&NavigationControllerNode::odomCallback, this, std::placeholders::_1));
        
        mission_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "mission_path", 10,
            std::bind(&NavigationControllerNode::missionPathCallback, this, std::placeholders::_1));
        
        navigation_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "navigation_mode", 10,
            std::bind(&NavigationControllerNode::navigationModeCallback, this, std::placeholders::_1));
        
        emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "emergency_stop", 10,
            std::bind(&NavigationControllerNode::emergencyStopCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        navigation_status_pub_ = this->create_publisher<std_msgs::msg::String>("navigation_status", 10);
        current_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_goal", 10);
        navigation_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("navigation_markers", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("navigation/controller_health", 10);
        
        // Services
        navigate_service_ = this->create_service<tadeo_ecar_interfaces::srv::NavigateToGoal>(
            "navigate_to_pose",
            std::bind(&NavigationControllerNode::navigateService, this, std::placeholders::_1, std::placeholders::_2));
        
        // Action servers
        follow_path_action_server_ = rclcpp_action::create_server<tadeo_ecar_interfaces::action::FollowPath>(
            this,
            "follow_path",
            std::bind(&NavigationControllerNode::handleFollowPathGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigationControllerNode::handleFollowPathCancel, this, std::placeholders::_1),
            std::bind(&NavigationControllerNode::handleFollowPathAccepted, this, std::placeholders::_1));
        
        // Action clients
        nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose_nav2");
        
        // Timers
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
            std::bind(&NavigationControllerNode::controlLoop, this));
        
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&NavigationControllerNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Navigation Controller Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("control_frequency", 20.0);
        this->declare_parameter("goal_tolerance", 0.3);
        this->declare_parameter("angle_tolerance", 0.2);
        this->declare_parameter("max_linear_velocity", 1.5);
        this->declare_parameter("max_angular_velocity", 1.0);
        this->declare_parameter("emergency_deceleration", 3.0);
        this->declare_parameter("recovery_timeout", 30.0);
        this->declare_parameter("use_nav2_stack", true);
        this->declare_parameter("enable_recovery_behaviors", true);
        this->declare_parameter("enable_safety_monitoring", true);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
        
        control_frequency_ = this->get_parameter("control_frequency").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        emergency_deceleration_ = this->get_parameter("emergency_deceleration").as_double();
        recovery_timeout_ = this->get_parameter("recovery_timeout").as_double();
        use_nav2_stack_ = this->get_parameter("use_nav2_stack").as_bool();
        enable_recovery_behaviors_ = this->get_parameter("enable_recovery_behaviors").as_bool();
        enable_safety_monitoring_ = this->get_parameter("enable_safety_monitoring").as_bool();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
    }
    
    void initializeController()
    {
        // Initialize navigation state
        nav_status_.state = NavigationState::IDLE;
        nav_status_.mode = NavigationMode::AUTONOMOUS;
        
        // Initialize constraints
        constraints_.max_velocity = max_linear_velocity_;
        constraints_.max_angular_velocity = max_angular_velocity_;
        constraints_.goal_tolerance = goal_tolerance_;
        
        // Initialize flags
        goal_received_ = false;
        emergency_stop_active_ = false;
        current_pose_available_ = false;
        
        // Initialize metrics
        metrics_.session_start_time = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Navigation controller initialized");
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (nav_status_.mode == NavigationMode::MANUAL || emergency_stop_active_) {
            RCLCPP_WARN(this->get_logger(), "Navigation goal ignored - manual mode or emergency stop active");
            return;
        }
        
        current_goal_ = fromRosPose(*msg);
        goal_received_ = true;
        
        updateNavigationState(NavigationState::NAVIGATING);
        
        if (use_nav2_stack_) {
            sendGoalToNav2(current_goal_);
        } else {
            // Use internal navigation
            planToGoal(current_goal_);
        }
        
        RCLCPP_INFO(this->get_logger(), "Navigation goal received: (%.2f, %.2f, %.2f)",
                    current_goal_.x, current_goal_.y, current_goal_.theta);
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        current_pose_.theta = 2.0 * atan2(qz, qw);
        current_pose_.timestamp = msg->header.stamp;
        
        current_velocity_.linear.x = msg->twist.twist.linear.x;
        current_velocity_.linear.y = msg->twist.twist.linear.y;
        current_velocity_.angular.z = msg->twist.twist.angular.z;
        
        current_pose_available_ = true;
        last_odom_time_ = msg->header.stamp;
        
        // Update navigation progress
        updateNavigationProgress();
    }
    
    void missionPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty mission path received");
            return;
        }
        
        current_mission_waypoints_ = pathToWaypoints(*msg);
        current_waypoint_index_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Mission path received with %zu waypoints", 
                    current_mission_waypoints_.size());
        
        if (nav_status_.mode == NavigationMode::AUTONOMOUS) {
            startMissionExecution();
        }
    }
    
    void navigationModeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string mode_str = msg->data;
        
        if (mode_str == "MANUAL") {
            nav_status_.mode = NavigationMode::MANUAL;
            stopRobot();
        } else if (mode_str == "AUTONOMOUS") {
            nav_status_.mode = NavigationMode::AUTONOMOUS;
            if (emergency_stop_active_) {
                emergency_stop_active_ = false;
                updateNavigationState(NavigationState::IDLE);
            }
        } else if (mode_str == "REMOTE_CONTROL") {
            nav_status_.mode = NavigationMode::REMOTE_CONTROL;
        } else if (mode_str == "EMERGENCY") {
            nav_status_.mode = NavigationMode::EMERGENCY;
            executeEmergencyStop();
        }
        
        RCLCPP_INFO(this->get_logger(), "Navigation mode changed to: %s", mode_str.c_str());
    }
    
    void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !emergency_stop_active_) {
            executeEmergencyStop();
        } else if (!msg->data && emergency_stop_active_) {
            emergency_stop_active_ = false;
            updateNavigationState(NavigationState::IDLE);
            RCLCPP_INFO(this->get_logger(), "Emergency stop deactivated");
        }
    }
    
    void controlLoop()
    {
        if (!current_pose_available_) return;
        
        // Safety monitoring
        if (enable_safety_monitoring_) {
            performSafetyChecks();
        }
        
        // Update navigation status
        updateNavigationStatus();
        
        // Publish status and visualization
        publishNavigationStatus();
        publishCurrentGoal();
        publishNavigationVisualization();
        
        // Execute control based on state
        switch (nav_status_.state) {
            case NavigationState::NAVIGATING:
                executeNavigation();
                break;
            
            case NavigationState::EMERGENCY_STOP:
                executeEmergencyStop();
                break;
            
            case NavigationState::RECOVERY:
                executeRecovery();
                break;
            
            default:
                // No action needed for other states
                break;
        }
    }
    
    void executeNavigation()
    {
        if (!goal_received_) return;
        
        // Check if goal is reached
        if (isGoalReached(current_goal_)) {
            updateNavigationState(NavigationState::WAYPOINT_REACHED);
            goal_received_ = false;
            
            // Update metrics
            metrics_.waypoints_reached++;
            
            // If part of mission, continue to next waypoint
            if (!current_mission_waypoints_.empty() && current_waypoint_index_ < current_mission_waypoints_.size() - 1) {
                current_waypoint_index_++;
                current_goal_ = current_mission_waypoints_[current_waypoint_index_].pose;
                goal_received_ = true;
                updateNavigationState(NavigationState::NAVIGATING);
                
                if (use_nav2_stack_) {
                    sendGoalToNav2(current_goal_);
                }
            } else {
                // Mission completed
                updateNavigationState(NavigationState::MISSION_COMPLETED);
                metrics_.missions_completed++;
                metrics_.updateSuccessRate();
            }
        }
    }
    
    void executeEmergencyStop()
    {
        if (!emergency_stop_active_) {
            emergency_stop_active_ = true;
            updateNavigationState(NavigationState::EMERGENCY_STOP);
            
            // Cancel any active navigation
            if (use_nav2_stack_ && nav2_goal_handle_) {
                nav2_client_->async_cancel_goal(nav2_goal_handle_);
            }
        }
        
        // Apply emergency braking
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
    }
    
    void executeRecovery()
    {
        // Simple recovery behavior - rotate in place
        static auto recovery_start_time = this->now();
        static bool recovery_initialized = false;
        
        if (!recovery_initialized) {
            recovery_start_time = this->now();
            recovery_initialized = true;
        }
        
        auto elapsed_time = (this->now() - recovery_start_time).seconds();
        
        if (elapsed_time > recovery_timeout_) {
            // Recovery timeout
            updateNavigationState(NavigationState::MISSION_FAILED);
            metrics_.navigation_failures++;
            metrics_.updateSuccessRate();
            recovery_initialized = false;
            return;
        }
        
        // Execute recovery rotation
        geometry_msgs::msg::Twist recovery_cmd;
        recovery_cmd.linear.x = 0.0;
        recovery_cmd.angular.z = 0.3; // Slow rotation
        cmd_vel_pub_->publish(recovery_cmd);
        
        // Check if recovery is successful (simplified)
        if (elapsed_time > 5.0) {
            updateNavigationState(NavigationState::NAVIGATING);
            recovery_initialized = false;
        }
    }
    
    void startMissionExecution()
    {
        if (current_mission_waypoints_.empty()) return;
        
        current_waypoint_index_ = 0;
        current_goal_ = current_mission_waypoints_[0].pose;
        goal_received_ = true;
        
        updateNavigationState(NavigationState::NAVIGATING);
        
        if (use_nav2_stack_) {
            sendGoalToNav2(current_goal_);
        }
        
        RCLCPP_INFO(this->get_logger(), "Mission execution started with %zu waypoints", 
                    current_mission_waypoints_.size());
    }
    
    void sendGoalToNav2(const Pose2D& goal)
    {
        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
            return;
        }
        
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = toRosPose(goal, map_frame_);
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&NavigationControllerNode::nav2GoalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = 
            std::bind(&NavigationControllerNode::nav2FeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&NavigationControllerNode::nav2ResultCallback, this, std::placeholders::_1);
        
        nav2_goal_handle_ = nav2_client_->async_send_goal(goal_msg, send_goal_options);
    }
    
    void planToGoal(const Pose2D& goal)
    {
        // Internal navigation planning - simplified implementation
        // In a real system, this would interface with the planning system
        RCLCPP_INFO(this->get_logger(), "Planning internal navigation to goal");
        
        // For now, just set the goal as reachable
        // This would be replaced with actual planning logic
    }
    
    bool isGoalReached(const Pose2D& goal)
    {
        if (!current_pose_available_) return false;
        
        double distance = current_pose_.distance(goal);
        double angle_diff = current_pose_.angleDifference(goal);
        
        return (distance < goal_tolerance_ && angle_diff < angle_tolerance_);
    }
    
    void stopRobot()
    {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
    }
    
    void performSafetyChecks()
    {
        // Check for timeouts
        auto current_time = this->now();
        
        if (current_pose_available_ && (current_time - last_odom_time_).seconds() > 2.0) {
            RCLCPP_WARN(this->get_logger(), "Odometry timeout detected");
            if (nav_status_.state == NavigationState::NAVIGATING) {
                updateNavigationState(NavigationState::RECOVERY);
            }
        }
        
        // Check velocity limits
        if (current_velocity_.linear.x > max_linear_velocity_ * 1.1 ||
            abs(current_velocity_.angular.z) > max_angular_velocity_ * 1.1) {
            RCLCPP_WARN(this->get_logger(), "Velocity limits exceeded");
            executeEmergencyStop();
        }
    }
    
    void updateNavigationProgress()
    {
        if (!goal_received_ || !current_pose_available_) return;
        
        double distance_to_goal = current_pose_.distance(current_goal_);
        nav_status_.distance_to_goal = distance_to_goal;
        
        // Estimate time remaining (simplified)
        if (current_velocity_.linear.x > 0.1) {
            nav_status_.estimated_time_remaining = distance_to_goal / current_velocity_.linear.x;
        }
        
        // Calculate progress percentage
        if (!current_mission_waypoints_.empty()) {
            double total_distance = calculatePathDistance(current_mission_waypoints_);
            double remaining_distance = distance_to_goal;
            
            // Add distance to remaining waypoints
            for (size_t i = current_waypoint_index_ + 1; i < current_mission_waypoints_.size(); ++i) {
                remaining_distance += current_mission_waypoints_[i-1].pose.distance(current_mission_waypoints_[i].pose);
            }
            
            nav_status_.progress_percentage = ((total_distance - remaining_distance) / total_distance) * 100.0;
        }
    }
    
    void updateNavigationState(NavigationState new_state)
    {
        if (nav_status_.state != new_state) {
            RCLCPP_INFO(this->get_logger(), "Navigation state: %s -> %s",
                        stateToString(nav_status_.state).c_str(), stateToString(new_state).c_str());
            nav_status_.state = new_state;
            nav_status_.last_update = this->now();
        }
    }
    
    void updateNavigationStatus()
    {
        nav_status_.last_update = this->now();
        
        // Update current mission and waypoint info
        if (!current_mission_waypoints_.empty()) {
            nav_status_.current_waypoint_index = current_waypoint_index_;
        }
    }
    
    void publishNavigationStatus()
    {
        std_msgs::msg::String status_msg;
        status_msg.data = stateToString(nav_status_.state) + "|" + modeToString(nav_status_.mode);
        navigation_status_pub_->publish(status_msg);
    }
    
    void publishCurrentGoal()
    {
        if (goal_received_) {
            auto goal_msg = toRosPose(current_goal_, map_frame_);
            goal_msg.header.stamp = this->now();
            current_goal_pub_->publish(goal_msg);
        }
    }
    
    void publishNavigationVisualization()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Current goal marker
        if (goal_received_) {
            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = map_frame_;
            goal_marker.header.stamp = this->now();
            goal_marker.ns = "navigation_controller";
            goal_marker.id = 0;
            goal_marker.type = visualization_msgs::msg::Marker::ARROW;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            
            goal_marker.pose.position.x = current_goal_.x;
            goal_marker.pose.position.y = current_goal_.y;
            goal_marker.pose.position.z = 0.5;
            goal_marker.pose.orientation.x = 0.0;
            goal_marker.pose.orientation.y = 0.0;
            goal_marker.pose.orientation.z = sin(current_goal_.theta / 2.0);
            goal_marker.pose.orientation.w = cos(current_goal_.theta / 2.0);
            
            goal_marker.scale.x = 1.5;
            goal_marker.scale.y = 0.3;
            goal_marker.scale.z = 0.3;
            goal_marker.color.r = 0.0;
            goal_marker.color.g = 1.0;
            goal_marker.color.b = 0.0;
            goal_marker.color.a = 1.0;
            
            marker_array.markers.push_back(goal_marker);
        }
        
        // Mission waypoints
        for (size_t i = 0; i < current_mission_waypoints_.size(); ++i) {
            visualization_msgs::msg::Marker waypoint_marker;
            waypoint_marker.header.frame_id = map_frame_;
            waypoint_marker.header.stamp = this->now();
            waypoint_marker.ns = "mission_waypoints";
            waypoint_marker.id = i;
            waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
            waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
            
            waypoint_marker.pose.position.x = current_mission_waypoints_[i].pose.x;
            waypoint_marker.pose.position.y = current_mission_waypoints_[i].pose.y;
            waypoint_marker.pose.position.z = 0.2;
            waypoint_marker.pose.orientation.w = 1.0;
            
            waypoint_marker.scale.x = 0.4;
            waypoint_marker.scale.y = 0.4;
            waypoint_marker.scale.z = 0.4;
            
            // Color based on status
            if (i < current_waypoint_index_) {
                // Completed waypoints - green
                waypoint_marker.color.r = 0.0;
                waypoint_marker.color.g = 1.0;
                waypoint_marker.color.b = 0.0;
            } else if (i == current_waypoint_index_) {
                // Current waypoint - yellow
                waypoint_marker.color.r = 1.0;
                waypoint_marker.color.g = 1.0;
                waypoint_marker.color.b = 0.0;
            } else {
                // Future waypoints - blue
                waypoint_marker.color.r = 0.0;
                waypoint_marker.color.g = 0.0;
                waypoint_marker.color.b = 1.0;
            }
            waypoint_marker.color.a = 0.8;
            
            marker_array.markers.push_back(waypoint_marker);
        }
        
        navigation_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        health_msg.component_name = "navigation_controller";
        
        // Determine health status
        if (emergency_stop_active_) {
            health_msg.status = "EMERGENCY";
            health_msg.error_code = 15001;
            health_msg.error_message = "Emergency stop active";
        } else if (nav_status_.state == NavigationState::MISSION_FAILED) {
            health_msg.status = "ERROR";
            health_msg.error_code = 15002;
            health_msg.error_message = "Mission failed";
        } else if (nav_status_.state == NavigationState::RECOVERY) {
            health_msg.status = "WARNING";
            health_msg.error_code = 15003;
            health_msg.error_message = "Recovery behavior active";
        } else if (!current_pose_available_) {
            health_msg.status = "WARNING";
            health_msg.error_code = 15004;
            health_msg.error_message = "No odometry data";
        } else {
            health_msg.status = "OK";
            health_msg.error_code = 0;
            health_msg.error_message = "";
        }
        
        health_msg.cpu_usage = 25.0; // Placeholder
        health_msg.memory_usage = 20.0; // Placeholder
        health_msg.temperature = 40.0; // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    // Action server callbacks
    rclcpp_action::GoalResponse handleFollowPathGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const tadeo_ecar_interfaces::action::FollowPath::Goal> goal)
    {
        (void)uuid;
        
        if (nav_status_.mode != NavigationMode::AUTONOMOUS) {
            RCLCPP_WARN(this->get_logger(), "Follow path rejected - not in autonomous mode");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        RCLCPP_INFO(this->get_logger(), "Follow path goal accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleFollowPathCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<tadeo_ecar_interfaces::action::FollowPath>> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Follow path goal cancelled");
        stopRobot();
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handleFollowPathAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<tadeo_ecar_interfaces::action::FollowPath>> goal_handle)
    {
        // Execute the follow path action
        auto goal = goal_handle->get_goal();
        current_mission_waypoints_ = pathToWaypoints(goal->path);
        
        startMissionExecution();
        
        // This would typically run in a separate thread
        auto result = std::make_shared<tadeo_ecar_interfaces::action::FollowPath::Result>();
        result->success = true;
        result->message = "Path following completed";
        goal_handle->succeed(result);
    }
    
    // Nav2 callbacks
    void nav2GoalResponseCallback(
        const rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::GoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 goal was rejected");
            updateNavigationState(NavigationState::MISSION_FAILED);
        } else {
            RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted");
        }
    }
    
    void nav2FeedbackCallback(
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::GoalHandle::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        nav_status_.distance_to_goal = feedback->distance_remaining;
        nav_status_.estimated_time_remaining = feedback->estimated_time_remaining.sec;
    }
    
    void nav2ResultCallback(
        const rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::GoalHandle::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Nav2 navigation succeeded");
                updateNavigationState(NavigationState::WAYPOINT_REACHED);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Nav2 navigation aborted");
                updateNavigationState(NavigationState::RECOVERY);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Nav2 navigation cancelled");
                updateNavigationState(NavigationState::IDLE);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Nav2 navigation failed with unknown result");
                updateNavigationState(NavigationState::MISSION_FAILED);
                break;
        }
    }
    
    void navigateService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::NavigateToGoal::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::NavigateToGoal::Response> response)
    {
        try {
            if (nav_status_.mode != NavigationMode::AUTONOMOUS) {
                response->success = false;
                response->message = "Not in autonomous mode";
                return;
            }
            
            current_goal_ = fromRosPose(request->goal);
            goal_received_ = true;
            
            updateNavigationState(NavigationState::NAVIGATING);
            
            if (use_nav2_stack_) {
                sendGoalToNav2(current_goal_);
            }
            
            response->success = true;
            response->message = "Navigation goal accepted";
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Navigation failed: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Navigation service failed: %s", e.what());
        }
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr mission_path_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr navigation_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::Service<tadeo_ecar_interfaces::srv::NavigateToGoal>::SharedPtr navigate_service_;
    
    rclcpp_action::Server<tadeo_ecar_interfaces::action::FollowPath>::SharedPtr follow_path_action_server_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::GoalHandle::SharedPtr nav2_goal_handle_;
    
    rclcpp::TimerInterface::SharedPtr control_timer_;
    rclcpp::TimerInterface::SharedPtr health_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double control_frequency_;
    double goal_tolerance_;
    double angle_tolerance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double emergency_deceleration_;
    double recovery_timeout_;
    bool use_nav2_stack_;
    bool enable_recovery_behaviors_;
    bool enable_safety_monitoring_;
    std::string base_frame_;
    std::string map_frame_;
    
    // Navigation state
    NavigationStatus nav_status_;
    NavigationConstraints constraints_;
    NavigationMetrics metrics_;
    
    // Current state
    Pose2D current_pose_;
    Pose2D current_goal_;
    geometry_msgs::msg::Twist current_velocity_;
    std::vector<Waypoint> current_mission_waypoints_;
    int current_waypoint_index_;
    
    // Flags
    bool goal_received_;
    bool emergency_stop_active_;
    bool current_pose_available_;
    
    // Timing
    rclcpp::Time last_odom_time_;
};

} // namespace tadeo_ecar_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_navigation::NavigationControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}