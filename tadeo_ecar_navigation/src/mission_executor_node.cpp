#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/action/follow_path.hpp>
#include <tadeo_ecar_interfaces/srv/navigate_to_goal.hpp>
#include "tadeo_ecar_navigation/navigation_types.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>

namespace tadeo_ecar_navigation
{

class MissionExecutorNode : public rclcpp::Node
{
public:
    MissionExecutorNode() : Node("mission_executor_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeExecutor();
        
        // Subscribers
        mission_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "mission_command", 10,
            std::bind(&MissionExecutorNode::missionCommandCallback, this, std::placeholders::_1));
        
        mission_id_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "execute_mission_id", 10,
            std::bind(&MissionExecutorNode::missionIdCallback, this, std::placeholders::_1));
        
        navigation_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "navigation_status", 10,
            std::bind(&MissionExecutorNode::navigationStatusCallback, this, std::placeholders::_1));
        
        // Publishers
        mission_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mission_path", 10);
        mission_status_pub_ = this->create_publisher<std_msgs::msg::String>("mission_status", 10);
        mission_progress_pub_ = this->create_publisher<std_msgs::msg::String>("mission_progress", 10);
        mission_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("mission_visualization", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("navigation/mission_health", 10);
        
        // Action clients
        follow_path_client_ = rclcpp_action::create_client<tadeo_ecar_interfaces::action::FollowPath>(
            this, "follow_path");
        
        // Service clients
        navigate_client_ = this->create_client<tadeo_ecar_interfaces::srv::NavigateToGoal>("navigate_to_pose");
        
        // Timers
        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / execution_frequency_)),
            std::bind(&MissionExecutorNode::executionLoop, this));
        
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&MissionExecutorNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Mission Executor Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("execution_frequency", 5.0);
        this->declare_parameter("mission_storage_path", "/tmp/tadeo_missions/");
        this->declare_parameter("auto_load_missions", true);
        this->declare_parameter("max_mission_attempts", 3);
        this->declare_parameter("waypoint_timeout", 60.0);
        this->declare_parameter("mission_timeout", 1800.0); // 30 minutes
        this->declare_parameter("enable_mission_logging", true);
        this->declare_parameter("retry_failed_waypoints", true);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
        
        execution_frequency_ = this->get_parameter("execution_frequency").as_double();
        mission_storage_path_ = this->get_parameter("mission_storage_path").as_string();
        auto_load_missions_ = this->get_parameter("auto_load_missions").as_bool();
        max_mission_attempts_ = this->get_parameter("max_mission_attempts").as_int();
        waypoint_timeout_ = this->get_parameter("waypoint_timeout").as_double();
        mission_timeout_ = this->get_parameter("mission_timeout").as_double();
        enable_mission_logging_ = this->get_parameter("enable_mission_logging").as_bool();
        retry_failed_waypoints_ = this->get_parameter("retry_failed_waypoints").as_bool();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
    }
    
    void initializeExecutor()
    {
        // Initialize mission state
        current_mission_state_ = MissionExecutionState::IDLE;
        current_mission_id_ = -1;
        current_waypoint_index_ = 0;
        mission_attempts_ = 0;
        
        // Create mission storage directory
        try {
            std::filesystem::create_directories(mission_storage_path_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create mission storage directory: %s", e.what());
        }
        
        // Load existing missions if enabled
        if (auto_load_missions_) {
            loadStoredMissions();
        }
        
        RCLCPP_INFO(this->get_logger(), "Mission executor initialized with %zu stored missions", 
                    stored_missions_.size());
    }
    
    void missionCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        
        if (command == "START") {
            if (current_mission_.isEmpty()) {
                RCLCPP_WARN(this->get_logger(), "No mission loaded to start");
                return;
            }
            startMissionExecution();
        } else if (command == "PAUSE") {
            pauseMissionExecution();
        } else if (command == "RESUME") {
            resumeMissionExecution();
        } else if (command == "STOP") {
            stopMissionExecution();
        } else if (command == "ABORT") {
            abortMissionExecution();
        } else if (command.substr(0, 4) == "LOAD") {
            // Format: "LOAD:mission_id"
            size_t colon_pos = command.find(':');
            if (colon_pos != std::string::npos) {
                try {
                    int mission_id = std::stoi(command.substr(colon_pos + 1));
                    loadMission(mission_id);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid mission ID in command: %s", command.c_str());
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown mission command: %s", command.c_str());
        }
    }
    
    void missionIdCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        loadMission(msg->data);
    }
    
    void navigationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string status = msg->data;
        
        // Parse combined status (state|mode)
        size_t separator = status.find('|');
        if (separator != std::string::npos) {
            std::string nav_state = status.substr(0, separator);
            std::string nav_mode = status.substr(separator + 1);
            
            // Update mission execution based on navigation status
            if (current_mission_state_ == MissionExecutionState::EXECUTING) {
                if (nav_state == "WAYPOINT_REACHED") {
                    handleWaypointReached();
                } else if (nav_state == "MISSION_FAILED" || nav_state == "EMERGENCY_STOP") {
                    handleMissionFailure();
                }
            }
        }
    }
    
    void executionLoop()
    {
        updateMissionProgress();
        checkMissionTimeouts();
        publishMissionStatus();
        publishMissionProgress();
        publishMissionVisualization();
        
        // Execute mission state machine
        switch (current_mission_state_) {
            case MissionExecutionState::EXECUTING:
                executeMissionStep();
                break;
            
            case MissionExecutionState::PAUSED:
                // Mission is paused, no action needed
                break;
            
            case MissionExecutionState::FAILED:
                handleMissionRetry();
                break;
            
            default:
                // No action needed for other states
                break;
        }
    }
    
    void loadMission(int mission_id)
    {
        if (stored_missions_.find(mission_id) == stored_missions_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Mission ID %d not found", mission_id);
            return;
        }
        
        if (current_mission_state_ == MissionExecutionState::EXECUTING) {
            RCLCPP_WARN(this->get_logger(), "Cannot load mission while another is executing");
            return;
        }
        
        current_mission_ = stored_missions_[mission_id];
        current_mission_id_ = mission_id;
        current_waypoint_index_ = 0;
        mission_attempts_ = 0;
        
        updateMissionState(MissionExecutionState::LOADED);
        
        RCLCPP_INFO(this->get_logger(), "Mission %d loaded: %s (%zu waypoints)",
                    mission_id, current_mission_.name.c_str(), current_mission_.size());
    }
    
    void startMissionExecution()
    {
        if (current_mission_.isEmpty()) {
            RCLCPP_ERROR(this->get_logger(), "No mission loaded");
            return;
        }
        
        if (current_mission_state_ == MissionExecutionState::EXECUTING) {
            RCLCPP_WARN(this->get_logger(), "Mission already executing");
            return;
        }
        
        current_waypoint_index_ = 0;
        mission_start_time_ = this->now();
        current_mission_.start_time = mission_start_time_;
        
        updateMissionState(MissionExecutionState::EXECUTING);
        
        // Publish mission path
        publishMissionPath();
        
        // Start navigation to first waypoint
        navigateToCurrentWaypoint();
        
        RCLCPP_INFO(this->get_logger(), "Mission execution started: %s", current_mission_.name.c_str());
    }
    
    void pauseMissionExecution()
    {
        if (current_mission_state_ != MissionExecutionState::EXECUTING) {
            RCLCPP_WARN(this->get_logger(), "No mission executing to pause");
            return;
        }
        
        updateMissionState(MissionExecutionState::PAUSED);
        
        // Cancel current navigation
        cancelCurrentNavigation();
        
        RCLCPP_INFO(this->get_logger(), "Mission execution paused");
    }
    
    void resumeMissionExecution()
    {
        if (current_mission_state_ != MissionExecutionState::PAUSED) {
            RCLCPP_WARN(this->get_logger(), "No paused mission to resume");
            return;
        }
        
        updateMissionState(MissionExecutionState::EXECUTING);
        
        // Resume navigation to current waypoint
        navigateToCurrentWaypoint();
        
        RCLCPP_INFO(this->get_logger(), "Mission execution resumed");
    }
    
    void stopMissionExecution()
    {
        if (current_mission_state_ == MissionExecutionState::IDLE) {
            RCLCPP_WARN(this->get_logger(), "No mission to stop");
            return;
        }
        
        updateMissionState(MissionExecutionState::STOPPED);
        
        // Cancel current navigation
        cancelCurrentNavigation();
        
        // Log mission completion
        if (enable_mission_logging_) {
            logMissionExecution("STOPPED");
        }
        
        RCLCPP_INFO(this->get_logger(), "Mission execution stopped");
    }
    
    void abortMissionExecution()
    {
        updateMissionState(MissionExecutionState::FAILED);
        
        // Cancel current navigation
        cancelCurrentNavigation();
        
        // Log mission failure
        if (enable_mission_logging_) {
            logMissionExecution("ABORTED");
        }
        
        RCLCPP_ERROR(this->get_logger(), "Mission execution aborted");
    }
    
    void executeMissionStep()
    {
        if (current_mission_.isEmpty() || current_waypoint_index_ >= current_mission_.size()) {
            // Mission completed
            completeMissionExecution();
            return;
        }
        
        // Check if current waypoint navigation is in progress
        // This is handled by navigation status callbacks
    }
    
    void navigateToCurrentWaypoint()
    {
        if (current_waypoint_index_ >= current_mission_.size()) {
            completeMissionExecution();
            return;
        }
        
        const Waypoint& waypoint = current_mission_.waypoints[current_waypoint_index_];
        
        // Use action client for path following or service for single waypoint
        if (follow_path_client_->wait_for_action_server(std::chrono::seconds(2))) {
            // Create path with remaining waypoints
            std::vector<Waypoint> remaining_waypoints(
                current_mission_.waypoints.begin() + current_waypoint_index_,
                current_mission_.waypoints.end());
            
            auto goal_msg = tadeo_ecar_interfaces::action::FollowPath::Goal();
            goal_msg.path = waypointsToPath(remaining_waypoints, map_frame_);
            
            auto send_goal_options = rclcpp_action::Client<tadeo_ecar_interfaces::action::FollowPath>::SendGoalOptions();
            send_goal_options.result_callback = 
                std::bind(&MissionExecutorNode::followPathResultCallback, this, std::placeholders::_1);
            
            follow_path_goal_handle_ = follow_path_client_->async_send_goal(goal_msg, send_goal_options);
            
        } else if (navigate_client_->wait_for_service(std::chrono::seconds(2))) {
            // Use navigation service for single waypoint
            auto request = std::make_shared<tadeo_ecar_interfaces::srv::NavigateToGoal::Request>();
            request->goal = toRosPose(waypoint.pose, map_frame_);
            
            auto future = navigate_client_->async_send_request(request);
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation services not available");
            handleMissionFailure();
        }
        
        waypoint_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Navigating to waypoint %d: %s", 
                    current_waypoint_index_, waypoint.name.c_str());
    }
    
    void handleWaypointReached()
    {
        if (current_waypoint_index_ < current_mission_.size()) {
            const Waypoint& waypoint = current_mission_.waypoints[current_waypoint_index_];
            
            RCLCPP_INFO(this->get_logger(), "Waypoint %d reached: %s", 
                        current_waypoint_index_, waypoint.name.c_str());
            
            // Execute waypoint-specific actions if any
            executeWaypointActions(waypoint);
            
            // Move to next waypoint
            current_waypoint_index_++;
            
            if (current_waypoint_index_ >= current_mission_.size()) {
                // Check if mission should loop
                if (current_mission_.loop_mission) {
                    current_waypoint_index_ = 0;
                    navigateToCurrentWaypoint();
                } else {
                    completeMissionExecution();
                }
            } else {
                navigateToCurrentWaypoint();
            }
        }
    }
    
    void executeWaypointActions(const Waypoint& waypoint)
    {
        // Execute custom actions based on waypoint metadata
        for (const auto& [key, value] : waypoint.metadata) {
            if (key == "wait_time") {
                double wait_time = std::stod(value);
                RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds at waypoint", wait_time);
                // In a real implementation, this would be handled asynchronously
            } else if (key == "action") {
                RCLCPP_INFO(this->get_logger(), "Executing action: %s", value.c_str());
                // Execute custom action
            }
        }
    }
    
    void handleMissionFailure()
    {
        RCLCPP_ERROR(this->get_logger(), "Mission failure detected");
        
        if (retry_failed_waypoints_ && mission_attempts_ < max_mission_attempts_) {
            mission_attempts_++;
            RCLCPP_INFO(this->get_logger(), "Retrying mission (attempt %d/%d)", 
                        mission_attempts_, max_mission_attempts_);
            
            // Retry from current waypoint
            navigateToCurrentWaypoint();
        } else {
            updateMissionState(MissionExecutionState::FAILED);
            
            if (enable_mission_logging_) {
                logMissionExecution("FAILED");
            }
        }
    }
    
    void handleMissionRetry()
    {
        // Check if retry timeout has passed
        static auto last_retry_time = this->now();
        auto current_time = this->now();
        
        if ((current_time - last_retry_time).seconds() > 30.0) { // 30 second retry delay
            if (mission_attempts_ < max_mission_attempts_) {
                mission_attempts_++;
                updateMissionState(MissionExecutionState::EXECUTING);
                navigateToCurrentWaypoint();
                last_retry_time = current_time;
                
                RCLCPP_INFO(this->get_logger(), "Retrying failed mission (attempt %d/%d)",
                            mission_attempts_, max_mission_attempts_);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Mission failed after %d attempts", max_mission_attempts_);
                updateMissionState(MissionExecutionState::FAILED);
            }
        }
    }
    
    void completeMissionExecution()
    {
        updateMissionState(MissionExecutionState::COMPLETED);
        
        current_mission_.end_time = this->now();
        current_mission_.status = "COMPLETED";
        
        // Log mission completion
        if (enable_mission_logging_) {
            logMissionExecution("COMPLETED");
        }
        
        RCLCPP_INFO(this->get_logger(), "Mission completed successfully: %s", current_mission_.name.c_str());
    }
    
    void cancelCurrentNavigation()
    {
        // Cancel active navigation
        if (follow_path_goal_handle_) {
            follow_path_client_->async_cancel_goal(follow_path_goal_handle_);
            follow_path_goal_handle_.reset();
        }
    }
    
    void updateMissionProgress()
    {
        if (current_mission_.isEmpty()) return;
        
        // Calculate progress percentage
        double progress = static_cast<double>(current_waypoint_index_) / current_mission_.size() * 100.0;
        mission_progress_percentage_ = std::min(100.0, progress);
    }
    
    void checkMissionTimeouts()
    {
        if (current_mission_state_ != MissionExecutionState::EXECUTING) return;
        
        auto current_time = this->now();
        
        // Check mission timeout
        double mission_elapsed = (current_time - mission_start_time_).seconds();
        if (mission_elapsed > mission_timeout_) {
            RCLCPP_ERROR(this->get_logger(), "Mission timeout after %.1f seconds", mission_elapsed);
            handleMissionFailure();
            return;
        }
        
        // Check waypoint timeout
        double waypoint_elapsed = (current_time - waypoint_start_time_).seconds();
        if (waypoint_elapsed > waypoint_timeout_) {
            RCLCPP_WARN(this->get_logger(), "Waypoint timeout after %.1f seconds", waypoint_elapsed);
            handleMissionFailure();
        }
    }
    
    void updateMissionState(MissionExecutionState new_state)
    {
        if (current_mission_state_ != new_state) {
            RCLCPP_INFO(this->get_logger(), "Mission state: %s -> %s",
                        missionStateToString(current_mission_state_).c_str(),
                        missionStateToString(new_state).c_str());
            current_mission_state_ = new_state;
        }
    }
    
    void publishMissionPath()
    {
        if (current_mission_.isEmpty()) return;
        
        auto path_msg = waypointsToPath(current_mission_.waypoints, map_frame_);
        mission_path_pub_->publish(path_msg);
    }
    
    void publishMissionStatus()
    {
        std_msgs::msg::String status_msg;
        status_msg.data = missionStateToString(current_mission_state_);
        mission_status_pub_->publish(status_msg);
    }
    
    void publishMissionProgress()
    {
        std_msgs::msg::String progress_msg;
        progress_msg.data = "Mission: " + current_mission_.name + 
                           " | Progress: " + std::to_string(static_cast<int>(mission_progress_percentage_)) + "%" +
                           " | Waypoint: " + std::to_string(current_waypoint_index_) + "/" + std::to_string(current_mission_.size()) +
                           " | Attempts: " + std::to_string(mission_attempts_);
        mission_progress_pub_->publish(progress_msg);
    }
    
    void publishMissionVisualization()
    {
        if (current_mission_.isEmpty()) return;
        
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Mission waypoints
        for (size_t i = 0; i < current_mission_.waypoints.size(); ++i) {
            const auto& waypoint = current_mission_.waypoints[i];
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp = this->now();
            marker.ns = "mission_waypoints";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = waypoint.pose.x;
            marker.pose.position.y = waypoint.pose.y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.6;
            marker.scale.y = 0.6;
            marker.scale.z = 0.2;
            
            // Color coding
            if (i < current_waypoint_index_) {
                // Completed - green
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
            } else if (i == current_waypoint_index_) {
                // Current - yellow
                marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
            } else {
                // Future - blue
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
            }
            marker.color.a = 0.8;
            
            marker_array.markers.push_back(marker);
            
            // Waypoint labels
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = marker.header;
            text_marker.ns = "waypoint_labels";
            text_marker.id = i;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose = marker.pose;
            text_marker.pose.position.z = 0.5;
            
            text_marker.text = waypoint.name;
            text_marker.scale.z = 0.3;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            marker_array.markers.push_back(text_marker);
        }
        
        // Mission path
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = map_frame_;
        path_marker.header.stamp = this->now();
        path_marker.ns = "mission_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        
        for (const auto& waypoint : current_mission_.waypoints) {
            geometry_msgs::msg::Point p;
            p.x = waypoint.pose.x;
            p.y = waypoint.pose.y;
            p.z = 0.1;
            path_marker.points.push_back(p);
        }
        
        path_marker.scale.x = 0.05;
        path_marker.color.r = 0.5;
        path_marker.color.g = 0.0;
        path_marker.color.b = 0.5;
        path_marker.color.a = 0.8;
        
        marker_array.markers.push_back(path_marker);
        
        mission_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        health_msg.component_name = "mission_executor";
        
        // Determine health status
        switch (current_mission_state_) {
            case MissionExecutionState::IDLE:
                health_msg.status = "IDLE";
                health_msg.error_code = 0;
                health_msg.error_message = "Ready for mission";
                break;
            
            case MissionExecutionState::LOADED:
                health_msg.status = "READY";
                health_msg.error_code = 0;
                health_msg.error_message = "Mission loaded";
                break;
            
            case MissionExecutionState::EXECUTING:
                health_msg.status = "OK";
                health_msg.error_code = 0;
                health_msg.error_message = "Mission executing";
                break;
            
            case MissionExecutionState::PAUSED:
                health_msg.status = "PAUSED";
                health_msg.error_code = 16001;
                health_msg.error_message = "Mission paused";
                break;
            
            case MissionExecutionState::COMPLETED:
                health_msg.status = "SUCCESS";
                health_msg.error_code = 0;
                health_msg.error_message = "Mission completed";
                break;
            
            case MissionExecutionState::FAILED:
                health_msg.status = "ERROR";
                health_msg.error_code = 16002;
                health_msg.error_message = "Mission failed";
                break;
            
            case MissionExecutionState::STOPPED:
                health_msg.status = "STOPPED";
                health_msg.error_code = 16003;
                health_msg.error_message = "Mission stopped";
                break;
            
            default:
                health_msg.status = "UNKNOWN";
                health_msg.error_code = 16999;
                health_msg.error_message = "Unknown state";
                break;
        }
        
        health_msg.cpu_usage = 15.0; // Placeholder
        health_msg.memory_usage = 12.0; // Placeholder
        health_msg.temperature = 35.0; // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    void loadStoredMissions()
    {
        try {
            for (const auto& entry : std::filesystem::directory_iterator(mission_storage_path_)) {
                if (entry.path().extension() == ".yaml") {
                    Mission mission = loadMissionFromFile(entry.path().string());
                    if (mission.id != -1) {
                        stored_missions_[mission.id] = mission;
                        RCLCPP_DEBUG(this->get_logger(), "Loaded mission: %s", mission.name.c_str());
                    }
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load stored missions: %s", e.what());
        }
    }
    
    Mission loadMissionFromFile(const std::string& filename)
    {
        Mission mission;
        
        try {
            YAML::Node yaml_mission = YAML::LoadFile(filename);
            
            mission.id = yaml_mission["id"].as<int>();
            mission.name = yaml_mission["name"].as<std::string>();
            mission.description = yaml_mission["description"].as<std::string>();
            mission.loop_mission = yaml_mission["loop_mission"].as<bool>();
            mission.max_attempts = yaml_mission["max_attempts"].as<int>();
            
            if (yaml_mission["waypoints"]) {
                for (const auto& wp_node : yaml_mission["waypoints"]) {
                    Waypoint waypoint;
                    waypoint.id = wp_node["id"].as<int>();
                    waypoint.name = wp_node["name"].as<std::string>();
                    waypoint.pose.x = wp_node["pose"]["x"].as<double>();
                    waypoint.pose.y = wp_node["pose"]["y"].as<double>();
                    waypoint.pose.theta = wp_node["pose"]["theta"].as<double>();
                    waypoint.tolerance = wp_node["tolerance"].as<double>();
                    waypoint.max_velocity = wp_node["max_velocity"].as<double>();
                    waypoint.is_mandatory = wp_node["is_mandatory"].as<bool>();
                    
                    mission.waypoints.push_back(waypoint);
                }
            }
            
            mission.calculateDistance();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mission from %s: %s", filename.c_str(), e.what());
            mission.id = -1; // Mark as invalid
        }
        
        return mission;
    }
    
    void logMissionExecution(const std::string& result)
    {
        try {
            std::string log_filename = mission_storage_path_ + "mission_log_" + 
                                       std::to_string(current_mission_id_) + "_" +
                                       std::to_string(std::time(nullptr)) + ".yaml";
            
            YAML::Node log_node;
            log_node["mission_id"] = current_mission_id_;
            log_node["mission_name"] = current_mission_.name;
            log_node["result"] = result;
            log_node["start_time"] = mission_start_time_.seconds();
            log_node["end_time"] = this->now().seconds();
            log_node["waypoints_completed"] = current_waypoint_index_;
            log_node["total_waypoints"] = current_mission_.size();
            log_node["attempts"] = mission_attempts_;
            
            std::ofstream log_file(log_filename);
            log_file << log_node;
            log_file.close();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to log mission execution: %s", e.what());
        }
    }
    
    void followPathResultCallback(
        const rclcpp_action::Client<tadeo_ecar_interfaces::action::FollowPath>::GoalHandle::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Path following succeeded");
                completeMissionExecution();
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Path following aborted");
                handleMissionFailure();
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Path following cancelled");
                updateMissionState(MissionExecutionState::STOPPED);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Path following failed");
                handleMissionFailure();
                break;
        }
    }
    
    enum class MissionExecutionState {
        IDLE,
        LOADED,
        EXECUTING,
        PAUSED,
        COMPLETED,
        FAILED,
        STOPPED
    };
    
    std::string missionStateToString(MissionExecutionState state) {
        switch (state) {
            case MissionExecutionState::IDLE: return "IDLE";
            case MissionExecutionState::LOADED: return "LOADED";
            case MissionExecutionState::EXECUTING: return "EXECUTING";
            case MissionExecutionState::PAUSED: return "PAUSED";
            case MissionExecutionState::COMPLETED: return "COMPLETED";
            case MissionExecutionState::FAILED: return "FAILED";
            case MissionExecutionState::STOPPED: return "STOPPED";
            default: return "UNKNOWN";
        }
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mission_id_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_progress_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mission_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp_action::Client<tadeo_ecar_interfaces::action::FollowPath>::SharedPtr follow_path_client_;
    rclcpp_action::Client<tadeo_ecar_interfaces::action::FollowPath>::GoalHandle::SharedPtr follow_path_goal_handle_;
    
    rclcpp::Client<tadeo_ecar_interfaces::srv::NavigateToGoal>::SharedPtr navigate_client_;
    
    rclcpp::TimerInterface::SharedPtr execution_timer_;
    rclcpp::TimerInterface::SharedPtr health_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double execution_frequency_;
    std::string mission_storage_path_;
    bool auto_load_missions_;
    int max_mission_attempts_;
    double waypoint_timeout_;
    double mission_timeout_;
    bool enable_mission_logging_;
    bool retry_failed_waypoints_;
    std::string base_frame_;
    std::string map_frame_;
    
    // Mission execution state
    MissionExecutionState current_mission_state_;
    Mission current_mission_;
    std::map<int, Mission> stored_missions_;
    int current_mission_id_;
    int current_waypoint_index_;
    int mission_attempts_;
    double mission_progress_percentage_;
    
    // Timing
    rclcpp::Time mission_start_time_;
    rclcpp::Time waypoint_start_time_;
};

} // namespace tadeo_ecar_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_navigation::MissionExecutorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}