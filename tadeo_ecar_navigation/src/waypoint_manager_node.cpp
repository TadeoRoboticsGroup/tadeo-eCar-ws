#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
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

class WaypointManagerNode : public rclcpp::Node
{
public:
    WaypointManagerNode() : Node("waypoint_manager_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeManager();
        
        // Subscribers
        waypoint_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "waypoint_command", 10,
            std::bind(&WaypointManagerNode::waypointCommandCallback, this, std::placeholders::_1));
        
        new_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "new_waypoint", 10,
            std::bind(&WaypointManagerNode::newWaypointCallback, this, std::placeholders::_1));
        
        clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10,
            std::bind(&WaypointManagerNode::clickedPointCallback, this, std::placeholders::_1));
        
        // Publishers
        waypoint_list_pub_ = this->create_publisher<nav_msgs::msg::Path>("waypoint_list", 10);
        current_mission_pub_ = this->create_publisher<nav_msgs::msg::Path>("current_mission", 10);
        waypoint_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_markers", 10);
        waypoint_status_pub_ = this->create_publisher<std_msgs::msg::String>("waypoint_status", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("navigation/waypoint_health", 10);
        
        // Services
        add_waypoint_service_ = this->create_service<tadeo_ecar_interfaces::srv::NavigateToGoal>(
            "add_waypoint_service",
            std::bind(&WaypointManagerNode::addWaypointService, this, std::placeholders::_1, std::placeholders::_2));
        
        // Timers
        manager_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency_)),
            std::bind(&WaypointManagerNode::managerLoop, this));
        
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&WaypointManagerNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Waypoint Manager Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("update_frequency", 10.0);
        this->declare_parameter("waypoint_storage_path", "/tmp/tadeo_waypoints/");
        this->declare_parameter("auto_save_interval", 30.0);
        this->declare_parameter("max_waypoints_per_mission", 50);
        this->declare_parameter("default_waypoint_tolerance", 0.3);
        this->declare_parameter("default_max_velocity", 1.0);
        this->declare_parameter("enable_interactive_markers", true);
        this->declare_parameter("auto_connect_waypoints", true);
        this->declare_parameter("waypoint_height_offset", 0.2);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
        
        update_frequency_ = this->get_parameter("update_frequency").as_double();
        waypoint_storage_path_ = this->get_parameter("waypoint_storage_path").as_string();
        auto_save_interval_ = this->get_parameter("auto_save_interval").as_double();
        max_waypoints_per_mission_ = this->get_parameter("max_waypoints_per_mission").as_int();
        default_waypoint_tolerance_ = this->get_parameter("default_waypoint_tolerance").as_double();
        default_max_velocity_ = this->get_parameter("default_max_velocity").as_double();
        enable_interactive_markers_ = this->get_parameter("enable_interactive_markers").as_bool();
        auto_connect_waypoints_ = this->get_parameter("auto_connect_waypoints").as_bool();
        waypoint_height_offset_ = this->get_parameter("waypoint_height_offset").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
    }
    
    void initializeManager()
    {
        // Initialize state
        current_mission_id_ = 0;
        next_waypoint_id_ = 0;
        editing_mode_ = false;
        
        // Create storage directory
        try {
            std::filesystem::create_directories(waypoint_storage_path_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create waypoint storage directory: %s", e.what());
        }
        
        // Load existing waypoints
        loadStoredWaypoints();
        
        // Initialize auto-save timer
        last_save_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Waypoint manager initialized with %zu stored waypoints", 
                    stored_waypoints_.size());
    }
    
    void waypointCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        
        if (command == "START_EDITING") {
            startEditingMode();
        } else if (command == "STOP_EDITING") {
            stopEditingMode();
        } else if (command == "CLEAR_ALL") {
            clearAllWaypoints();
        } else if (command == "SAVE_MISSION") {
            saveMissionToFile();
        } else if (command == "NEW_MISSION") {
            createNewMission();
        } else if (command == "UNDO_LAST") {
            undoLastWaypoint();
        } else if (command == "OPTIMIZE_PATH") {
            optimizeWaypointPath();
        } else if (command.substr(0, 4) == "LOAD") {
            // Format: "LOAD:mission_id"
            size_t colon_pos = command.find(':');
            if (colon_pos != std::string::npos) {
                try {
                    int mission_id = std::stoi(command.substr(colon_pos + 1));
                    loadMissionFromFile(mission_id);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid mission ID in command: %s", command.c_str());
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown waypoint command: %s", command.c_str());
        }
    }
    
    void newWaypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!editing_mode_) {
            RCLCPP_WARN(this->get_logger(), "Not in editing mode, waypoint ignored");
            return;
        }
        
        addWaypoint(fromRosPose(*msg));
    }
    
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!editing_mode_) return;
        
        // Convert clicked point to waypoint
        Pose2D pose;
        pose.x = msg->point.x;
        pose.y = msg->point.y;
        pose.theta = 0.0; // Default orientation
        
        addWaypoint(pose);
    }
    
    void managerLoop()
    {
        // Auto-save waypoints if needed
        auto current_time = this->now();
        if ((current_time - last_save_time_).seconds() > auto_save_interval_) {
            autoSaveWaypoints();
            last_save_time_ = current_time;
        }
        
        // Update visualizations
        publishWaypointVisualization();
        publishWaypointList();
        publishCurrentMission();
        publishWaypointStatus();
    }
    
    void startEditingMode()
    {
        editing_mode_ = true;
        RCLCPP_INFO(this->get_logger(), "Started waypoint editing mode");
        
        // Clear current editing waypoints to start fresh
        current_editing_waypoints_.clear();
        
        publishWaypointStatus();
    }
    
    void stopEditingMode()
    {
        editing_mode_ = false;
        
        // Save current editing waypoints to mission
        if (!current_editing_waypoints_.empty()) {
            current_mission_.waypoints = current_editing_waypoints_;
            current_mission_.id = current_mission_id_;
            current_mission_.name = "Mission_" + std::to_string(current_mission_id_);
            current_mission_.calculateDistance();
            
            RCLCPP_INFO(this->get_logger(), "Stopped editing mode. Mission created with %zu waypoints",
                        current_editing_waypoints_.size());
        }
        
        publishWaypointStatus();
    }
    
    void addWaypoint(const Pose2D& pose)
    {
        if (current_editing_waypoints_.size() >= static_cast<size_t>(max_waypoints_per_mission_)) {
            RCLCPP_WARN(this->get_logger(), "Maximum waypoints per mission reached (%d)", max_waypoints_per_mission_);
            return;
        }
        
        Waypoint waypoint;
        waypoint.id = next_waypoint_id_++;
        waypoint.name = "WP_" + std::to_string(waypoint.id);
        waypoint.pose = pose;
        waypoint.tolerance = default_waypoint_tolerance_;
        waypoint.max_velocity = default_max_velocity_;
        waypoint.is_mandatory = true;
        
        current_editing_waypoints_.push_back(waypoint);
        
        // Store in global waypoint list
        stored_waypoints_[waypoint.id] = waypoint;
        
        RCLCPP_INFO(this->get_logger(), "Added waypoint %d: %s at (%.2f, %.2f, %.2f)",
                    waypoint.id, waypoint.name.c_str(), pose.x, pose.y, pose.theta);
        
        publishWaypointVisualization();
    }
    
    void clearAllWaypoints()
    {
        current_editing_waypoints_.clear();
        current_mission_.waypoints.clear();
        
        RCLCPP_INFO(this->get_logger(), "All waypoints cleared");
        publishWaypointVisualization();
    }
    
    void createNewMission()
    {
        current_mission_id_++;
        current_editing_waypoints_.clear();
        current_mission_ = Mission();
        current_mission_.id = current_mission_id_;
        
        RCLCPP_INFO(this->get_logger(), "Created new mission %d", current_mission_id_);
    }
    
    void undoLastWaypoint()
    {
        if (!current_editing_waypoints_.empty()) {
            auto last_waypoint = current_editing_waypoints_.back();
            current_editing_waypoints_.pop_back();
            
            // Remove from stored waypoints
            stored_waypoints_.erase(last_waypoint.id);
            
            RCLCPP_INFO(this->get_logger(), "Removed last waypoint: %s", last_waypoint.name.c_str());
            publishWaypointVisualization();
        }
    }
    
    void optimizeWaypointPath()
    {
        if (current_editing_waypoints_.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Need at least 3 waypoints to optimize");
            return;
        }
        
        // Simple optimization: remove waypoints that are too close together
        std::vector<Waypoint> optimized_waypoints;
        const double min_distance = 0.5; // Minimum distance between waypoints
        
        optimized_waypoints.push_back(current_editing_waypoints_[0]); // Always keep first
        
        for (size_t i = 1; i < current_editing_waypoints_.size(); ++i) {
            double distance = current_editing_waypoints_[i].pose.distance(optimized_waypoints.back().pose);
            
            if (distance > min_distance || i == current_editing_waypoints_.size() - 1) {
                optimized_waypoints.push_back(current_editing_waypoints_[i]);
            }
        }
        
        size_t removed_count = current_editing_waypoints_.size() - optimized_waypoints.size();
        current_editing_waypoints_ = optimized_waypoints;
        
        RCLCPP_INFO(this->get_logger(), "Path optimized: removed %zu waypoints", removed_count);
        publishWaypointVisualization();
    }
    
    void saveMissionToFile()
    {
        if (current_mission_.waypoints.empty()) {
            RCLCPP_WARN(this->get_logger(), "No mission to save");
            return;
        }
        
        try {
            std::string filename = waypoint_storage_path_ + "mission_" + 
                                   std::to_string(current_mission_.id) + ".yaml";
            
            YAML::Node mission_node;
            mission_node["id"] = current_mission_.id;
            mission_node["name"] = current_mission_.name;
            mission_node["description"] = current_mission_.description;
            mission_node["loop_mission"] = current_mission_.loop_mission;
            mission_node["max_attempts"] = current_mission_.max_attempts;
            mission_node["total_distance"] = current_mission_.total_distance;
            
            for (size_t i = 0; i < current_mission_.waypoints.size(); ++i) {
                const auto& wp = current_mission_.waypoints[i];
                
                YAML::Node wp_node;
                wp_node["id"] = wp.id;
                wp_node["name"] = wp.name;
                wp_node["pose"]["x"] = wp.pose.x;
                wp_node["pose"]["y"] = wp.pose.y;
                wp_node["pose"]["theta"] = wp.pose.theta;
                wp_node["tolerance"] = wp.tolerance;
                wp_node["max_velocity"] = wp.max_velocity;
                wp_node["is_mandatory"] = wp.is_mandatory;
                
                mission_node["waypoints"].push_back(wp_node);
            }
            
            std::ofstream file(filename);
            file << mission_node;
            file.close();
            
            RCLCPP_INFO(this->get_logger(), "Mission saved to: %s", filename.c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save mission: %s", e.what());
        }
    }
    
    void loadMissionFromFile(int mission_id)
    {
        try {
            std::string filename = waypoint_storage_path_ + "mission_" + 
                                   std::to_string(mission_id) + ".yaml";
            
            if (!std::filesystem::exists(filename)) {
                RCLCPP_ERROR(this->get_logger(), "Mission file not found: %s", filename.c_str());
                return;
            }
            
            YAML::Node mission_node = YAML::LoadFile(filename);
            
            current_mission_.id = mission_node["id"].as<int>();
            current_mission_.name = mission_node["name"].as<std::string>();
            current_mission_.description = mission_node["description"].as<std::string>();
            current_mission_.loop_mission = mission_node["loop_mission"].as<bool>();
            current_mission_.max_attempts = mission_node["max_attempts"].as<int>();
            current_mission_.total_distance = mission_node["total_distance"].as<double>();
            
            current_mission_.waypoints.clear();
            current_editing_waypoints_.clear();
            
            if (mission_node["waypoints"]) {
                for (const auto& wp_node : mission_node["waypoints"]) {
                    Waypoint waypoint;
                    waypoint.id = wp_node["id"].as<int>();
                    waypoint.name = wp_node["name"].as<std::string>();
                    waypoint.pose.x = wp_node["pose"]["x"].as<double>();
                    waypoint.pose.y = wp_node["pose"]["y"].as<double>();
                    waypoint.pose.theta = wp_node["pose"]["theta"].as<double>();
                    waypoint.tolerance = wp_node["tolerance"].as<double>();
                    waypoint.max_velocity = wp_node["max_velocity"].as<double>();
                    waypoint.is_mandatory = wp_node["is_mandatory"].as<bool>();
                    
                    current_mission_.waypoints.push_back(waypoint);
                    current_editing_waypoints_.push_back(waypoint);
                    stored_waypoints_[waypoint.id] = waypoint;
                }
            }
            
            current_mission_id_ = mission_id;
            
            RCLCPP_INFO(this->get_logger(), "Mission loaded: %s (%zu waypoints)", 
                        current_mission_.name.c_str(), current_mission_.waypoints.size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mission: %s", e.what());
        }
    }
    
    void autoSaveWaypoints()
    {
        if (current_editing_waypoints_.empty()) return;
        
        try {
            std::string filename = waypoint_storage_path_ + "autosave_waypoints.yaml";
            
            YAML::Node waypoints_node;
            waypoints_node["timestamp"] = this->now().seconds();
            waypoints_node["count"] = current_editing_waypoints_.size();
            
            for (size_t i = 0; i < current_editing_waypoints_.size(); ++i) {
                const auto& wp = current_editing_waypoints_[i];
                
                YAML::Node wp_node;
                wp_node["id"] = wp.id;
                wp_node["name"] = wp.name;
                wp_node["pose"]["x"] = wp.pose.x;
                wp_node["pose"]["y"] = wp.pose.y;
                wp_node["pose"]["theta"] = wp.pose.theta;
                wp_node["tolerance"] = wp.tolerance;
                wp_node["max_velocity"] = wp.max_velocity;
                
                waypoints_node["waypoints"].push_back(wp_node);
            }
            
            std::ofstream file(filename);
            file << waypoints_node;
            file.close();
            
            RCLCPP_DEBUG(this->get_logger(), "Auto-saved %zu waypoints", current_editing_waypoints_.size());
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Auto-save failed: %s", e.what());
        }
    }
    
    void loadStoredWaypoints()
    {
        try {
            std::string filename = waypoint_storage_path_ + "autosave_waypoints.yaml";
            
            if (std::filesystem::exists(filename)) {
                YAML::Node waypoints_node = YAML::LoadFile(filename);
                
                if (waypoints_node["waypoints"]) {
                    for (const auto& wp_node : waypoints_node["waypoints"]) {
                        Waypoint waypoint;
                        waypoint.id = wp_node["id"].as<int>();
                        waypoint.name = wp_node["name"].as<std::string>();
                        waypoint.pose.x = wp_node["pose"]["x"].as<double>();
                        waypoint.pose.y = wp_node["pose"]["y"].as<double>();
                        waypoint.pose.theta = wp_node["pose"]["theta"].as<double>();
                        waypoint.tolerance = wp_node["tolerance"].as<double>();
                        waypoint.max_velocity = wp_node["max_velocity"].as<double>();
                        
                        stored_waypoints_[waypoint.id] = waypoint;
                        
                        if (waypoint.id >= next_waypoint_id_) {
                            next_waypoint_id_ = waypoint.id + 1;
                        }
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "Loaded %zu stored waypoints", stored_waypoints_.size());
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load stored waypoints: %s", e.what());
        }
    }
    
    void publishWaypointList()
    {
        if (stored_waypoints_.empty()) return;
        
        std::vector<Waypoint> all_waypoints;
        for (const auto& [id, waypoint] : stored_waypoints_) {
            all_waypoints.push_back(waypoint);
        }
        
        auto path_msg = waypointsToPath(all_waypoints, map_frame_);
        waypoint_list_pub_->publish(path_msg);
    }
    
    void publishCurrentMission()
    {
        if (current_editing_waypoints_.empty()) return;
        
        auto path_msg = waypointsToPath(current_editing_waypoints_, map_frame_);
        current_mission_pub_->publish(path_msg);
    }
    
    void publishWaypointStatus()
    {
        std_msgs::msg::String status_msg;
        
        if (editing_mode_) {
            status_msg.data = "EDITING | Waypoints: " + std::to_string(current_editing_waypoints_.size()) +
                             " | Mission: " + std::to_string(current_mission_id_);
        } else {
            status_msg.data = "IDLE | Stored: " + std::to_string(stored_waypoints_.size()) +
                             " | Mission: " + current_mission_.name;
        }
        
        waypoint_status_pub_->publish(status_msg);
    }
    
    void publishWaypointVisualization()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Clear previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // Current editing waypoints
        for (size_t i = 0; i < current_editing_waypoints_.size(); ++i) {
            const auto& waypoint = current_editing_waypoints_[i];
            
            // Waypoint marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp = this->now();
            marker.ns = "editing_waypoints";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = waypoint.pose.x;
            marker.pose.position.y = waypoint.pose.y;
            marker.pose.position.z = waypoint_height_offset_;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.4;
            
            // Color coding
            if (editing_mode_) {
                marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0; // Orange
            } else {
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; // Blue
            }
            marker.color.a = 0.8;
            
            marker_array.markers.push_back(marker);
            
            // Waypoint number
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = marker.header;
            text_marker.ns = "waypoint_numbers";
            text_marker.id = i;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose = marker.pose;
            text_marker.pose.position.z += 0.3;
            
            text_marker.text = std::to_string(i + 1);
            text_marker.scale.z = 0.3;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            marker_array.markers.push_back(text_marker);
            
            // Arrow showing orientation
            visualization_msgs::msg::Marker arrow_marker;
            arrow_marker.header = marker.header;
            arrow_marker.ns = "waypoint_orientations";
            arrow_marker.id = i;
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.action = visualization_msgs::msg::Marker::ADD;
            
            arrow_marker.pose.position = marker.pose.position;
            arrow_marker.pose.orientation.x = 0.0;
            arrow_marker.pose.orientation.y = 0.0;
            arrow_marker.pose.orientation.z = sin(waypoint.pose.theta / 2.0);
            arrow_marker.pose.orientation.w = cos(waypoint.pose.theta / 2.0);
            
            arrow_marker.scale.x = 0.6;
            arrow_marker.scale.y = 0.1;
            arrow_marker.scale.z = 0.1;
            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 1.0;
            arrow_marker.color.b = 0.0;
            arrow_marker.color.a = 0.6;
            
            marker_array.markers.push_back(arrow_marker);
        }
        
        // Path connections
        if (current_editing_waypoints_.size() > 1 && auto_connect_waypoints_) {
            visualization_msgs::msg::Marker path_marker;
            path_marker.header.frame_id = map_frame_;
            path_marker.header.stamp = this->now();
            path_marker.ns = "waypoint_path";
            path_marker.id = 0;
            path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::msg::Marker::ADD;
            
            for (const auto& waypoint : current_editing_waypoints_) {
                geometry_msgs::msg::Point p;
                p.x = waypoint.pose.x;
                p.y = waypoint.pose.y;
                p.z = waypoint_height_offset_;
                path_marker.points.push_back(p);
            }
            
            path_marker.scale.x = 0.05;
            path_marker.color.r = 0.5;
            path_marker.color.g = 0.5;
            path_marker.color.b = 0.5;
            path_marker.color.a = 0.7;
            
            marker_array.markers.push_back(path_marker);
        }
        
        waypoint_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        health_msg.component_name = "waypoint_manager";
        
        if (editing_mode_) {
            health_msg.status = "EDITING";
            health_msg.error_code = 17001;
            health_msg.error_message = "Waypoint editing active";
        } else if (current_editing_waypoints_.size() > max_waypoints_per_mission_) {
            health_msg.status = "WARNING";
            health_msg.error_code = 17002;
            health_msg.error_message = "Too many waypoints";
        } else {
            health_msg.status = "OK";
            health_msg.error_code = 0;
            health_msg.error_message = "";
        }
        
        health_msg.cpu_usage = 10.0; // Placeholder
        health_msg.memory_usage = 8.0; // Placeholder
        health_msg.temperature = 30.0; // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    void addWaypointService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::NavigateToGoal::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::NavigateToGoal::Response> response)
    {
        try {
            Pose2D pose = fromRosPose(request->goal);
            
            if (editing_mode_) {
                addWaypoint(pose);
                response->success = true;
                response->message = "Waypoint added to current mission";
            } else {
                response->success = false;
                response->message = "Not in editing mode";
            }
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to add waypoint: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Add waypoint service failed: %s", e.what());
        }
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoint_command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr new_waypoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoint_list_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr current_mission_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_viz_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoint_status_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::Service<tadeo_ecar_interfaces::srv::NavigateToGoal>::SharedPtr add_waypoint_service_;
    
    rclcpp::TimerInterface::SharedPtr manager_timer_;
    rclcpp::TimerInterface::SharedPtr health_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double update_frequency_;
    std::string waypoint_storage_path_;
    double auto_save_interval_;
    int max_waypoints_per_mission_;
    double default_waypoint_tolerance_;
    double default_max_velocity_;
    bool enable_interactive_markers_;
    bool auto_connect_waypoints_;
    double waypoint_height_offset_;
    std::string base_frame_;
    std::string map_frame_;
    
    // Waypoint management state
    std::map<int, Waypoint> stored_waypoints_;
    std::vector<Waypoint> current_editing_waypoints_;
    Mission current_mission_;
    int current_mission_id_;
    int next_waypoint_id_;
    bool editing_mode_;
    
    // Timing
    rclcpp::Time last_save_time_;
};

} // namespace tadeo_ecar_navigation

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_navigation::WaypointManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}