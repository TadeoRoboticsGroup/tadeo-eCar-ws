#ifndef TADEO_ECAR_NAVIGATION_TYPES_HPP_
#define TADEO_ECAR_NAVIGATION_TYPES_HPP_

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>

namespace tadeo_ecar_navigation
{

struct Pose2D
{
    double x, y, theta;
    rclcpp::Time timestamp;
    
    Pose2D() : x(0), y(0), theta(0) {}
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
    
    double distance(const Pose2D& other) const {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }
    
    double angleDifference(const Pose2D& other) const {
        double diff = theta - other.theta;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return abs(diff);
    }
};

struct Waypoint
{
    int id;
    std::string name;
    Pose2D pose;
    double tolerance;
    double max_velocity;
    bool is_mandatory;
    rclcpp::Duration max_wait_time;
    std::map<std::string, std::string> metadata;
    
    Waypoint() : id(-1), tolerance(0.3), max_velocity(1.0), is_mandatory(true), max_wait_time(10, 0) {}
    Waypoint(int id_, const std::string& name_, const Pose2D& pose_) 
        : id(id_), name(name_), pose(pose_), tolerance(0.3), max_velocity(1.0), is_mandatory(true), max_wait_time(10, 0) {}
};

struct Mission
{
    int id;
    std::string name;
    std::string description;
    std::vector<Waypoint> waypoints;
    bool loop_mission;
    int max_attempts;
    rclcpp::Time start_time;
    rclcpp::Time end_time;
    double total_distance;
    std::string status;
    
    Mission() : id(-1), loop_mission(false), max_attempts(3), total_distance(0.0), status("PENDING") {}
    
    bool isEmpty() const { return waypoints.empty(); }
    size_t size() const { return waypoints.size(); }
    
    void calculateDistance() {
        total_distance = 0.0;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            total_distance += waypoints[i].pose.distance(waypoints[i-1].pose);
        }
    }
};

enum class NavigationState
{
    IDLE,
    MISSION_LOADED,
    NAVIGATING,
    WAYPOINT_REACHED,
    MISSION_COMPLETED,
    MISSION_FAILED,
    EMERGENCY_STOP,
    RECOVERY,
    PAUSED
};

enum class NavigationMode
{
    MANUAL,
    AUTONOMOUS,
    REMOTE_CONTROL,
    EMERGENCY
};

struct NavigationStatus
{
    NavigationState state;
    NavigationMode mode;
    int current_mission_id;
    int current_waypoint_index;
    double progress_percentage;
    double distance_to_goal;
    double estimated_time_remaining;
    std::string error_message;
    rclcpp::Time last_update;
    
    NavigationStatus() : 
        state(NavigationState::IDLE), 
        mode(NavigationMode::MANUAL),
        current_mission_id(-1),
        current_waypoint_index(-1),
        progress_percentage(0.0),
        distance_to_goal(0.0),
        estimated_time_remaining(0.0) {}
};

struct NavigationMetrics
{
    double total_distance_traveled;
    double total_time_elapsed;
    double average_velocity;
    int waypoints_reached;
    int missions_completed;
    int navigation_failures;
    double success_rate;
    rclcpp::Time session_start_time;
    
    NavigationMetrics() : 
        total_distance_traveled(0.0),
        total_time_elapsed(0.0),
        average_velocity(0.0),
        waypoints_reached(0),
        missions_completed(0),
        navigation_failures(0),
        success_rate(100.0) {}
    
    void updateMetrics(double distance, double time) {
        total_distance_traveled += distance;
        total_time_elapsed += time;
        if (total_time_elapsed > 0) {
            average_velocity = total_distance_traveled / total_time_elapsed;
        }
    }
    
    void updateSuccessRate() {
        int total_attempts = missions_completed + navigation_failures;
        if (total_attempts > 0) {
            success_rate = (static_cast<double>(missions_completed) / total_attempts) * 100.0;
        }
    }
};

struct RecoveryAction
{
    std::string action_type;
    std::map<std::string, double> parameters;
    int max_attempts;
    double timeout;
    
    RecoveryAction() : max_attempts(3), timeout(30.0) {}
    RecoveryAction(const std::string& type) : action_type(type), max_attempts(3), timeout(30.0) {}
};

struct NavigationConstraints
{
    double max_velocity;
    double max_acceleration;
    double max_angular_velocity;
    double safety_distance;
    double goal_tolerance;
    double path_tolerance;
    double time_tolerance;
    bool allow_reverse;
    
    NavigationConstraints() :
        max_velocity(2.0),
        max_acceleration(1.0),
        max_angular_velocity(1.0),
        safety_distance(0.5),
        goal_tolerance(0.3),
        path_tolerance(1.0),
        time_tolerance(30.0),
        allow_reverse(false) {}
};

struct CostmapLayer
{
    std::string name;
    bool enabled;
    double weight;
    std::map<std::string, std::string> parameters;
    
    CostmapLayer() : enabled(true), weight(1.0) {}
    CostmapLayer(const std::string& name_, bool enabled_ = true, double weight_ = 1.0) 
        : name(name_), enabled(enabled_), weight(weight_) {}
};

struct NavigationConfiguration
{
    NavigationConstraints constraints;
    std::vector<CostmapLayer> costmap_layers;
    std::vector<RecoveryAction> recovery_behaviors;
    std::string global_planner;
    std::string local_planner;
    std::string controller;
    std::map<std::string, double> planner_parameters;
    
    NavigationConfiguration() :
        global_planner("NavfnPlanner"),
        local_planner("DWBLocalPlanner"),
        controller("FollowPath") {}
};

// Utility functions
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline std::string stateToString(NavigationState state) {
    switch (state) {
        case NavigationState::IDLE: return "IDLE";
        case NavigationState::MISSION_LOADED: return "MISSION_LOADED";
        case NavigationState::NAVIGATING: return "NAVIGATING";
        case NavigationState::WAYPOINT_REACHED: return "WAYPOINT_REACHED";
        case NavigationState::MISSION_COMPLETED: return "MISSION_COMPLETED";
        case NavigationState::MISSION_FAILED: return "MISSION_FAILED";
        case NavigationState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case NavigationState::RECOVERY: return "RECOVERY";
        case NavigationState::PAUSED: return "PAUSED";
        default: return "UNKNOWN";
    }
}

inline std::string modeToString(NavigationMode mode) {
    switch (mode) {
        case NavigationMode::MANUAL: return "MANUAL";
        case NavigationMode::AUTONOMOUS: return "AUTONOMOUS";
        case NavigationMode::REMOTE_CONTROL: return "REMOTE_CONTROL";
        case NavigationMode::EMERGENCY: return "EMERGENCY";
        default: return "UNKNOWN";
    }
}

inline geometry_msgs::msg::PoseStamped toRosPose(const Pose2D& pose, const std::string& frame_id) {
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header.frame_id = frame_id;
    ros_pose.pose.position.x = pose.x;
    ros_pose.pose.position.y = pose.y;
    ros_pose.pose.position.z = 0.0;
    
    // Convert theta to quaternion
    ros_pose.pose.orientation.x = 0.0;
    ros_pose.pose.orientation.y = 0.0;
    ros_pose.pose.orientation.z = sin(pose.theta / 2.0);
    ros_pose.pose.orientation.w = cos(pose.theta / 2.0);
    
    return ros_pose;
}

inline Pose2D fromRosPose(const geometry_msgs::msg::PoseStamped& ros_pose) {
    Pose2D pose;
    pose.x = ros_pose.pose.position.x;
    pose.y = ros_pose.pose.position.y;
    
    // Convert quaternion to theta
    double qw = ros_pose.pose.orientation.w;
    double qz = ros_pose.pose.orientation.z;
    pose.theta = 2.0 * atan2(qz, qw);
    
    return pose;
}

inline nav_msgs::msg::Path waypointsToPath(const std::vector<Waypoint>& waypoints, const std::string& frame_id) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = rclcpp::Clock().now();
    
    for (const auto& waypoint : waypoints) {
        geometry_msgs::msg::PoseStamped pose_stamped = toRosPose(waypoint.pose, frame_id);
        pose_stamped.header = path.header;
        path.poses.push_back(pose_stamped);
    }
    
    return path;
}

inline std::vector<Waypoint> pathToWaypoints(const nav_msgs::msg::Path& path) {
    std::vector<Waypoint> waypoints;
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
        Waypoint waypoint;
        waypoint.id = i;
        waypoint.name = "waypoint_" + std::to_string(i);
        waypoint.pose = fromRosPose(path.poses[i]);
        waypoints.push_back(waypoint);
    }
    
    return waypoints;
}

inline double calculatePathDistance(const std::vector<Waypoint>& waypoints) {
    double distance = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        distance += waypoints[i].pose.distance(waypoints[i-1].pose);
    }
    return distance;
}

inline double estimateNavigationTime(const std::vector<Waypoint>& waypoints, double average_velocity) {
    if (average_velocity <= 0) return 0.0;
    double distance = calculatePathDistance(waypoints);
    return distance / average_velocity;
}

} // namespace tadeo_ecar_navigation

#endif // TADEO_ECAR_NAVIGATION_TYPES_HPP_