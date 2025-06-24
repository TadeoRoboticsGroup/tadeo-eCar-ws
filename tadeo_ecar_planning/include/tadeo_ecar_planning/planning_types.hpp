#ifndef TADEO_ECAR_PLANNING_TYPES_HPP_
#define TADEO_ECAR_PLANNING_TYPES_HPP_

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace tadeo_ecar_planning
{

struct Point2D
{
    double x, y;
    
    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
    
    double distance(const Point2D& other) const {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }
};

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

struct Velocity2D
{
    double linear_x, linear_y, angular_z;
    
    Velocity2D() : linear_x(0), linear_y(0), angular_z(0) {}
    Velocity2D(double vx, double vy, double w) : linear_x(vx), linear_y(vy), angular_z(w) {}
};

struct PathPoint
{
    Pose2D pose;
    Velocity2D velocity;
    double curvature;
    double speed;
    double time_from_start;
    
    PathPoint() : curvature(0), speed(0), time_from_start(0) {}
    PathPoint(const Pose2D& p, double s = 0) : pose(p), curvature(0), speed(s), time_from_start(0) {}
};

struct Path
{
    std::vector<PathPoint> points;
    double total_length;
    double total_time;
    rclcpp::Time timestamp;
    std::string frame_id;
    
    Path() : total_length(0), total_time(0), frame_id("map") {}
    
    bool empty() const { return points.empty(); }
    size_t size() const { return points.size(); }
    
    void clear() {
        points.clear();
        total_length = 0;
        total_time = 0;
    }
    
    void calculateLength() {
        total_length = 0;
        for (size_t i = 1; i < points.size(); ++i) {
            total_length += points[i].pose.distance(points[i-1].pose);
        }
    }
    
    PathPoint& operator[](size_t index) { return points[index]; }
    const PathPoint& operator[](size_t index) const { return points[index]; }
};

struct Trajectory
{
    std::vector<PathPoint> points;
    double max_velocity;
    double max_acceleration;
    double max_curvature;
    rclcpp::Time timestamp;
    std::string frame_id;
    
    Trajectory() : max_velocity(2.0), max_acceleration(1.0), max_curvature(1.0), frame_id("map") {}
    
    bool empty() const { return points.empty(); }
    size_t size() const { return points.size(); }
    void clear() { points.clear(); }
    
    PathPoint& operator[](size_t index) { return points[index]; }
    const PathPoint& operator[](size_t index) const { return points[index]; }
};

struct Obstacle
{
    Point2D center;
    double radius;
    Velocity2D velocity;
    bool is_static;
    double confidence;
    rclcpp::Time timestamp;
    
    Obstacle() : radius(0.5), is_static(true), confidence(1.0) {}
    Obstacle(const Point2D& c, double r) : center(c), radius(r), is_static(true), confidence(1.0) {}
    
    bool contains(const Point2D& point) const {
        return center.distance(point) <= radius;
    }
    
    bool intersects(const Point2D& p1, const Point2D& p2) const {
        // Line-circle intersection
        double A = p2.y - p1.y;
        double B = p1.x - p2.x;
        double C = p2.x * p1.y - p1.x * p2.y;
        
        double distance = abs(A * center.x + B * center.y + C) / sqrt(A * A + B * B);
        return distance <= radius;
    }
};

struct PlanningResult
{
    enum Status {
        SUCCESS,
        FAILURE,
        NO_PATH_FOUND,
        START_OCCUPIED,
        GOAL_OCCUPIED,
        TIMEOUT,
        INVALID_START,
        INVALID_GOAL
    };
    
    Status status;
    Path path;
    double planning_time;
    int iterations;
    double cost;
    std::string error_message;
    
    PlanningResult() : status(FAILURE), planning_time(0), iterations(0), cost(0) {}
};

struct PlanningRequest
{
    Pose2D start;
    Pose2D goal;
    std::vector<Obstacle> obstacles;
    double max_planning_time;
    double goal_tolerance;
    std::string planner_type;
    
    PlanningRequest() : max_planning_time(5.0), goal_tolerance(0.2), planner_type("RRT*") {}
};

struct VehicleConstraints
{
    double max_velocity;
    double max_acceleration;
    double max_deceleration;
    double max_angular_velocity;
    double max_angular_acceleration;
    double wheelbase;
    double track_width;
    double min_turning_radius;
    double max_steering_angle;
    
    VehicleConstraints() :
        max_velocity(2.0),
        max_acceleration(1.0),
        max_deceleration(2.0),
        max_angular_velocity(1.0),
        max_angular_acceleration(1.0),
        wheelbase(0.3),
        track_width(0.25),
        min_turning_radius(0.5),
        max_steering_angle(0.7) {}
};

struct CostWeights
{
    double obstacle_cost;
    double smoothness_cost;
    double length_cost;
    double curvature_cost;
    double velocity_cost;
    double goal_cost;
    
    CostWeights() :
        obstacle_cost(1000.0),
        smoothness_cost(1.0),
        length_cost(1.0),
        curvature_cost(10.0),
        velocity_cost(1.0),
        goal_cost(100.0) {}
};

// State space for planning
struct State
{
    double x, y, theta;
    double velocity;
    
    State() : x(0), y(0), theta(0), velocity(0) {}
    State(double x_, double y_, double theta_, double v_ = 0) : x(x_), y(y_), theta(theta_), velocity(v_) {}
    
    Eigen::VectorXd toVector() const {
        Eigen::VectorXd vec(4);
        vec << x, y, theta, velocity;
        return vec;
    }
    
    void fromVector(const Eigen::VectorXd& vec) {
        x = vec(0);
        y = vec(1);
        theta = vec(2);
        velocity = vec(3);
    }
    
    double distance(const State& other) const {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }
};

// Control input for trajectory generation
struct Control
{
    double linear_velocity;
    double angular_velocity;
    double acceleration;
    double steering_angle;
    
    Control() : linear_velocity(0), angular_velocity(0), acceleration(0), steering_angle(0) {}
    Control(double v, double w) : linear_velocity(v), angular_velocity(w), acceleration(0), steering_angle(0) {}
};

// Utility functions
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline double distance(const Point2D& p1, const Point2D& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

inline double distance(const Pose2D& p1, const Pose2D& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

inline Pose2D interpolate(const Pose2D& p1, const Pose2D& p2, double t) {
    Pose2D result;
    result.x = p1.x + t * (p2.x - p1.x);
    result.y = p1.y + t * (p2.y - p1.y);
    result.theta = normalizeAngle(p1.theta + t * normalizeAngle(p2.theta - p1.theta));
    return result;
}

inline double calculateCurvature(const Pose2D& p1, const Pose2D& p2, const Pose2D& p3) {
    // Calculate curvature using three points
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;
    
    double cross = dx1 * dy2 - dy1 * dx2;
    double norm1 = sqrt(dx1 * dx1 + dy1 * dy1);
    double norm2 = sqrt(dx2 * dx2 + dy2 * dy2);
    
    if (norm1 < 1e-6 || norm2 < 1e-6) return 0.0;
    
    return 2.0 * cross / (norm1 * norm2 * (norm1 + norm2));
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

} // namespace tadeo_ecar_planning

#endif // TADEO_ECAR_PLANNING_TYPES_HPP_