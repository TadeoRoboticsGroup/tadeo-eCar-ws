#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include "tadeo_ecar_planning/planning_types.hpp"
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>

namespace tadeo_ecar_planning
{

class LocalPathPlannerNode : public rclcpp::Node
{
public:
    LocalPathPlannerNode() : Node("local_path_planner_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializePlanner();
        
        // Subscribers
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_path", 10,
            std::bind(&LocalPathPlannerNode::globalPathCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&LocalPathPlannerNode::odomCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&LocalPathPlannerNode::scanCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 10);
        trajectory_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("local_trajectory_markers", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("planning/local_health", 10);
        
        // Timers
        planning_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / planning_frequency_)),
            std::bind(&LocalPathPlannerNode::planningLoop, this));
        
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&LocalPathPlannerNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Local Path Planner Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("planning_frequency", 20.0);
        this->declare_parameter("lookahead_distance", 2.0);
        this->declare_parameter("max_linear_velocity", 1.5);
        this->declare_parameter("max_angular_velocity", 1.0);
        this->declare_parameter("max_linear_acceleration", 1.0);
        this->declare_parameter("max_angular_acceleration", 2.0);
        this->declare_parameter("min_obstacle_distance", 0.5);
        this->declare_parameter("prediction_horizon", 2.0);
        this->declare_parameter("trajectory_samples", 20);
        this->declare_parameter("angular_samples", 11);
        this->declare_parameter("robot_radius", 0.3);
        this->declare_parameter("safety_margin", 0.1);
        this->declare_parameter("goal_tolerance", 0.2);
        this->declare_parameter("path_following_enabled", true);
        this->declare_parameter("obstacle_avoidance_enabled", true);
        this->declare_parameter("dynamic_window_approach", true);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
        
        planning_frequency_ = this->get_parameter("planning_frequency").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        max_linear_acceleration_ = this->get_parameter("max_linear_acceleration").as_double();
        max_angular_acceleration_ = this->get_parameter("max_angular_acceleration").as_double();
        min_obstacle_distance_ = this->get_parameter("min_obstacle_distance").as_double();
        prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
        trajectory_samples_ = this->get_parameter("trajectory_samples").as_int();
        angular_samples_ = this->get_parameter("angular_samples").as_int();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        safety_margin_ = this->get_parameter("safety_margin").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        path_following_enabled_ = this->get_parameter("path_following_enabled").as_bool();
        obstacle_avoidance_enabled_ = this->get_parameter("obstacle_avoidance_enabled").as_bool();
        dynamic_window_approach_ = this->get_parameter("dynamic_window_approach").as_bool();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
    }
    
    void initializePlanner()
    {
        // Initialize vehicle constraints
        constraints_.max_velocity = max_linear_velocity_;
        constraints_.max_acceleration = max_linear_acceleration_;
        constraints_.max_angular_velocity = max_angular_velocity_;
        constraints_.max_angular_acceleration = max_angular_acceleration_;
        
        // Initialize cost weights
        cost_weights_.obstacle_cost = 1000.0;
        cost_weights_.smoothness_cost = 1.0;
        cost_weights_.goal_cost = 100.0;
        cost_weights_.velocity_cost = 1.0;
        
        // Initialize state
        global_path_available_ = false;
        current_pose_available_ = false;
        obstacles_detected_ = false;
        at_goal_ = false;
        
        current_velocity_.linear_x = 0.0;
        current_velocity_.angular_z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Local planner initialized with DWA: %s", 
                    dynamic_window_approach_ ? "enabled" : "disabled");
    }
    
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            global_path_available_ = false;
            return;
        }
        
        // Convert to internal path representation
        global_path_.clear();
        global_path_.frame_id = msg->header.frame_id;
        global_path_.timestamp = msg->header.stamp;
        
        for (const auto& pose_stamped : msg->poses) {
            PathPoint point;
            point.pose.x = pose_stamped.pose.position.x;
            point.pose.y = pose_stamped.pose.position.y;
            
            // Convert quaternion to theta
            double qw = pose_stamped.pose.orientation.w;
            double qz = pose_stamped.pose.orientation.z;
            point.pose.theta = 2.0 * atan2(qz, qw);
            
            global_path_.points.push_back(point);
        }
        
        global_path_available_ = true;
        last_global_path_time_ = msg->header.stamp;
        
        RCLCPP_DEBUG(this->get_logger(), "Global path updated with %zu points", global_path_.size());
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update current pose
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        current_pose_.theta = 2.0 * atan2(qz, qw);
        current_pose_.timestamp = msg->header.stamp;
        
        // Update current velocity
        current_velocity_.linear_x = msg->twist.twist.linear.x;
        current_velocity_.linear_y = msg->twist.twist.linear.y;
        current_velocity_.angular_z = msg->twist.twist.angular.z;
        
        current_pose_available_ = true;
        last_odom_time_ = msg->header.stamp;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!obstacle_avoidance_enabled_) return;
        
        // Convert scan to obstacles
        obstacles_.clear();
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (std::isnan(msg->ranges[i]) || std::isinf(msg->ranges[i])) continue;
            
            double range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max) continue;
            
            double angle = msg->angle_min + i * msg->angle_increment;
            
            // Convert to world coordinates
            double x = current_pose_.x + range * cos(current_pose_.theta + angle);
            double y = current_pose_.y + range * sin(current_pose_.theta + angle);
            
            // Only consider obstacles within prediction horizon
            if (distance(Point2D(x, y), Point2D(current_pose_.x, current_pose_.y)) <= prediction_horizon_) {
                Obstacle obstacle(Point2D(x, y), 0.1); // Small radius for scan points
                obstacles_.push_back(obstacle);
            }
        }
        
        obstacles_detected_ = !obstacles_.empty();
        last_scan_time_ = msg->header.stamp;
        
        RCLCPP_DEBUG(this->get_logger(), "Detected %zu obstacles within %.2fm", 
                     obstacles_.size(), prediction_horizon_);
    }
    
    void planningLoop()
    {
        if (!current_pose_available_) return;
        
        // Check if we've reached the goal
        if (global_path_available_ && !global_path_.empty()) {
            double distance_to_goal = current_pose_.distance(global_path_.points.back().pose);
            at_goal_ = (distance_to_goal < goal_tolerance_);
            
            if (at_goal_) {
                publishStopCommand();
                return;
            }
        }
        
        // Plan local trajectory
        Trajectory best_trajectory;
        
        if (dynamic_window_approach_) {
            best_trajectory = planWithDWA();
        } else {
            best_trajectory = planWithPurePursuit();
        }
        
        // Execute trajectory
        if (!best_trajectory.empty()) {
            executeTrajectory(best_trajectory);
            publishTrajectoryVisualization(best_trajectory);
        } else {
            publishStopCommand();
        }
    }
    
    Trajectory planWithDWA()
    {
        if (!global_path_available_) {
            return Trajectory();
        }
        
        // Dynamic Window Approach
        std::vector<Trajectory> candidate_trajectories;
        
        // Calculate dynamic window
        double min_v = std::max(0.0, current_velocity_.linear_x - max_linear_acceleration_ / planning_frequency_);
        double max_v = std::min(max_linear_velocity_, current_velocity_.linear_x + max_linear_acceleration_ / planning_frequency_);
        double min_w = std::max(-max_angular_velocity_, current_velocity_.angular_z - max_angular_acceleration_ / planning_frequency_);
        double max_w = std::min(max_angular_velocity_, current_velocity_.angular_z + max_angular_acceleration_ / planning_frequency_);
        
        // Sample trajectories
        for (int v_idx = 0; v_idx < trajectory_samples_; ++v_idx) {
            double v = min_v + v_idx * (max_v - min_v) / (trajectory_samples_ - 1);
            
            for (int w_idx = 0; w_idx < angular_samples_; ++w_idx) {
                double w = min_w + w_idx * (max_w - min_w) / (angular_samples_ - 1);
                
                Trajectory traj = simulateTrajectory(v, w);
                if (!traj.empty()) {
                    candidate_trajectories.push_back(traj);
                }
            }
        }
        
        // Evaluate trajectories and select best
        return selectBestTrajectory(candidate_trajectories);
    }
    
    Trajectory planWithPurePursuit()
    {
        if (!global_path_available_ || global_path_.empty()) {
            return Trajectory();
        }
        
        // Find lookahead point
        Point2D lookahead_point = findLookaheadPoint();
        
        // Calculate desired velocity
        double distance_to_lookahead = distance(Point2D(current_pose_.x, current_pose_.y), lookahead_point);
        double desired_velocity = std::min(max_linear_velocity_, distance_to_lookahead);
        
        // Calculate angular velocity for pure pursuit
        double angle_to_lookahead = atan2(lookahead_point.y - current_pose_.y, 
                                          lookahead_point.x - current_pose_.x);
        double angle_error = normalizeAngle(angle_to_lookahead - current_pose_.theta);
        double angular_velocity = 2.0 * desired_velocity * sin(angle_error) / lookahead_distance_;
        
        // Limit angular velocity
        angular_velocity = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);
        
        // Create trajectory
        return simulateTrajectory(desired_velocity, angular_velocity);
    }
    
    Point2D findLookaheadPoint()
    {
        if (global_path_.empty()) {
            return Point2D(current_pose_.x, current_pose_.y);
        }
        
        // Find closest point on path
        size_t closest_idx = 0;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < global_path_.size(); ++i) {
            double dist = current_pose_.distance(global_path_[i].pose);
            if (dist < min_distance) {
                min_distance = dist;
                closest_idx = i;
            }
        }
        
        // Find lookahead point
        for (size_t i = closest_idx; i < global_path_.size(); ++i) {
            double dist = current_pose_.distance(global_path_[i].pose);
            if (dist >= lookahead_distance_) {
                return Point2D(global_path_[i].pose.x, global_path_[i].pose.y);
            }
        }
        
        // Return last point if no lookahead point found
        return Point2D(global_path_.points.back().pose.x, global_path_.points.back().pose.y);
    }
    
    Trajectory simulateTrajectory(double linear_vel, double angular_vel)
    {
        Trajectory trajectory;
        
        // Simulate trajectory forward in time
        State current_state(current_pose_.x, current_pose_.y, current_pose_.theta, linear_vel);
        
        double dt = 1.0 / planning_frequency_;
        int steps = static_cast<int>(prediction_horizon_ / dt);
        
        for (int i = 0; i <= steps; ++i) {
            PathPoint point;
            point.pose.x = current_state.x;
            point.pose.y = current_state.y;
            point.pose.theta = current_state.theta;
            point.velocity.linear_x = linear_vel;
            point.velocity.angular_z = angular_vel;
            point.time_from_start = i * dt;
            
            trajectory.points.push_back(point);
            
            // Integrate motion model
            current_state.x += linear_vel * cos(current_state.theta) * dt;
            current_state.y += linear_vel * sin(current_state.theta) * dt;
            current_state.theta = normalizeAngle(current_state.theta + angular_vel * dt);
        }
        
        return trajectory;
    }
    
    Trajectory selectBestTrajectory(const std::vector<Trajectory>& candidates)
    {
        if (candidates.empty()) return Trajectory();
        
        double best_cost = std::numeric_limits<double>::max();
        Trajectory best_trajectory;
        
        for (const auto& trajectory : candidates) {
            double cost = evaluateTrajectory(trajectory);
            
            if (cost < best_cost) {
                best_cost = cost;
                best_trajectory = trajectory;
            }
        }
        
        return best_trajectory;
    }
    
    double evaluateTrajectory(const Trajectory& trajectory)
    {
        if (trajectory.empty()) return std::numeric_limits<double>::max();
        
        double total_cost = 0.0;
        
        // Obstacle cost
        if (obstacle_avoidance_enabled_) {
            for (const auto& point : trajectory.points) {
                double min_obstacle_dist = std::numeric_limits<double>::max();
                
                for (const auto& obstacle : obstacles_) {
                    double dist = distance(Point2D(point.pose.x, point.pose.y), obstacle.center);
                    min_obstacle_dist = std::min(min_obstacle_dist, dist);
                }
                
                if (min_obstacle_dist < robot_radius_ + safety_margin_) {
                    return std::numeric_limits<double>::max(); // Collision
                }
                
                if (min_obstacle_dist < min_obstacle_distance_) {
                    total_cost += cost_weights_.obstacle_cost / (min_obstacle_dist + 0.01);
                }
            }
        }
        
        // Goal alignment cost
        if (global_path_available_ && !global_path_.empty()) {
            Point2D goal(global_path_.points.back().pose.x, global_path_.points.back().pose.y);
            Point2D trajectory_end(trajectory.points.back().pose.x, trajectory.points.back().pose.y);
            double goal_distance = distance(trajectory_end, goal);
            total_cost += cost_weights_.goal_cost * goal_distance;
        }
        
        // Velocity cost (prefer higher velocities)
        double avg_velocity = 0.0;
        for (const auto& point : trajectory.points) {
            avg_velocity += point.velocity.linear_x;
        }
        avg_velocity /= trajectory.points.size();
        total_cost += cost_weights_.velocity_cost * (max_linear_velocity_ - avg_velocity);
        
        // Smoothness cost
        for (size_t i = 1; i < trajectory.points.size(); ++i) {
            double angular_change = abs(normalizeAngle(
                trajectory.points[i].pose.theta - trajectory.points[i-1].pose.theta));
            total_cost += cost_weights_.smoothness_cost * angular_change;
        }
        
        return total_cost;
    }
    
    void executeTrajectory(const Trajectory& trajectory)
    {
        if (trajectory.empty()) return;
        
        // Use first control command from trajectory
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = trajectory.points[1].velocity.linear_x; // Use second point for smoother control
        cmd_vel.angular.z = trajectory.points[1].velocity.angular_z;
        
        cmd_vel_pub_->publish(cmd_vel);
        
        // Publish local path
        publishLocalPath(trajectory);
    }
    
    void publishStopCommand()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    void publishLocalPath(const Trajectory& trajectory)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = map_frame_;
        
        for (const auto& point : trajectory.points) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose.position.x = point.pose.x;
            pose_stamped.pose.position.y = point.pose.y;
            pose_stamped.pose.position.z = 0.0;
            
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = sin(point.pose.theta / 2.0);
            pose_stamped.pose.orientation.w = cos(point.pose.theta / 2.0);
            
            path_msg.poses.push_back(pose_stamped);
        }
        
        local_path_pub_->publish(path_msg);
    }
    
    void publishTrajectoryVisualization(const Trajectory& trajectory)
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Trajectory line
        visualization_msgs::msg::Marker traj_marker;
        traj_marker.header.frame_id = map_frame_;
        traj_marker.header.stamp = this->now();
        traj_marker.ns = "local_trajectory";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::msg::Marker::ADD;
        
        for (const auto& point : trajectory.points) {
            geometry_msgs::msg::Point p;
            p.x = point.pose.x;
            p.y = point.pose.y;
            p.z = 0.05;
            traj_marker.points.push_back(p);
        }
        
        traj_marker.scale.x = 0.03;
        traj_marker.color.r = 1.0;
        traj_marker.color.g = 0.5;
        traj_marker.color.b = 0.0;
        traj_marker.color.a = 1.0;
        
        marker_array.markers.push_back(traj_marker);
        
        // Robot footprint
        visualization_msgs::msg::Marker robot_marker;
        robot_marker.header.frame_id = map_frame_;
        robot_marker.header.stamp = this->now();
        robot_marker.ns = "local_trajectory";
        robot_marker.id = 1;
        robot_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        robot_marker.action = visualization_msgs::msg::Marker::ADD;
        
        robot_marker.pose.position.x = current_pose_.x;
        robot_marker.pose.position.y = current_pose_.y;
        robot_marker.pose.position.z = 0.1;
        robot_marker.pose.orientation.x = 0.0;
        robot_marker.pose.orientation.y = 0.0;
        robot_marker.pose.orientation.z = sin(current_pose_.theta / 2.0);
        robot_marker.pose.orientation.w = cos(current_pose_.theta / 2.0);
        
        robot_marker.scale.x = robot_radius_ * 2;
        robot_marker.scale.y = robot_radius_ * 2;
        robot_marker.scale.z = 0.2;
        robot_marker.color.r = 0.0;
        robot_marker.color.g = 0.0;
        robot_marker.color.b = 1.0;
        robot_marker.color.a = 0.5;
        
        marker_array.markers.push_back(robot_marker);
        
        trajectory_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        health_msg.component_name = "local_path_planner";
        
        // Check system health
        auto current_time = this->now();
        bool path_timeout = global_path_available_ && (current_time - last_global_path_time_).seconds() > 2.0;
        bool odom_timeout = current_pose_available_ && (current_time - last_odom_time_).seconds() > 1.0;
        bool scan_timeout = obstacle_avoidance_enabled_ && (current_time - last_scan_time_).seconds() > 1.0;
        
        if (odom_timeout) {
            health_msg.status = "WARNING";
            health_msg.error_code = 12001;
            health_msg.error_message = "Odometry data timeout";
        } else if (path_timeout) {
            health_msg.status = "WARNING";
            health_msg.error_code = 12002;
            health_msg.error_message = "Global path timeout";
        } else if (scan_timeout) {
            health_msg.status = "WARNING";
            health_msg.error_code = 12003;
            health_msg.error_message = "Scan data timeout";
        } else if (!current_pose_available_) {
            health_msg.status = "WAITING";
            health_msg.error_code = 12004;
            health_msg.error_message = "Waiting for odometry";
        } else if (!global_path_available_) {
            health_msg.status = "WAITING";
            health_msg.error_code = 12005;
            health_msg.error_message = "Waiting for global path";
        } else if (at_goal_) {
            health_msg.status = "AT_GOAL";
            health_msg.error_code = 12006;
            health_msg.error_message = "Reached goal";
        } else {
            health_msg.status = "OK";
            health_msg.error_code = 0;
            health_msg.error_message = "";
        }
        
        health_msg.cpu_usage = 25.0; // Placeholder
        health_msg.memory_usage = 15.0; // Placeholder
        health_msg.temperature = 40.0; // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::TimerInterface::SharedPtr planning_timer_;
    rclcpp::TimerInterface::SharedPtr health_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double planning_frequency_;
    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double max_linear_acceleration_;
    double max_angular_acceleration_;
    double min_obstacle_distance_;
    double prediction_horizon_;
    int trajectory_samples_;
    int angular_samples_;
    double robot_radius_;
    double safety_margin_;
    double goal_tolerance_;
    bool path_following_enabled_;
    bool obstacle_avoidance_enabled_;
    bool dynamic_window_approach_;
    std::string base_frame_;
    std::string map_frame_;
    
    // Planning state
    Path global_path_;
    Pose2D current_pose_;
    Velocity2D current_velocity_;
    std::vector<Obstacle> obstacles_;
    VehicleConstraints constraints_;
    CostWeights cost_weights_;
    
    bool global_path_available_;
    bool current_pose_available_;
    bool obstacles_detected_;
    bool at_goal_;
    
    rclcpp::Time last_global_path_time_;
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_scan_time_;
};

} // namespace tadeo_ecar_planning

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_planning::LocalPathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}