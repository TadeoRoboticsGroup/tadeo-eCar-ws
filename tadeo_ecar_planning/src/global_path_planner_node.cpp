#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/srv/navigate_to_goal.hpp>
#include "tadeo_ecar_planning/planning_types.hpp"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <cmath>
#include <algorithm>
#include <memory>
#include <chrono>

namespace tadeo_ecar_planning
{

class GlobalPathPlannerNode : public rclcpp::Node
{
public:
    GlobalPathPlannerNode() : Node("global_path_planner_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializePlanner();
        
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10,
            std::bind(&GlobalPathPlannerNode::mapCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "move_base_simple/goal", 10,
            std::bind(&GlobalPathPlannerNode::goalCallback, this, std::placeholders::_1));
        
        current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10,
            std::bind(&GlobalPathPlannerNode::currentPoseCallback, this, std::placeholders::_1));
        
        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
        path_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("global_path_markers", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("planning/global_health", 10);
        
        // Services
        plan_service_ = this->create_service<tadeo_ecar_interfaces::srv::NavigateToGoal>(
            "plan_global_path",
            std::bind(&GlobalPathPlannerNode::planPathService, this, std::placeholders::_1, std::placeholders::_2));
        
        // Timers
        planning_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / planning_frequency_)),
            std::bind(&GlobalPathPlannerNode::planningLoop, this));
        
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&GlobalPathPlannerNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Global Path Planner Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("planning_frequency", 2.0);
        this->declare_parameter("planner_type", "RRTstar");
        this->declare_parameter("max_planning_time", 5.0);
        this->declare_parameter("goal_tolerance", 0.2);
        this->declare_parameter("robot_radius", 0.3);
        this->declare_parameter("safety_distance", 0.1);
        this->declare_parameter("resolution", 0.05);
        this->declare_parameter("max_velocity", 2.0);
        this->declare_parameter("path_smoothing_enabled", true);
        this->declare_parameter("dynamic_replanning", true);
        this->declare_parameter("replanning_distance_threshold", 1.0);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("base_frame", "base_link");
        
        planning_frequency_ = this->get_parameter("planning_frequency").as_double();
        planner_type_ = this->get_parameter("planner_type").as_string();
        max_planning_time_ = this->get_parameter("max_planning_time").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        safety_distance_ = this->get_parameter("safety_distance").as_double();
        resolution_ = this->get_parameter("resolution").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        path_smoothing_enabled_ = this->get_parameter("path_smoothing_enabled").as_bool();
        dynamic_replanning_ = this->get_parameter("dynamic_replanning").as_bool();
        replanning_distance_threshold_ = this->get_parameter("replanning_distance_threshold").as_double();
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
    }
    
    void initializePlanner()
    {
        // Initialize OMPL state space
        space_ = std::make_shared<ompl::base::SE2StateSpace>();
        
        // Set bounds (will be updated when map is received)
        ompl::base::RealVectorBounds bounds(2);
        bounds.setLow(-10.0);
        bounds.setHigh(10.0);
        space_->setBounds(bounds);
        
        // Create space information
        si_ = std::make_shared<ompl::base::SpaceInformation>(space_);
        si_->setStateValidityChecker(std::bind(&GlobalPathPlannerNode::isStateValid, this, std::placeholders::_1));
        si_->setup();
        
        // Initialize planning state
        current_pose_available_ = false;
        goal_received_ = false;
        map_available_ = false;
        planning_in_progress_ = false;
        
        RCLCPP_INFO(this->get_logger(), "OMPL planner initialized with %s", planner_type_.c_str());
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = *msg;
        map_available_ = true;
        last_map_time_ = msg->header.stamp;
        
        // Update planning bounds based on map
        updatePlanningBounds();
        
        // Trigger replanning if goal exists and dynamic replanning is enabled
        if (goal_received_ && dynamic_replanning_ && current_pose_available_) {
            should_replan_ = true;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Map updated: %dx%d at %.3fm resolution",
                     current_map_.info.width, current_map_.info.height, current_map_.info.resolution);
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_goal_ = fromRosPose(*msg);
        goal_received_ = true;
        should_replan_ = true;
        
        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f, %.2f)",
                    current_goal_.x, current_goal_.y, current_goal_.theta);
    }
    
    void currentPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_pose_ = Pose2D(msg->pose.pose.position.x, msg->pose.pose.position.y, 
                               2.0 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        current_pose_available_ = true;
        last_pose_time_ = msg->header.stamp;
        
        // Check if we need to replan due to significant position change
        if (goal_received_ && dynamic_replanning_ && !current_path_.empty()) {
            double distance_to_path = calculateDistanceToPath(current_pose_);
            if (distance_to_path > replanning_distance_threshold_) {
                should_replan_ = true;
                RCLCPP_INFO(this->get_logger(), "Triggering replanning due to path deviation: %.2fm", distance_to_path);
            }
        }
    }
    
    void updatePlanningBounds()
    {
        if (!map_available_) return;
        
        ompl::base::RealVectorBounds bounds(2);
        bounds.setLow(0, current_map_.info.origin.position.x);
        bounds.setHigh(0, current_map_.info.origin.position.x + current_map_.info.width * current_map_.info.resolution);
        bounds.setLow(1, current_map_.info.origin.position.y);
        bounds.setHigh(1, current_map_.info.origin.position.y + current_map_.info.height * current_map_.info.resolution);
        
        space_->setBounds(bounds);
        si_->setup();
        
        RCLCPP_DEBUG(this->get_logger(), "Planning bounds updated: [%.2f,%.2f] x [%.2f,%.2f]",
                     bounds.low[0], bounds.high[0], bounds.low[1], bounds.high[1]);
    }
    
    bool isStateValid(const ompl::base::State *state)
    {
        if (!map_available_) return false;
        
        const auto *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
        double x = se2state->getX();
        double y = se2state->getY();
        
        // Check if position is within map bounds
        if (x < current_map_.info.origin.position.x || 
            y < current_map_.info.origin.position.y ||
            x >= current_map_.info.origin.position.x + current_map_.info.width * current_map_.info.resolution ||
            y >= current_map_.info.origin.position.y + current_map_.info.height * current_map_.info.resolution) {
            return false;
        }
        
        // Check collision with obstacles (including robot radius + safety distance)
        double total_radius = robot_radius_ + safety_distance_;
        int steps = static_cast<int>(ceil(total_radius / current_map_.info.resolution));
        
        for (int dx = -steps; dx <= steps; ++dx) {
            for (int dy = -steps; dy <= steps; ++dy) {
                double check_x = x + dx * current_map_.info.resolution;
                double check_y = y + dy * current_map_.info.resolution;
                
                // Skip if outside robot footprint
                if (sqrt(dx*dx + dy*dy) * current_map_.info.resolution > total_radius) continue;
                
                // Convert to map coordinates
                int map_x = static_cast<int>((check_x - current_map_.info.origin.position.x) / current_map_.info.resolution);
                int map_y = static_cast<int>((check_y - current_map_.info.origin.position.y) / current_map_.info.resolution);
                
                if (map_x >= 0 && map_x < static_cast<int>(current_map_.info.width) &&
                    map_y >= 0 && map_y < static_cast<int>(current_map_.info.height)) {
                    
                    int index = map_y * current_map_.info.width + map_x;
                    if (current_map_.data[index] > 50 || current_map_.data[index] == -1) { // Occupied or unknown
                        return false;
                    }
                }
            }
        }
        
        return true;
    }
    
    void planningLoop()
    {
        if (!should_replan_ || planning_in_progress_) return;
        
        if (current_pose_available_ && goal_received_ && map_available_) {
            planPath();
        }
    }
    
    void planPath()
    {
        if (planning_in_progress_) return;
        
        planning_in_progress_ = true;
        should_replan_ = false;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Create problem definition
        auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si_);
        
        // Set start state
        ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space_);
        start->setX(current_pose_.x);
        start->setY(current_pose_.y);
        start->setYaw(current_pose_.theta);
        pdef->setStartAndGoalStates(start, createGoalState());
        
        // Create planner
        ompl::base::PlannerPtr planner = createPlanner();
        planner->setProblemDefinition(pdef);
        planner->setup();
        
        // Solve
        ompl::base::PlannerStatus solved = planner->solve(max_planning_time_);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (solved) {
            // Get solution path
            auto solution = pdef->getSolutionPath();
            auto geometric_path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(solution);
            
            if (geometric_path) {
                // Smooth path if enabled
                if (path_smoothing_enabled_) {
                    ompl::geometric::PathSimplifier simplifier(si_);
                    simplifier.smoothBSpline(*geometric_path);
                }
                
                // Convert to ROS path
                current_path_ = convertOMPLPathToROS(geometric_path);
                
                // Publish path
                publishPath();
                publishPathVisualization();
                
                RCLCPP_INFO(this->get_logger(), "Path planned successfully in %ldms (%zu points)",
                            duration.count(), current_path_.points.size());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to find path in %ldms", duration.count());
            current_path_.clear();
        }
        
        planning_in_progress_ = false;
        last_planning_time_ = this->now();
    }
    
    ompl::base::ScopedState<ompl::base::SE2StateSpace> createGoalState()
    {
        ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space_);
        goal->setX(current_goal_.x);
        goal->setY(current_goal_.y);
        goal->setYaw(current_goal_.theta);
        return goal;
    }
    
    ompl::base::PlannerPtr createPlanner()
    {
        if (planner_type_ == "RRTstar") {
            auto planner = std::make_shared<ompl::geometric::RRTstar>(si_);
            planner->setRange(1.0);
            return planner;
        } else if (planner_type_ == "PRM") {
            auto planner = std::make_shared<ompl::geometric::PRM>(si_);
            return planner;
        } else if (planner_type_ == "EST") {
            auto planner = std::make_shared<ompl::geometric::EST>(si_);
            return planner;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown planner type: %s, using RRTstar", planner_type_.c_str());
            auto planner = std::make_shared<ompl::geometric::RRTstar>(si_);
            planner->setRange(1.0);
            return planner;
        }
    }
    
    Path convertOMPLPathToROS(const std::shared_ptr<ompl::geometric::PathGeometric>& ompl_path)
    {
        Path path;
        path.frame_id = map_frame_;
        path.timestamp = this->now();
        
        for (size_t i = 0; i < ompl_path->getStateCount(); ++i) {
            const auto *state = ompl_path->getState(i)->as<ompl::base::SE2StateSpace::StateType>();
            
            PathPoint point;
            point.pose.x = state->getX();
            point.pose.y = state->getY();
            point.pose.theta = state->getYaw();
            point.speed = max_velocity_; // Will be refined by trajectory optimizer
            
            path.points.push_back(point);
        }
        
        // Calculate curvature and time stamps
        for (size_t i = 1; i < path.points.size() - 1; ++i) {
            path.points[i].curvature = calculateCurvature(
                path.points[i-1].pose, path.points[i].pose, path.points[i+1].pose);
        }
        
        path.calculateLength();
        return path;
    }
    
    double calculateDistanceToPath(const Pose2D& pose)
    {
        if (current_path_.empty()) return std::numeric_limits<double>::max();
        
        double min_distance = std::numeric_limits<double>::max();
        
        for (const auto& point : current_path_.points) {
            double dist = pose.distance(point.pose);
            min_distance = std::min(min_distance, dist);
        }
        
        return min_distance;
    }
    
    void publishPath()
    {
        if (current_path_.empty()) return;
        
        nav_msgs::msg::Path ros_path;
        ros_path.header.stamp = this->now();
        ros_path.header.frame_id = map_frame_;
        
        for (const auto& point : current_path_.points) {
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
        
        path_pub_->publish(ros_path);
    }
    
    void publishPathVisualization()
    {
        if (current_path_.empty()) return;
        
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Path line marker
        visualization_msgs::msg::Marker path_marker;
        path_marker.header.frame_id = map_frame_;
        path_marker.header.stamp = this->now();
        path_marker.ns = "global_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        
        for (const auto& point : current_path_.points) {
            geometry_msgs::msg::Point p;
            p.x = point.pose.x;
            p.y = point.pose.y;
            p.z = 0.1;
            path_marker.points.push_back(p);
        }
        
        path_marker.scale.x = 0.05;
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;
        
        marker_array.markers.push_back(path_marker);
        
        // Start and goal markers
        visualization_msgs::msg::Marker start_marker;
        start_marker.header.frame_id = map_frame_;
        start_marker.header.stamp = this->now();
        start_marker.ns = "global_path";
        start_marker.id = 1;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        
        start_marker.pose.position.x = current_pose_.x;
        start_marker.pose.position.y = current_pose_.y;
        start_marker.pose.position.z = 0.2;
        start_marker.pose.orientation.w = 1.0;
        
        start_marker.scale.x = 0.3;
        start_marker.scale.y = 0.3;
        start_marker.scale.z = 0.3;
        start_marker.color.r = 0.0;
        start_marker.color.g = 0.0;
        start_marker.color.b = 1.0;
        start_marker.color.a = 1.0;
        
        marker_array.markers.push_back(start_marker);
        
        visualization_msgs::msg::Marker goal_marker = start_marker;
        goal_marker.id = 2;
        goal_marker.pose.position.x = current_goal_.x;
        goal_marker.pose.position.y = current_goal_.y;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
        
        marker_array.markers.push_back(goal_marker);
        
        path_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        health_msg.component_name = "global_path_planner";
        
        // Check system health
        auto current_time = this->now();
        bool map_timeout = map_available_ && (current_time - last_map_time_).seconds() > 5.0;
        bool pose_timeout = current_pose_available_ && (current_time - last_pose_time_).seconds() > 2.0;
        
        if (map_timeout) {
            health_msg.status = "WARNING";
            health_msg.error_code = 11001;
            health_msg.error_message = "Map data timeout";
        } else if (pose_timeout) {
            health_msg.status = "WARNING";
            health_msg.error_code = 11002;
            health_msg.error_message = "Pose data timeout";
        } else if (planning_in_progress_) {
            health_msg.status = "PLANNING";
            health_msg.error_code = 11003;
            health_msg.error_message = "Planning in progress";
        } else if (!map_available_) {
            health_msg.status = "WAITING";
            health_msg.error_code = 11004;
            health_msg.error_message = "Waiting for map data";
        } else if (!current_pose_available_) {
            health_msg.status = "WAITING";
            health_msg.error_code = 11005;
            health_msg.error_message = "Waiting for pose data";
        } else {
            health_msg.status = "OK";
            health_msg.error_code = 0;
            health_msg.error_message = "";
        }
        
        health_msg.cpu_usage = 30.0; // Placeholder
        health_msg.memory_usage = 25.0; // Placeholder
        health_msg.temperature = 45.0; // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    void planPathService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::NavigateToGoal::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::NavigateToGoal::Response> response)
    {
        try {
            // Set goal from service request
            current_goal_ = Pose2D(request->goal.pose.position.x, 
                                   request->goal.pose.position.y,
                                   2.0 * atan2(request->goal.pose.orientation.z, request->goal.pose.orientation.w));
            goal_received_ = true;
            
            // Plan path synchronously
            planPath();
            
            if (!current_path_.empty()) {
                response->success = true;
                response->message = "Path planned successfully";
                
                // Convert path to response format
                nav_msgs::msg::Path ros_path;
                ros_path.header.stamp = this->now();
                ros_path.header.frame_id = map_frame_;
                
                for (const auto& point : current_path_.points) {
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
                
                response->path = ros_path;
            } else {
                response->success = false;
                response->message = "Failed to plan path";
            }
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Planning failed: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Planning service failed: %s", e.what());
        }
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr current_pose_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::Service<tadeo_ecar_interfaces::srv::NavigateToGoal>::SharedPtr plan_service_;
    
    rclcpp::TimerInterface::SharedPtr planning_timer_;
    rclcpp::TimerInterface::SharedPtr health_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // OMPL components
    std::shared_ptr<ompl::base::SE2StateSpace> space_;
    std::shared_ptr<ompl::base::SpaceInformation> si_;
    
    // Parameters
    double planning_frequency_;
    std::string planner_type_;
    double max_planning_time_;
    double goal_tolerance_;
    double robot_radius_;
    double safety_distance_;
    double resolution_;
    double max_velocity_;
    bool path_smoothing_enabled_;
    bool dynamic_replanning_;
    double replanning_distance_threshold_;
    std::string map_frame_;
    std::string base_frame_;
    
    // State
    nav_msgs::msg::OccupancyGrid current_map_;
    Pose2D current_pose_;
    Pose2D current_goal_;
    Path current_path_;
    
    bool current_pose_available_;
    bool goal_received_;
    bool map_available_;
    bool planning_in_progress_;
    bool should_replan_;
    
    rclcpp::Time last_map_time_;
    rclcpp::Time last_pose_time_;
    rclcpp::Time last_planning_time_;
};

} // namespace tadeo_ecar_planning

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_planning::GlobalPathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}