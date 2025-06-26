#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/srv/save_map.hpp>
#include "tadeo_ecar_slam/slam_types.hpp"
#include <cmath>
#include <algorithm>

namespace tadeo_ecar_slam
{

class GridSLAMNode : public rclcpp::Node
{
public:
    GridSLAMNode() : Node("grid_slam_node"), tf_broadcaster_(this), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeGrid();
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&GridSLAMNode::scanCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&GridSLAMNode::odomCallback, this, std::placeholders::_1));
        
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            std::bind(&GridSLAMNode::initialPoseCallback, this, std::placeholders::_1));
        
        // Publishers
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("slam_pose", 10);
        corrected_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("slam/grid_health", 10);
        
        // Services
        save_map_service_ = this->create_service<tadeo_ecar_interfaces::srv::SaveMap>(
            "save_map",
            std::bind(&GridSLAMNode::saveMapService, this, std::placeholders::_1, std::placeholders::_2));
        
        // Timers
        map_update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / map_update_frequency_)),
            std::bind(&GridSLAMNode::updateMapLoop, this));
        
        slam_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / slam_frequency_)),
            std::bind(&GridSLAMNode::slamLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Grid SLAM Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("slam_frequency", 10.0);
        this->declare_parameter("map_update_frequency", 2.0);
        this->declare_parameter("map_resolution", 0.05);
        this->declare_parameter("map_width", 2000);
        this->declare_parameter("map_height", 2000);
        this->declare_parameter("map_origin_x", -50.0);
        this->declare_parameter("map_origin_y", -50.0);
        this->declare_parameter("scan_matching_enabled", true);
        this->declare_parameter("scan_matching_max_iterations", 20);
        this->declare_parameter("scan_matching_epsilon", 0.001);
        this->declare_parameter("scan_matching_max_range", 10.0);
        this->declare_parameter("min_translation_for_scan", 0.2);
        this->declare_parameter("min_rotation_for_scan", 0.1);
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        
        slam_frequency_ = this->get_parameter("slam_frequency").as_double();
        map_update_frequency_ = this->get_parameter("map_update_frequency").as_double();
        map_resolution_ = this->get_parameter("map_resolution").as_double();
        map_width_ = this->get_parameter("map_width").as_int();
        map_height_ = this->get_parameter("map_height").as_int();
        map_origin_x_ = this->get_parameter("map_origin_x").as_double();
        map_origin_y_ = this->get_parameter("map_origin_y").as_double();
        scan_matching_enabled_ = this->get_parameter("scan_matching_enabled").as_bool();
        scan_matching_max_iterations_ = this->get_parameter("scan_matching_max_iterations").as_int();
        scan_matching_epsilon_ = this->get_parameter("scan_matching_epsilon").as_double();
        scan_matching_max_range_ = this->get_parameter("scan_matching_max_range").as_double();
        min_translation_for_scan_ = this->get_parameter("min_translation_for_scan").as_double();
        min_rotation_for_scan_ = this->get_parameter("min_rotation_for_scan").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
    }
    
    void initializeGrid()
    {
        occupancy_grid_ = std::make_unique<OccupancyGrid>(
            map_resolution_, map_width_, map_height_, map_origin_x_, map_origin_y_);
        
        // Initialize pose
        current_pose_ = Pose2D(0.0, 0.0, 0.0);
        last_scan_pose_ = current_pose_;
        
        // Initialize state
        first_scan_received_ = false;
        odom_available_ = false;
        last_odom_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Grid initialized: %dx%d at %.3fm resolution", 
                    map_width_, map_height_, map_resolution_);
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        current_scan_ = *msg;
        last_scan_time_ = msg->header.stamp;
        
        if (!first_scan_received_) {
            first_scan_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First scan received");
        }
        
        // Check if we should process this scan
        if (shouldProcessScan()) {
            processScan();
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        odom_available_ = true;
        last_odom_time_ = msg->header.stamp;
        
        // Update pose from odometry
        updatePoseFromOdometry();
    }
    
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Reset SLAM with new initial pose
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pose_.theta = yaw;
        
        last_scan_pose_ = current_pose_;
        
        RCLCPP_INFO(this->get_logger(), "SLAM reset with initial pose: (%.2f, %.2f, %.2f)",
                    current_pose_.x, current_pose_.y, current_pose_.theta);
    }
    
    bool shouldProcessScan()
    {
        if (!first_scan_received_ || !odom_available_) {
            return false;
        }
        
        // Check if robot has moved enough since last scan processing
        double distance = sqrt(pow(current_pose_.x - last_scan_pose_.x, 2) + 
                               pow(current_pose_.y - last_scan_pose_.y, 2));
        double angle_diff = abs(normalizeAngle(current_pose_.theta - last_scan_pose_.theta));
        
        return (distance > min_translation_for_scan_ || angle_diff > min_rotation_for_scan_);
    }
    
    void updatePoseFromOdometry()
    {
        if (!odom_available_) return;
        
        // Simple odometry integration (this could be more sophisticated)
        static nav_msgs::msg::Odometry last_odom;
        static bool first_odom = true;
        
        if (first_odom) {
            last_odom = current_odom_;
            first_odom = false;
            return;
        }
        
        // Calculate odometry change
        double dx = current_odom_.pose.pose.position.x - last_odom.pose.pose.position.x;
        double dy = current_odom_.pose.pose.position.y - last_odom.pose.pose.position.y;
        
        tf2::Quaternion q_current, q_last;
        tf2::fromMsg(current_odom_.pose.pose.orientation, q_current);
        tf2::fromMsg(last_odom.pose.pose.orientation, q_last);
        
        tf2::Matrix3x3 m_current(q_current), m_last(q_last);
        double roll, pitch, yaw_current, yaw_last;
        m_current.getRPY(roll, pitch, yaw_current);
        m_last.getRPY(roll, pitch, yaw_last);
        
        double dtheta = normalizeAngle(yaw_current - yaw_last);
        
        // Update current pose (simple dead reckoning)
        current_pose_.x += dx * cos(current_pose_.theta) - dy * sin(current_pose_.theta);
        current_pose_.y += dx * sin(current_pose_.theta) + dy * cos(current_pose_.theta);
        current_pose_.theta = normalizeAngle(current_pose_.theta + dtheta);
        
        last_odom = current_odom_;
    }
    
    void processScan()
    {
        if (scan_matching_enabled_) {
            // Perform scan matching to correct pose
            Pose2D corrected_pose = performScanMatching();
            current_pose_ = corrected_pose;
        }
        
        // Update map with corrected pose
        updateMap();
        
        // Update last scan pose
        last_scan_pose_ = current_pose_;
        
        RCLCPP_DEBUG(this->get_logger(), "Processed scan at pose: (%.2f, %.2f, %.2f)",
                     current_pose_.x, current_pose_.y, current_pose_.theta);
    }
    
    Pose2D performScanMatching()
    {
        Pose2D best_pose = current_pose_;
        double best_score = -std::numeric_limits<double>::max();
        
        // Search parameters
        double search_radius = 0.3;  // 30cm search radius
        double search_step = 0.05;   // 5cm step
        double angle_range = 0.2;    // ~11 degrees
        double angle_step = 0.05;    // ~3 degrees
        
        for (double dx = -search_radius; dx <= search_radius; dx += search_step) {
            for (double dy = -search_radius; dy <= search_radius; dy += search_step) {
                for (double dtheta = -angle_range; dtheta <= angle_range; dtheta += angle_step) {
                    
                    Pose2D test_pose = current_pose_;
                    test_pose.x += dx;
                    test_pose.y += dy;
                    test_pose.theta = normalizeAngle(test_pose.theta + dtheta);
                    
                    double score = calculateScanMatchScore(test_pose);
                    
                    if (score > best_score) {
                        best_score = score;
                        best_pose = test_pose;
                    }
                }
            }
        }
        
        return best_pose;
    }
    
    double calculateScanMatchScore(const Pose2D& pose)
    {
        double score = 0.0;
        int valid_points = 0;
        
        for (size_t i = 0; i < current_scan_.ranges.size(); i += 3) { // Sample every 3rd point
            if (std::isnan(current_scan_.ranges[i]) || std::isinf(current_scan_.ranges[i])) {
                continue;
            }
            
            double range = current_scan_.ranges[i];
            if (range < current_scan_.range_min || range > current_scan_.range_max || 
                range > scan_matching_max_range_) {
                continue;
            }
            
            double angle = current_scan_.angle_min + i * current_scan_.angle_increment;
            
            // Transform point to world coordinates
            double local_x = range * cos(angle);
            double local_y = range * sin(angle);
            
            double world_x = pose.x + local_x * cos(pose.theta) - local_y * sin(pose.theta);
            double world_y = pose.y + local_x * sin(pose.theta) + local_y * cos(pose.theta);
            
            // Check occupancy at this point
            int grid_x, grid_y;
            occupancy_grid_->worldToGrid(world_x, world_y, grid_x, grid_y);
            
            if (occupancy_grid_->isValid(grid_x, grid_y)) {
                const GridCell& cell = occupancy_grid_->getCell(grid_x, grid_y);
                
                if (cell.occupancy == 100) {
                    score += 1.0; // Hit occupied cell
                } else if (cell.occupancy == 0) {
                    score -= 0.1; // Hit free cell (small penalty)
                }
                
                valid_points++;
            }
        }
        
        return valid_points > 0 ? score / valid_points : -1000.0;
    }
    
    void updateMap()
    {
        // Ray tracing for occupancy grid update
        for (size_t i = 0; i < current_scan_.ranges.size(); ++i) {
            if (std::isnan(current_scan_.ranges[i]) || std::isinf(current_scan_.ranges[i])) {
                continue;
            }
            
            double range = current_scan_.ranges[i];
            if (range < current_scan_.range_min || range > current_scan_.range_max) {
                continue;
            }
            
            double angle = current_scan_.angle_min + i * current_scan_.angle_increment;
            
            // Ray from robot position to scan point
            double local_x = range * cos(angle);
            double local_y = range * sin(angle);
            
            double end_x = current_pose_.x + local_x * cos(current_pose_.theta) - local_y * sin(current_pose_.theta);
            double end_y = current_pose_.y + local_x * sin(current_pose_.theta) + local_y * cos(current_pose_.theta);
            
            // Bresenham's line algorithm for ray tracing
            rayTrace(current_pose_.x, current_pose_.y, end_x, end_y, range < current_scan_.range_max);
        }
    }
    
    void rayTrace(double start_x, double start_y, double end_x, double end_y, bool hit_obstacle)
    {
        int start_grid_x, start_grid_y, end_grid_x, end_grid_y;
        occupancy_grid_->worldToGrid(start_x, start_y, start_grid_x, start_grid_y);
        occupancy_grid_->worldToGrid(end_x, end_y, end_grid_x, end_grid_y);
        
        // Bresenham's line algorithm
        int dx = abs(end_grid_x - start_grid_x);
        int dy = abs(end_grid_y - start_grid_y);
        int x = start_grid_x;
        int y = start_grid_y;
        int x_inc = (end_grid_x > start_grid_x) ? 1 : -1;
        int y_inc = (end_grid_y > start_grid_y) ? 1 : -1;
        int error = dx - dy;
        
        while (true) {
            if (occupancy_grid_->isValid(x, y)) {
                GridCell& cell = occupancy_grid_->getCell(x, y);
                
                // Update cell (free space along ray)
                if (x != end_grid_x || y != end_grid_y) {
                    cell.misses++;
                    cell.log_odds -= 0.4; // Free space update
                } else if (hit_obstacle) {
                    cell.hits++;
                    cell.log_odds += 0.9; // Occupied space update
                }
                
                // Clamp log odds
                cell.log_odds = std::clamp(cell.log_odds, -2.0, 2.0);
                
                // Update occupancy value
                if (cell.log_odds > 0.5) {
                    cell.occupancy = 100; // Occupied
                } else if (cell.log_odds < -0.5) {
                    cell.occupancy = 0;   // Free
                } else {
                    cell.occupancy = -1;  // Unknown
                }
            }
            
            if (x == end_grid_x && y == end_grid_y) break;
            
            int error2 = 2 * error;
            if (error2 > -dy) {
                error -= dy;
                x += x_inc;
            }
            if (error2 < dx) {
                error += dx;
                y += y_inc;
            }
        }
    }
    
    void updateMapLoop()
    {
        publishMap();
    }
    
    void slamLoop()
    {
        publishPose();
        publishCorrectedOdometry();
        publishHealthStatus();
        
        if (publish_tf_) {
            publishTransform();
        }
    }
    
    void publishMap()
    {
        auto map_msg = nav_msgs::msg::OccupancyGrid();
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = map_frame_;
        
        map_msg.info.resolution = occupancy_grid_->getResolution();
        map_msg.info.width = occupancy_grid_->getWidth();
        map_msg.info.height = occupancy_grid_->getHeight();
        map_msg.info.origin.position.x = occupancy_grid_->getOriginX();
        map_msg.info.origin.position.y = occupancy_grid_->getOriginY();
        map_msg.info.origin.position.z = 0.0;
        map_msg.info.origin.orientation.w = 1.0;
        
        map_msg.data.resize(map_width_ * map_height_);
        for (int y = 0; y < map_height_; ++y) {
            for (int x = 0; x < map_width_; ++x) {
                const GridCell& cell = occupancy_grid_->getCell(x, y);
                map_msg.data[y * map_width_ + x] = cell.occupancy;
            }
        }
        
        map_pub_->publish(map_msg);
    }
    
    void publishPose()
    {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = map_frame_;
        
        pose_msg.pose.pose.position.x = current_pose_.x;
        pose_msg.pose.pose.position.y = current_pose_.y;
        pose_msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, current_pose_.theta);
        pose_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Set covariance (simplified)
        for (int i = 0; i < 36; ++i) {
            pose_msg.pose.covariance[i] = 0.0;
        }
        pose_msg.pose.covariance[0] = 0.01;   // x
        pose_msg.pose.covariance[7] = 0.01;   // y
        pose_msg.pose.covariance[35] = 0.01;  // yaw
        
        pose_pub_->publish(pose_msg);
    }
    
    void publishCorrectedOdometry()
    {
        if (!odom_available_) return;
        
        auto odom_msg = current_odom_;
        odom_msg.header.frame_id = map_frame_;
        odom_msg.pose.pose.position.x = current_pose_.x;
        odom_msg.pose.pose.position.y = current_pose_.y;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, current_pose_.theta);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        corrected_odom_pub_->publish(odom_msg);
    }
    
    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = odom_frame_;
        
        // Calculate map -> odom transform
        if (odom_available_) {
            // Extract odom pose
            double odom_x = current_odom_.pose.pose.position.x;
            double odom_y = current_odom_.pose.pose.position.y;
            
            tf2::Quaternion odom_q;
            tf2::fromMsg(current_odom_.pose.pose.orientation, odom_q);
            tf2::Matrix3x3 odom_m(odom_q);
            double roll, pitch, odom_theta;
            odom_m.getRPY(roll, pitch, odom_theta);
            
            // Calculate transform
            double dx = current_pose_.x - odom_x;
            double dy = current_pose_.y - odom_y;
            double dtheta = normalizeAngle(current_pose_.theta - odom_theta);
            
            transform.transform.translation.x = dx;
            transform.transform.translation.y = dy;
            transform.transform.translation.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, dtheta);
            transform.transform.rotation = tf2::toMsg(q);
        } else {
            // Identity transform if no odom
            transform.transform.translation.x = current_pose_.x;
            transform.transform.translation.y = current_pose_.y;
            transform.transform.translation.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, current_pose_.theta);
            transform.transform.rotation = tf2::toMsg(q);
        }
        
        tf_broadcaster_.sendTransform(transform);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        // Remove component_name field - not part of SystemHealth message
        
        // Check data availability
        auto current_time = this->now();
        bool scan_timeout = (current_time - last_scan_time_).seconds() > 2.0;
        bool odom_timeout = (current_time - last_odom_time_).seconds() > 1.0;
        
        if (scan_timeout || odom_timeout) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(7001);
            health_msg.error_messages.push_back("Sensor data timeout");
        } else if (!first_scan_received_) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(7002);
            health_msg.error_messages.push_back("Waiting for scan data");
        } else {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(0);
            health_msg.error_messages.push_back("");
        }
        
        // Replace with cpu_temperature = 40.0; // See SystemHealth.msg // Placeholder
        // Replace with appropriate field - memory_usage not in SystemHealth.msg // Placeholder
        // Replace with specific temperature fields: cpu_temperature, gpu_temperature, motor_temperature // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    void saveMapService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::SaveMap::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::SaveMap::Response> response)
    {
        try {
            // In a real implementation, you would save the map to file
            // For now, just report success
            response->success = true;
            response->message = "Map saved to " + request->filename;
            
            RCLCPP_INFO(this->get_logger(), "Map save requested: %s", request->filename.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to save map: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Map save failed: %s", e.what());
        }
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::Service<tadeo_ecar_interfaces::srv::SaveMap>::SharedPtr save_map_service_;
    
    rclcpp::TimerBase::SharedPtr map_update_timer_;
    rclcpp::TimerBase::SharedPtr slam_timer_;
    
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double slam_frequency_;
    double map_update_frequency_;
    double map_resolution_;
    int map_width_, map_height_;
    double map_origin_x_, map_origin_y_;
    bool scan_matching_enabled_;
    int scan_matching_max_iterations_;
    double scan_matching_epsilon_;
    double scan_matching_max_range_;
    double min_translation_for_scan_;
    double min_rotation_for_scan_;
    bool publish_tf_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    
    // SLAM state
    std::unique_ptr<OccupancyGrid> occupancy_grid_;
    Pose2D current_pose_;
    Pose2D last_scan_pose_;
    
    // Sensor data
    sensor_msgs::msg::LaserScan current_scan_;
    nav_msgs::msg::Odometry current_odom_;
    
    // Status
    bool first_scan_received_;
    bool odom_available_;
    rclcpp::Time last_scan_time_;
    rclcpp::Time last_odom_time_;
};

} // namespace tadeo_ecar_slam

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_slam::GridSLAMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}