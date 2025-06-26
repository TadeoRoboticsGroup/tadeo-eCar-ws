#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <tadeo_ecar_msgs/msg/system_health.hpp>

namespace tadeo_ecar_localization
{

class MapMatcherNode : public rclcpp::Node
{
public:
    MapMatcherNode() : Node("map_matcher_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        
        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose/fused", 10,
            std::bind(&MapMatcherNode::poseCallback, this, std::placeholders::_1));
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10,
            std::bind(&MapMatcherNode::mapCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&MapMatcherNode::scanCallback, this, std::placeholders::_1));
        
        // Publishers
        corrected_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose/corrected", 10);
        
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "localization/map_matcher_health", 10);
        
        // Timer
        matching_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / matching_frequency_)),
            std::bind(&MapMatcherNode::matchingLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Map Matcher Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("matching_frequency", 10.0);
        this->declare_parameter("max_correction_distance", 0.5);
        this->declare_parameter("min_scan_match_score", 0.7);
        this->declare_parameter("enable_correction", true);
        
        matching_frequency_ = this->get_parameter("matching_frequency").as_double();
        max_correction_distance_ = this->get_parameter("max_correction_distance").as_double();
        min_scan_match_score_ = this->get_parameter("min_scan_match_score").as_double();
        enable_correction_ = this->get_parameter("enable_correction").as_bool();
        
        map_available_ = false;
        pose_available_ = false;
        scan_available_ = false;
    }
    
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        pose_available_ = true;
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = *msg;
        map_available_ = true;
        RCLCPP_INFO(this->get_logger(), "Map received: %dx%d", msg->info.width, msg->info.height);
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        current_scan_ = *msg;
        scan_available_ = true;
    }
    
    void matchingLoop()
    {
        if (!map_available_ || !pose_available_ || !scan_available_ || !enable_correction_) {
            publishHealthStatus("WAITING", "Waiting for map, pose, or scan data");
            return;
        }
        
        // Perform scan matching
        auto corrected_pose = performScanMatching();
        
        if (corrected_pose) {
            corrected_pose_pub_->publish(*corrected_pose);
            publishHealthStatus("OK", "");
        } else {
            publishHealthStatus("WARNING", "Scan matching failed");
        }
    }
    
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> performScanMatching()
    {
        // Simple grid-based scan matching implementation
        // This is a simplified version - in practice, you would use libraries like scan_tools
        
        double best_score = 0.0;
        geometry_msgs::msg::Pose best_pose = current_pose_.pose.pose;
        
        // Search around current pose
        double search_range = 0.2; // 20 cm search range
        double search_step = 0.05;  // 5 cm step size
        double angle_range = 0.1;   // ~6 degrees
        double angle_step = 0.02;   // ~1 degree step
        
        for (double dx = -search_range; dx <= search_range; dx += search_step) {
            for (double dy = -search_range; dy <= search_range; dy += search_step) {
                for (double dtheta = -angle_range; dtheta <= angle_range; dtheta += angle_step) {
                    
                    geometry_msgs::msg::Pose test_pose = current_pose_.pose.pose;
                    test_pose.position.x += dx;
                    test_pose.position.y += dy;
                    
                    // Update orientation
                    tf2::Quaternion q_orig, q_rot, q_new;
                    tf2::fromMsg(test_pose.orientation, q_orig);
                    q_rot.setRPY(0, 0, dtheta);
                    q_new = q_rot * q_orig;
                    test_pose.orientation = tf2::toMsg(q_new);
                    
                    double score = calculateScanMatchScore(test_pose);
                    
                    if (score > best_score) {
                        best_score = score;
                        best_pose = test_pose;
                    }
                }
            }
        }
        
        // Check if correction is significant and score is good enough
        double correction_distance = sqrt(
            pow(best_pose.position.x - current_pose_.pose.pose.position.x, 2) +
            pow(best_pose.position.y - current_pose_.pose.pose.position.y, 2)
        );
        
        if (best_score > min_scan_match_score_ && correction_distance < max_correction_distance_) {
            auto corrected_pose = current_pose_;
            corrected_pose.pose.pose = best_pose;
            corrected_pose.header.stamp = this->now();
            
            // Increase covariance slightly due to correction uncertainty
            for (int i = 0; i < 6; ++i) {
                corrected_pose.pose.covariance[i * 6 + i] *= 1.1;
            }
            
            return corrected_pose;
        }
        
        return std::nullopt;
    }
    
    double calculateScanMatchScore(const geometry_msgs::msg::Pose& pose)
    {
        if (current_scan_.ranges.empty()) return 0.0;
        
        int hits = 0;
        int total_points = 0;
        
        // Transform scan points to map frame and check occupancy
        for (size_t i = 0; i < current_scan_.ranges.size(); i += 5) { // Sample every 5th point for efficiency
            if (std::isnan(current_scan_.ranges[i]) || std::isinf(current_scan_.ranges[i])) {
                continue;
            }
            
            double range = current_scan_.ranges[i];
            if (range < current_scan_.range_min || range > current_scan_.range_max) {
                continue;
            }
            
            double angle = current_scan_.angle_min + i * current_scan_.angle_increment;
            
            // Convert to robot frame
            double robot_x = range * cos(angle);
            double robot_y = range * sin(angle);
            
            // Transform to map frame using test pose
            tf2::Quaternion q;
            tf2::fromMsg(pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            double map_x = pose.position.x + robot_x * cos(yaw) - robot_y * sin(yaw);
            double map_y = pose.position.y + robot_x * sin(yaw) + robot_y * cos(yaw);
            
            // Convert to grid coordinates
            int grid_x = static_cast<int>((map_x - current_map_.info.origin.position.x) / current_map_.info.resolution);
            int grid_y = static_cast<int>((map_y - current_map_.info.origin.position.y) / current_map_.info.resolution);
            
            // Check if point is within map bounds
            if (grid_x >= 0 && grid_x < static_cast<int>(current_map_.info.width) &&
                grid_y >= 0 && grid_y < static_cast<int>(current_map_.info.height)) {
                
                int index = grid_y * current_map_.info.width + grid_x;
                int8_t occupancy = current_map_.data[index];
                
                total_points++;
                
                // Check if scan point hits an occupied cell (occupied = 100, free = 0, unknown = -1)
                if (occupancy > 50) { // Occupied
                    hits++;
                }
            }
        }
        
        return total_points > 0 ? static_cast<double>(hits) / total_points : 0.0;
    }
    
    void publishHealthStatus(const std::string& status, const std::string& message)
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = "base_link";
        
        // Set system component statuses based on map matching health
        if (status == "OK") {
            health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        } else if (status == "WARNING" || status == "WAITING") {
            health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            if (status == "WARNING") {
                health_msg.error_codes.push_back(6005);
            } else {
                health_msg.error_codes.push_back(6006);
            }
            health_msg.error_messages.push_back(message);
        } else {
            health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.error_codes.push_back(6007);
            health_msg.error_messages.push_back(message);
        }
        
        // Set motor statuses
        health_msg.front_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.front_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set temperatures
        health_msg.cpu_temperature = 48.0; // Placeholder
        health_msg.gpu_temperature = 45.0; // Placeholder
        health_msg.motor_temperature = 40.0; // Placeholder
        
        // Set diagnostic info
        health_msg.diagnostic_info = "Map matcher running";
        health_msg.uptime_seconds = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr corrected_pose_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::TimerBase::SharedPtr matching_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double matching_frequency_;
    double max_correction_distance_;
    double min_scan_match_score_;
    bool enable_correction_;
    
    // Data
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
    nav_msgs::msg::OccupancyGrid current_map_;
    sensor_msgs::msg::LaserScan current_scan_;
    
    // Status
    bool map_available_;
    bool pose_available_;
    bool scan_available_;
};

} // namespace tadeo_ecar_localization

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_localization::MapMatcherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}