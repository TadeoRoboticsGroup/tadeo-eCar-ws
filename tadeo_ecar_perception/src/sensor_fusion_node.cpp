#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <map>
#include <memory>

namespace tadeo_ecar_perception
{

struct FusedObstacle
{
    geometry_msgs::msg::Point position;
    double radius;
    double height;
    std::string type;
    double confidence;
    std::vector<std::string> detected_by; // sensors that detected this obstacle
    rclcpp::Time last_seen;
    int track_id;
};

struct SensorReading
{
    rclcpp::Time timestamp;
    std::string sensor_type;
    bool is_valid;
    double confidence;
};

class SensorFusionNode : public rclcpp::Node
{
public:
    SensorFusionNode() : Node("sensor_fusion_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeFusion();
        
        // Subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&SensorFusionNode::scanCallback, this, std::placeholders::_1));
        
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10,
            std::bind(&SensorFusionNode::cameraCallback, this, std::placeholders::_1));
        
        lidar_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "perception/obstacles", 10,
            std::bind(&SensorFusionNode::lidarObstaclesCallback, this, std::placeholders::_1));
        
        camera_objects_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "perception/detected_objects", 10,
            std::bind(&SensorFusionNode::cameraObjectsCallback, this, std::placeholders::_1));
        
        // Publishers
        fused_obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "perception/fused_obstacles", 10);
        
        nearest_obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "perception/nearest_obstacle", 10);
        
        sensor_status_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "perception/fusion_health", 10);
        
        confidence_map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "perception/confidence_map", 10);
        
        // Processing timer
        fusion_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / fusion_frequency_)),
            std::bind(&SensorFusionNode::fusionLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("fusion.frequency", 20.0);
        this->declare_parameter("fusion.max_association_distance", 1.0);
        this->declare_parameter("fusion.obstacle_timeout", 2.0);
        this->declare_parameter("fusion.min_confidence", 0.3);
        this->declare_parameter("fusion.track_threshold", 0.7);
        this->declare_parameter("weights.imu", 1.0);
        this->declare_parameter("weights.lidar", 0.8);
        this->declare_parameter("weights.camera", 0.6);
        this->declare_parameter("sensor_timeout.imu", 1.0);
        this->declare_parameter("sensor_timeout.lidar", 1.0);
        this->declare_parameter("sensor_timeout.camera", 2.0);
        
        fusion_frequency_ = this->get_parameter("fusion.frequency").as_double();
        max_association_distance_ = this->get_parameter("fusion.max_association_distance").as_double();
        obstacle_timeout_ = this->get_parameter("fusion.obstacle_timeout").as_double();
        min_confidence_ = this->get_parameter("fusion.min_confidence").as_double();
        track_threshold_ = this->get_parameter("fusion.track_threshold").as_double();
        
        sensor_weights_["imu"] = this->get_parameter("weights.imu").as_double();
        sensor_weights_["lidar"] = this->get_parameter("weights.lidar").as_double();
        sensor_weights_["camera"] = this->get_parameter("weights.camera").as_double();
        
        sensor_timeouts_["imu"] = this->get_parameter("sensor_timeout.imu").as_double();
        sensor_timeouts_["lidar"] = this->get_parameter("sensor_timeout.lidar").as_double();
        sensor_timeouts_["camera"] = this->get_parameter("sensor_timeout.camera").as_double();
    }
    
    void initializeFusion()
    {
        next_track_id_ = 1;
        
        // Initialize sensor status
        sensor_status_["imu"] = SensorReading{this->now(), "imu", false, 0.0};
        sensor_status_["lidar"] = SensorReading{this->now(), "lidar", false, 0.0};
        sensor_status_["camera"] = SensorReading{this->now(), "camera", false, 0.0};
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        updateSensorStatus("imu", msg->header.stamp, true, 0.9);
        
        // Store IMU data for fusion calculations
        current_imu_ = *msg;
        
        // Extract orientation for coordinate transformations
        tf2::fromMsg(msg->orientation, current_orientation_);
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        updateSensorStatus("lidar", msg->header.stamp, true, 0.8);
        current_scan_ = *msg;
    }
    
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        updateSensorStatus("camera", msg->header.stamp, true, 0.7);
        // Store camera data timestamp for fusion
        last_camera_time_ = msg->header.stamp;
    }
    
    void lidarObstaclesCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        lidar_obstacles_.clear();
        
        for (const auto& marker : msg->markers) {
            if (marker.action == visualization_msgs::msg::Marker::ADD) {
                FusedObstacle obstacle;
                obstacle.position = marker.pose.position;
                obstacle.radius = marker.scale.x / 2.0;
                obstacle.height = marker.scale.z;
                obstacle.type = "lidar_obstacle";
                obstacle.confidence = 0.8; // Base confidence for LiDAR
                obstacle.detected_by = {"lidar"};
                obstacle.last_seen = this->now();
                obstacle.track_id = -1; // Will be assigned during fusion
                
                lidar_obstacles_.push_back(obstacle);
            }
        }
    }
    
    void cameraObjectsCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        camera_objects_.clear();
        
        for (const auto& marker : msg->markers) {
            if (marker.action == visualization_msgs::msg::Marker::ADD) {
                FusedObstacle obstacle;
                obstacle.position = marker.pose.position;
                obstacle.radius = std::max(marker.scale.x, marker.scale.y) / 2.0;
                obstacle.height = marker.scale.z;
                obstacle.type = "camera_object";
                obstacle.confidence = 0.6; // Base confidence for camera
                obstacle.detected_by = {"camera"};
                obstacle.last_seen = this->now();
                obstacle.track_id = -1; // Will be assigned during fusion
                
                camera_objects_.push_back(obstacle);
            }
        }
    }
    
    void updateSensorStatus(const std::string& sensor_name, const rclcpp::Time& timestamp, 
                            bool is_valid, double confidence)
    {
        sensor_status_[sensor_name] = SensorReading{timestamp, sensor_name, is_valid, confidence};
    }
    
    void fusionLoop()
    {
        // Check sensor health
        updateSensorHealth();
        
        // Perform obstacle fusion
        fuseObstacles();
        
        // Remove expired obstacles
        removeExpiredObstacles();
        
        // Publish fused results
        publishFusedObstacles();
        publishNearestObstacle();
        publishConfidenceMap();
        publishHealthStatus();
    }
    
    void updateSensorHealth()
    {
        auto current_time = this->now();
        
        for (auto& [sensor_name, status] : sensor_status_) {
            double time_since_update = (current_time - status.timestamp).seconds();
            
            if (time_since_update > sensor_timeouts_[sensor_name]) {
                status.is_valid = false;
                status.confidence = 0.0;
            }
        }
    }
    
    void fuseObstacles()
    {
        // Combine all detected obstacles from different sensors
        std::vector<FusedObstacle> all_detections;
        
        // Add LiDAR obstacles
        for (auto obstacle : lidar_obstacles_) {
            all_detections.push_back(obstacle);
        }
        
        // Add camera objects
        for (auto obstacle : camera_objects_) {
            all_detections.push_back(obstacle);
        }
        
        // Data association and fusion
        std::vector<FusedObstacle> new_fused_obstacles;
        
        for (const auto& detection : all_detections) {
            // Try to associate with existing tracks
            int best_track_id = findBestTrackAssociation(detection);
            
            if (best_track_id >= 0) {
                // Update existing track
                updateExistingTrack(best_track_id, detection);
            } else {
                // Create new track
                FusedObstacle new_obstacle = detection;
                new_obstacle.track_id = next_track_id_++;
                new_obstacle.confidence = calculateFusedConfidence(detection);
                new_fused_obstacles.push_back(new_obstacle);
            }
        }
        
        // Add new obstacles to existing tracks
        for (const auto& new_obstacle : new_fused_obstacles) {
            fused_obstacles_[new_obstacle.track_id] = new_obstacle;
        }
    }
    
    int findBestTrackAssociation(const FusedObstacle& detection)
    {
        double min_distance = max_association_distance_;
        int best_track_id = -1;
        
        for (const auto& [track_id, existing_obstacle] : fused_obstacles_) {
            double distance = calculateDistance(detection.position, existing_obstacle.position);
            
            if (distance < min_distance) {
                min_distance = distance;
                best_track_id = track_id;
            }
        }
        
        return best_track_id;
    }
    
    void updateExistingTrack(int track_id, const FusedObstacle& detection)
    {
        auto& existing_obstacle = fused_obstacles_[track_id];
        
        // Update position using weighted average
        double weight = sensor_weights_[detection.detected_by[0]];
        double total_weight = weight + existing_obstacle.confidence;
        
        existing_obstacle.position.x = (existing_obstacle.position.x * existing_obstacle.confidence + 
                                        detection.position.x * weight) / total_weight;
        existing_obstacle.position.y = (existing_obstacle.position.y * existing_obstacle.confidence + 
                                        detection.position.y * weight) / total_weight;
        existing_obstacle.position.z = (existing_obstacle.position.z * existing_obstacle.confidence + 
                                        detection.position.z * weight) / total_weight;
        
        // Update other properties
        existing_obstacle.radius = std::max(existing_obstacle.radius, detection.radius);
        existing_obstacle.height = std::max(existing_obstacle.height, detection.height);
        
        // Merge detected_by lists
        for (const auto& sensor : detection.detected_by) {
            if (std::find(existing_obstacle.detected_by.begin(), 
                          existing_obstacle.detected_by.end(), sensor) == 
                existing_obstacle.detected_by.end()) {
                existing_obstacle.detected_by.push_back(sensor);
            }
        }
        
        // Update confidence
        existing_obstacle.confidence = calculateFusedConfidence(existing_obstacle);
        existing_obstacle.last_seen = this->now();
        
        // Update type based on sensor fusion
        existing_obstacle.type = determineFusedType(existing_obstacle);
    }
    
    double calculateFusedConfidence(const FusedObstacle& obstacle)
    {
        double total_confidence = 0.0;
        double total_weight = 0.0;
        
        for (const auto& sensor : obstacle.detected_by) {
            if (sensor_status_[sensor].is_valid) {
                double weight = sensor_weights_[sensor];
                total_confidence += sensor_status_[sensor].confidence * weight;
                total_weight += weight;
            }
        }
        
        if (total_weight > 0) {
            return std::min(1.0, total_confidence / total_weight);
        }
        
        return 0.0;
    }
    
    std::string determineFusedType(const FusedObstacle& obstacle)
    {
        // Prioritize classification based on sensor reliability
        if (std::find(obstacle.detected_by.begin(), obstacle.detected_by.end(), "camera") != 
            obstacle.detected_by.end()) {
            // Camera can provide more detailed classification
            return "classified_object";
        } else if (std::find(obstacle.detected_by.begin(), obstacle.detected_by.end(), "lidar") != 
                   obstacle.detected_by.end()) {
            // LiDAR provides accurate spatial information
            return "spatial_obstacle";
        } else {
            return "unknown_obstacle";
        }
    }
    
    double calculateDistance(const geometry_msgs::msg::Point& p1, 
                             const geometry_msgs::msg::Point& p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
    }
    
    void removeExpiredObstacles()
    {
        auto current_time = this->now();
        auto it = fused_obstacles_.begin();
        
        while (it != fused_obstacles_.end()) {
            double time_since_seen = (current_time - it->second.last_seen).seconds();
            
            if (time_since_seen > obstacle_timeout_ || it->second.confidence < min_confidence_) {
                it = fused_obstacles_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void publishFusedObstacles()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        for (const auto& [track_id, obstacle] : fused_obstacles_) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.stamp = this->now();
            marker.header.frame_id = "base_link";
            marker.id = track_id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position = obstacle.position;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = obstacle.radius * 2;
            marker.scale.y = obstacle.radius * 2;
            marker.scale.z = obstacle.height;
            
            // Color based on confidence and sensors
            if (obstacle.confidence > track_threshold_) {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            marker.color.a = obstacle.confidence;
            
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            
            // Add text annotation
            auto text_marker = marker;
            text_marker.id = track_id + 10000;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.pose.position.z += obstacle.height + 0.2;
            text_marker.scale.x = 0.3;
            text_marker.scale.y = 0.3;
            text_marker.scale.z = 0.3;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            std::string sensors_str;
            for (const auto& sensor : obstacle.detected_by) {
                if (!sensors_str.empty()) sensors_str += "+";
                sensors_str += sensor.substr(0, 1); // First letter
            }
            text_marker.text = "ID:" + std::to_string(track_id) + " [" + sensors_str + "] " + 
                               std::to_string(static_cast<int>(obstacle.confidence * 100)) + "%";
            
            marker_array.markers.push_back(marker);
            marker_array.markers.push_back(text_marker);
        }
        
        fused_obstacles_pub_->publish(marker_array);
    }
    
    void publishNearestObstacle()
    {
        if (fused_obstacles_.empty()) return;
        
        double min_distance = std::numeric_limits<double>::max();
        const FusedObstacle* nearest = nullptr;
        
        for (const auto& [track_id, obstacle] : fused_obstacles_) {
            double distance = sqrt(obstacle.position.x * obstacle.position.x + 
                                   obstacle.position.y * obstacle.position.y);
            
            if (distance < min_distance && obstacle.confidence > min_confidence_) {
                min_distance = distance;
                nearest = &obstacle;
            }
        }
        
        if (nearest) {
            auto point_msg = geometry_msgs::msg::PointStamped();
            point_msg.header.stamp = this->now();
            point_msg.header.frame_id = "base_link";
            point_msg.point = nearest->position;
            
            nearest_obstacle_pub_->publish(point_msg);
        }
    }
    
    void publishConfidenceMap()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Create confidence visualization for each active sensor
        int marker_id = 0;
        for (const auto& [sensor_name, status] : sensor_status_) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.stamp = this->now();
            marker.header.frame_id = "base_link";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = 0.0;
            marker.pose.position.y = marker_id * 0.5;
            marker.pose.position.z = 2.0;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            
            if (status.is_valid) {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            marker.color.a = 1.0;
            
            marker.text = sensor_name + ": " + 
                          (status.is_valid ? "OK" : "FAIL") + " (" + 
                          std::to_string(static_cast<int>(status.confidence * 100)) + "%)";
            
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            marker_array.markers.push_back(marker);
        }
        
        confidence_map_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = "base_link";
        
        health_msg.component_name = "sensor_fusion";
        
        // Calculate overall health based on sensor status
        int active_sensors = 0;
        int total_sensors = sensor_status_.size();
        
        for (const auto& [sensor_name, status] : sensor_status_) {
            if (status.is_valid) {
                active_sensors++;
            }
        }
        
        double health_ratio = static_cast<double>(active_sensors) / total_sensors;
        
        if (health_ratio >= 0.8) {
            health_msg.status = "OK";
            health_msg.error_code = 0;
            health_msg.error_message = "";
        } else if (health_ratio >= 0.5) {
            health_msg.status = "WARNING";
            health_msg.error_code = 4001;
            health_msg.error_message = "Some sensors unavailable";
        } else {
            health_msg.status = "ERROR";
            health_msg.error_code = 4002;
            health_msg.error_message = "Multiple sensor failures";
        }
        
        health_msg.cpu_usage = 30.0; // Placeholder
        health_msg.memory_usage = 20.0; // Placeholder
        health_msg.temperature = 45.0; // Placeholder
        
        sensor_status_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr lidar_obstacles_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr camera_objects_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fused_obstacles_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nearest_obstacle_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr sensor_status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr confidence_map_pub_;
    
    rclcpp::TimerInterface::SharedPtr fusion_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double fusion_frequency_;
    double max_association_distance_;
    double obstacle_timeout_;
    double min_confidence_;
    double track_threshold_;
    std::map<std::string, double> sensor_weights_;
    std::map<std::string, double> sensor_timeouts_;
    
    // State variables
    std::map<int, FusedObstacle> fused_obstacles_;
    std::vector<FusedObstacle> lidar_obstacles_;
    std::vector<FusedObstacle> camera_objects_;
    std::map<std::string, SensorReading> sensor_status_;
    int next_track_id_;
    
    sensor_msgs::msg::Imu current_imu_;
    sensor_msgs::msg::LaserScan current_scan_;
    tf2::Quaternion current_orientation_;
    rclcpp::Time last_camera_time_;
};

} // namespace tadeo_ecar_perception

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_perception::SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}