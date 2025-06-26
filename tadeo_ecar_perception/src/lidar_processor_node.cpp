#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <cmath>
#include <vector>

namespace tadeo_ecar_perception
{

struct Obstacle
{
    pcl::PointXYZ center;
    double radius;
    double height;
    std::string type;
    double confidence;
};

class LidarProcessorNode : public rclcpp::Node
{
public:
    LidarProcessorNode() : Node("lidar_processor_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&LidarProcessorNode::scanCallback, this, std::placeholders::_1));
        
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", 10,
            std::bind(&LidarProcessorNode::pointCloudCallback, this, std::placeholders::_1));
        
        // Publishers
        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan_filtered", 10);
        
        filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "points_filtered", 10);
        
        obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "perception/obstacles", 10);
        
        free_space_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "perception/free_space", 10);
        
        closest_obstacle_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "perception/closest_obstacle", 10);
        
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "perception/lidar_health", 10);
        
        // Processing timer
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / processing_frequency_)),
            std::bind(&LidarProcessorNode::processingLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "LiDAR Processor Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("processing.frequency", 10.0);
        this->declare_parameter("filtering.min_range", 0.1);
        this->declare_parameter("filtering.max_range", 30.0);
        this->declare_parameter("filtering.voxel_size", 0.05);
        this->declare_parameter("clustering.min_cluster_size", 10);
        this->declare_parameter("clustering.max_cluster_size", 500);
        this->declare_parameter("clustering.cluster_tolerance", 0.2);
        this->declare_parameter("obstacle.min_height", 0.1);
        this->declare_parameter("obstacle.max_height", 3.0);
        this->declare_parameter("obstacle.min_radius", 0.05);
        this->declare_parameter("obstacle.max_radius", 2.0);
        this->declare_parameter("safety.warning_distance", 2.0);
        this->declare_parameter("safety.emergency_distance", 0.5);
        
        processing_frequency_ = this->get_parameter("processing.frequency").as_double();
        min_range_ = this->get_parameter("filtering.min_range").as_double();
        max_range_ = this->get_parameter("filtering.max_range").as_double();
        voxel_size_ = this->get_parameter("filtering.voxel_size").as_double();
        min_cluster_size_ = this->get_parameter("clustering.min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("clustering.max_cluster_size").as_int();
        cluster_tolerance_ = this->get_parameter("clustering.cluster_tolerance").as_double();
        min_obstacle_height_ = this->get_parameter("obstacle.min_height").as_double();
        max_obstacle_height_ = this->get_parameter("obstacle.max_height").as_double();
        min_obstacle_radius_ = this->get_parameter("obstacle.min_radius").as_double();
        max_obstacle_radius_ = this->get_parameter("obstacle.max_radius").as_double();
        warning_distance_ = this->get_parameter("safety.warning_distance").as_double();
        emergency_distance_ = this->get_parameter("safety.emergency_distance").as_double();
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        current_scan_ = *msg;
        last_scan_time_ = this->now();
        
        // Process scan data
        processScan();
    }
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        current_cloud_msg_ = *msg;
        last_cloud_time_ = this->now();
        
        // Convert to PCL format
        pcl::fromROSMsg(*msg, current_cloud_);
        
        // Process point cloud
        processPointCloud();
    }
    
    void processScan()
    {
        if (current_scan_.ranges.empty()) return;
        
        auto filtered_scan = current_scan_;
        
        // Filter scan data
        for (size_t i = 0; i < filtered_scan.ranges.size(); ++i) {
            float& range = filtered_scan.ranges[i];
            
            // Remove invalid ranges
            if (std::isnan(range) || std::isinf(range)) {
                range = 0.0;
                continue;
            }
            
            // Apply range limits
            if (range < min_range_ || range > max_range_) {
                range = 0.0;
                continue;
            }
        }
        
        // Detect obstacles from scan
        detectObstaclesFromScan(filtered_scan);
        
        // Calculate free space
        calculateFreeSpace(filtered_scan);
        
        // Publish filtered scan
        filtered_scan_pub_->publish(filtered_scan);
    }
    
    void processPointCloud()
    {
        if (current_cloud_.empty()) return;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(current_cloud_));
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Filter point cloud
        filterPointCloud(cloud, filtered_cloud);
        
        // Detect obstacles from point cloud
        detectObstaclesFromCloud(filtered_cloud);
        
        // Publish filtered cloud
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header = current_cloud_msg_.header;
        filtered_cloud_pub_->publish(output_msg);
    }
    
    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
        // Range filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr range_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : input->points) {
            double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance >= min_range_ && distance <= max_range_) {
                range_filtered->push_back(point);
            }
        }
        
        // Voxel grid filtering
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(range_filtered);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.filter(*voxel_filtered);
        
        // Statistical outlier removal
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
        outlier_filter.setInputCloud(voxel_filtered);
        outlier_filter.setMeanK(50);
        outlier_filter.setStddevMulThresh(1.0);
        outlier_filter.filter(*output);
    }
    
    void detectObstaclesFromScan(const sensor_msgs::msg::LaserScan& scan)
    {
        std::vector<Obstacle> obstacles;
        
        // Simple clustering based on range discontinuities
        std::vector<std::vector<int>> clusters;
        std::vector<int> current_cluster;
        
        for (size_t i = 1; i < scan.ranges.size(); ++i) {
            if (scan.ranges[i] > 0 && scan.ranges[i-1] > 0) {
                double range_diff = std::abs(scan.ranges[i] - scan.ranges[i-1]);
                
                if (range_diff < cluster_tolerance_) {
                    current_cluster.push_back(i);
                } else {
                    if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
                        clusters.push_back(current_cluster);
                    }
                    current_cluster.clear();
                    current_cluster.push_back(i);
                }
            }
        }
        
        if (current_cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
            clusters.push_back(current_cluster);
        }
        
        // Convert clusters to obstacles
        for (const auto& cluster : clusters) {
            if (cluster.size() < static_cast<size_t>(min_cluster_size_) || 
                cluster.size() > static_cast<size_t>(max_cluster_size_)) continue;
            
            Obstacle obstacle;
            
            // Calculate cluster center
            double sum_x = 0, sum_y = 0;
            double min_range = std::numeric_limits<double>::max();
            double max_range = 0;
            
            for (int idx : cluster) {
                double angle = scan.angle_min + idx * scan.angle_increment;
                double range = scan.ranges[idx];
                
                sum_x += range * cos(angle);
                sum_y += range * sin(angle);
                min_range = std::min(min_range, range);
                max_range = std::max(max_range, range);
            }
            
            obstacle.center.x = sum_x / cluster.size();
            obstacle.center.y = sum_y / cluster.size();
            obstacle.center.z = 0.0;
            
            // Estimate obstacle properties
            obstacle.radius = (max_range - min_range) / 2.0;
            obstacle.height = 1.0; // Assume fixed height for 2D scan
            obstacle.type = classifyObstacle(obstacle);
            obstacle.confidence = std::min(1.0, static_cast<double>(cluster.size()) / 50.0);
            
            if (obstacle.radius >= min_obstacle_radius_ && obstacle.radius <= max_obstacle_radius_) {
                obstacles.push_back(obstacle);
            }
        }
        
        current_obstacles_ = obstacles;
    }
    
    void detectObstaclesFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        if (cloud->empty()) return;
        
        std::vector<Obstacle> obstacles;
        
        // Remove ground plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        removeGroundPlane(cloud, cloud_no_ground);
        
        // Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        euclideanClustering(cloud_no_ground, cluster_indices);
        
        // Convert clusters to obstacles
        for (const auto& indices : cluster_indices) {
            if (indices.indices.size() < static_cast<size_t>(min_cluster_size_) ||
                indices.indices.size() > static_cast<size_t>(max_cluster_size_)) continue;
            
            Obstacle obstacle;
            
            // Calculate cluster properties
            double sum_x = 0, sum_y = 0, sum_z = 0;
            double min_z = std::numeric_limits<double>::max();
            double max_z = std::numeric_limits<double>::lowest();
            
            for (int idx : indices.indices) {
                const auto& point = cloud_no_ground->points[idx];
                sum_x += point.x;
                sum_y += point.y;
                sum_z += point.z;
                min_z = std::min(min_z, static_cast<double>(point.z));
                max_z = std::max(max_z, static_cast<double>(point.z));
            }
            
            obstacle.center.x = sum_x / indices.indices.size();
            obstacle.center.y = sum_y / indices.indices.size();
            obstacle.center.z = sum_z / indices.indices.size();
            
            obstacle.height = max_z - min_z;
            obstacle.radius = calculateClusterRadius(cloud_no_ground, indices, obstacle.center);
            obstacle.type = classifyObstacle(obstacle);
            obstacle.confidence = std::min(1.0, static_cast<double>(indices.indices.size()) / 100.0);
            
            if (obstacle.height >= min_obstacle_height_ && obstacle.height <= max_obstacle_height_ &&
                obstacle.radius >= min_obstacle_radius_ && obstacle.radius <= max_obstacle_radius_) {
                obstacles.push_back(obstacle);
            }
        }
        
        current_obstacles_ = obstacles;
    }
    
    void removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr output)
    {
        // RANSAC plane segmentation
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        seg.setInputCloud(input);
        seg.segment(*inliers, *coefficients);
        
        // Extract non-ground points
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(input);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*output);
    }
    
    void euclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             std::vector<pcl::PointIndices>& cluster_indices)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
    }
    
    double calculateClusterRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                  const pcl::PointIndices& indices,
                                  const pcl::PointXYZ& center)
    {
        double max_distance = 0;
        for (int idx : indices.indices) {
            const auto& point = cloud->points[idx];
            double distance = sqrt(pow(point.x - center.x, 2) + 
                                   pow(point.y - center.y, 2) + 
                                   pow(point.z - center.z, 2));
            max_distance = std::max(max_distance, distance);
        }
        return max_distance;
    }
    
    std::string classifyObstacle(const Obstacle& obstacle)
    {
        // Simple classification based on size
        if (obstacle.height > 1.5 && obstacle.radius < 0.5) {
            return "person";
        } else if (obstacle.height > 1.0 && obstacle.radius > 1.0) {
            return "vehicle";
        } else if (obstacle.height < 0.5) {
            return "low_obstacle";
        } else {
            return "unknown";
        }
    }
    
    void calculateFreeSpace(const sensor_msgs::msg::LaserScan& scan)
    {
        auto free_space_msg = geometry_msgs::msg::PolygonStamped();
        free_space_msg.header = scan.header;
        
        // Simple free space calculation - find continuous ranges
        for (size_t i = 0; i < scan.ranges.size(); i += 10) { // Sample every 10th point
            if (scan.ranges[i] > 0 && scan.ranges[i] < max_range_) {
                double angle = scan.angle_min + i * scan.angle_increment;
                
                geometry_msgs::msg::Point32 point;
                point.x = scan.ranges[i] * cos(angle);
                point.y = scan.ranges[i] * sin(angle);
                point.z = 0.0;
                
                free_space_msg.polygon.points.push_back(point);
            }
        }
        
        free_space_pub_->publish(free_space_msg);
    }
    
    void publishObstacles()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        for (size_t i = 0; i < current_obstacles_.size(); ++i) {
            const auto& obstacle = current_obstacles_[i];
            
            auto marker = visualization_msgs::msg::Marker();
            marker.header.stamp = this->now();
            marker.header.frame_id = "laser_link";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = obstacle.center.x;
            marker.pose.position.y = obstacle.center.y;
            marker.pose.position.z = obstacle.center.z;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = obstacle.radius * 2;
            marker.scale.y = obstacle.radius * 2;
            marker.scale.z = obstacle.height;
            
            // Color based on type and distance
            double distance = sqrt(obstacle.center.x * obstacle.center.x + 
                                   obstacle.center.y * obstacle.center.y);
            
            if (distance < emergency_distance_) {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else if (distance < warning_distance_) {
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            marker.color.a = 0.7;
            
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            marker_array.markers.push_back(marker);
        }
        
        obstacles_pub_->publish(marker_array);
    }
    
    void publishClosestObstacle()
    {
        if (current_obstacles_.empty()) return;
        
        // Find closest obstacle
        double min_distance = std::numeric_limits<double>::max();
        const Obstacle* closest = nullptr;
        
        for (const auto& obstacle : current_obstacles_) {
            double distance = sqrt(obstacle.center.x * obstacle.center.x + 
                                   obstacle.center.y * obstacle.center.y);
            if (distance < min_distance) {
                min_distance = distance;
                closest = &obstacle;
            }
        }
        
        if (closest) {
            auto point_msg = geometry_msgs::msg::PointStamped();
            point_msg.header.stamp = this->now();
            point_msg.header.frame_id = "laser_link";
            point_msg.point.x = closest->center.x;
            point_msg.point.y = closest->center.y;
            point_msg.point.z = closest->center.z;
            
            closest_obstacle_pub_->publish(point_msg);
        }
    }
    
    void processingLoop()
    {
        publishObstacles();
        publishClosestObstacle();
        publishHealthStatus();
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = "laser_link";
        
        auto current_time = this->now();
        bool scan_timeout = (current_time - last_scan_time_).seconds() > 1.0;
        bool cloud_timeout = (current_time - last_cloud_time_).seconds() > 1.0;
        
        // Set LiDAR status
        if (scan_timeout && cloud_timeout) {
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.error_codes.push_back(2001);
            health_msg.error_messages.push_back("No LiDAR data received");
        } else {
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        }
        
        // Set system component statuses
        health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set motor statuses
        health_msg.front_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.front_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set temperatures
        health_msg.cpu_temperature = 50.0; // Placeholder
        health_msg.gpu_temperature = 47.0; // Placeholder
        health_msg.motor_temperature = 42.0; // Placeholder
        
        // Set diagnostic info
        health_msg.diagnostic_info = "LiDAR processor running";
        health_msg.uptime_seconds = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr free_space_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr closest_obstacle_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double processing_frequency_;
    double min_range_;
    double max_range_;
    double voxel_size_;
    int min_cluster_size_;
    int max_cluster_size_;
    double cluster_tolerance_;
    double min_obstacle_height_;
    double max_obstacle_height_;
    double min_obstacle_radius_;
    double max_obstacle_radius_;
    double warning_distance_;
    double emergency_distance_;
    
    // Current data
    sensor_msgs::msg::LaserScan current_scan_;
    sensor_msgs::msg::PointCloud2 current_cloud_msg_;
    pcl::PointCloud<pcl::PointXYZ> current_cloud_;
    std::vector<Obstacle> current_obstacles_;
    rclcpp::Time last_scan_time_;
    rclcpp::Time last_cloud_time_;
};

} // namespace tadeo_ecar_perception

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_perception::LidarProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}