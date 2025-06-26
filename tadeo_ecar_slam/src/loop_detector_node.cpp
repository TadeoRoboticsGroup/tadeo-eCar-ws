#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include "tadeo_ecar_slam/slam_types.hpp"
#include <cmath>
#include <algorithm>
#include <memory>
#include <unordered_map>

namespace tadeo_ecar_slam
{

struct LoopDetectionResult
{
    int current_keyframe;
    int loop_keyframe;
    double confidence;
    Pose2D relative_pose;
    std::vector<std::pair<int, int>> correspondences;
    rclcpp::Time timestamp;
};

class LoopDetectorNode : public rclcpp::Node
{
public:
    LoopDetectorNode() : Node("loop_detector_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeDetector();
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&LoopDetectorNode::scanCallback, this, std::placeholders::_1));
        
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10,
            std::bind(&LoopDetectorNode::pointcloudCallback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "slam_pose", 10,
            std::bind(&LoopDetectorNode::poseCallback, this, std::placeholders::_1));
        
        keyframe_trigger_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "keyframe_trigger", 10,
            std::bind(&LoopDetectorNode::keyframeTriggerCallback, this, std::placeholders::_1));
        
        // Publishers
        loop_detection_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("loop_detections", 10);
        loop_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("loop_closures", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("slam/loop_detector_health", 10);
        
        // Timers
        detection_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / detection_frequency_)),
            std::bind(&LoopDetectorNode::detectionLoop, this));
        
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&LoopDetectorNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Loop Detector Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("detection_frequency", 2.0);
        this->declare_parameter("use_pointcloud", false);
        this->declare_parameter("min_keyframe_distance", 10);
        this->declare_parameter("max_search_distance", 5.0);
        this->declare_parameter("descriptor_match_threshold", 0.7);
        this->declare_parameter("geometric_verification_threshold", 0.8);
        this->declare_parameter("min_loop_closure_score", 0.6);
        this->declare_parameter("voxel_leaf_size", 0.1);
        this->declare_parameter("normal_search_radius", 0.3);
        this->declare_parameter("fpfh_search_radius", 0.5);
        this->declare_parameter("max_correspondence_distance", 1.0);
        this->declare_parameter("ransac_iterations", 1000);
        this->declare_parameter("ransac_threshold", 0.1);
        this->declare_parameter("enable_temporal_consistency", true);
        this->declare_parameter("consistency_window", 5);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("base_frame", "base_link");
        
        detection_frequency_ = this->get_parameter("detection_frequency").as_double();
        use_pointcloud_ = this->get_parameter("use_pointcloud").as_bool();
        min_keyframe_distance_ = this->get_parameter("min_keyframe_distance").as_int();
        max_search_distance_ = this->get_parameter("max_search_distance").as_double();
        descriptor_match_threshold_ = this->get_parameter("descriptor_match_threshold").as_double();
        geometric_verification_threshold_ = this->get_parameter("geometric_verification_threshold").as_double();
        min_loop_closure_score_ = this->get_parameter("min_loop_closure_score").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        normal_search_radius_ = this->get_parameter("normal_search_radius").as_double();
        fpfh_search_radius_ = this->get_parameter("fpfh_search_radius").as_double();
        max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
        ransac_iterations_ = this->get_parameter("ransac_iterations").as_int();
        ransac_threshold_ = this->get_parameter("ransac_threshold").as_double();
        enable_temporal_consistency_ = this->get_parameter("enable_temporal_consistency").as_bool();
        consistency_window_ = this->get_parameter("consistency_window").as_int();
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
    }
    
    void initializeDetector()
    {
        // Initialize keyframe storage
        keyframes_.clear();
        keyframe_descriptors_.clear();
        loop_detections_.clear();
        
        // Initialize state
        next_keyframe_id_ = 0;
        current_pose_available_ = false;
        last_detection_time_ = this->now();
        
        // Initialize statistics
        total_detections_ = 0;
        successful_detections_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Loop detector initialized");
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!use_pointcloud_) {
            current_scan_ = *msg;
            last_sensor_time_ = msg->header.stamp;
            
            // Convert scan to point cloud for processing
            convertScanToPointCloud(msg);
        }
    }
    
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (use_pointcloud_) {
            current_pointcloud_msg_ = *msg;
            last_sensor_time_ = msg->header.stamp;
            
            // Convert to PCL format
            pcl::fromROSMsg(*msg, *current_pointcloud_);
        }
    }
    
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pose_.theta = yaw;
        current_pose_.timestamp = msg->header.stamp;
        
        current_pose_available_ = true;
        last_pose_time_ = msg->header.stamp;
    }
    
    void keyframeTriggerCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 1) {
            int keyframe_id = msg->data[0];
            
            if (current_pose_available_ && current_pointcloud_->size() > 0) {
                createKeyframe(keyframe_id);
            }
        }
    }
    
    void convertScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        current_pointcloud_->clear();
        
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            if (std::isnan(scan->ranges[i]) || std::isinf(scan->ranges[i])) {
                continue;
            }
            
            double range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max) {
                continue;
            }
            
            double angle = scan->angle_min + i * scan->angle_increment;
            
            pcl::PointXYZ point;
            point.x = range * cos(angle);
            point.y = range * sin(angle);
            point.z = 0.0;
            
            current_pointcloud_->push_back(point);
        }
    }
    
    void createKeyframe(int keyframe_id)
    {
        KeyFrame keyframe;
        keyframe.id = keyframe_id;
        keyframe.pose = current_pose_;
        keyframe.timestamp = this->now();
        
        // Store point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Downsample point cloud
        if (current_pointcloud_->size() > 0) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(current_pointcloud_);
            voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_filter.filter(*filtered_cloud);
        }
        
        // Convert to scan points for storage
        for (const auto& point : filtered_cloud->points) {
            ScanPoint scan_point;
            scan_point.x = point.x;
            scan_point.y = point.y;
            scan_point.range = sqrt(point.x * point.x + point.y * point.y);
            scan_point.angle = atan2(point.y, point.x);
            scan_point.intensity = 0.0;
            scan_point.timestamp = keyframe.timestamp;
            keyframe.scan_points.push_back(scan_point);
        }
        
        // Compute descriptor
        Eigen::MatrixXd descriptor = computeDescriptor(filtered_cloud);
        keyframe.descriptor = descriptor;
        
        // Store keyframe
        keyframes_[keyframe_id] = keyframe;
        keyframe_descriptors_[keyframe_id] = descriptor;
        
        RCLCPP_DEBUG(this->get_logger(), "Created keyframe %d with %zu points and descriptor size %zdx%zd",
                     keyframe_id, keyframe.scan_points.size(), descriptor.rows(), descriptor.cols());
    }
    
    Eigen::MatrixXd computeDescriptor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        if (cloud->size() == 0) {
            return Eigen::MatrixXd::Zero(1, 1);
        }
        
        // Compute normals
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setRadiusSearch(normal_search_radius_);
        normal_estimator.compute(*normals);
        
        // Compute FPFH features
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features(new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimator;
        
        fpfh_estimator.setInputCloud(cloud);
        fpfh_estimator.setInputNormals(normals);
        fpfh_estimator.setSearchMethod(tree);
        fpfh_estimator.setRadiusSearch(fpfh_search_radius_);
        fpfh_estimator.compute(*fpfh_features);
        
        // Convert to Eigen matrix
        if (fpfh_features->size() == 0) {
            return Eigen::MatrixXd::Zero(1, 33);
        }
        
        Eigen::MatrixXd descriptor(fpfh_features->size(), 33);
        for (size_t i = 0; i < fpfh_features->size(); ++i) {
            for (int j = 0; j < 33; ++j) {
                descriptor(i, j) = fpfh_features->points[i].histogram[j];
            }
        }
        
        return descriptor;
    }
    
    void detectionLoop()
    {
        if (keyframes_.size() < 2) return;
        
        // Get the latest keyframe
        int latest_keyframe_id = keyframes_.rbegin()->first;
        
        // Look for loop closures
        detectLoopClosures(latest_keyframe_id);
        
        last_detection_time_ = this->now();
    }
    
    void detectLoopClosures(int current_keyframe_id)
    {
        if (keyframes_.find(current_keyframe_id) == keyframes_.end()) {
            return;
        }
        
        const KeyFrame& current_keyframe = keyframes_[current_keyframe_id];
        std::vector<LoopDetectionResult> candidates;
        
        // Search for potential loop closures
        for (const auto& pair : keyframes_) {
            int candidate_id = pair.first;
            const KeyFrame& candidate_keyframe = pair.second;
            
            // Skip recent keyframes
            if (current_keyframe_id - candidate_id < min_keyframe_distance_) {
                continue;
            }
            
            // Check spatial proximity
            double distance = poseDistance(current_keyframe.pose, candidate_keyframe.pose);
            if (distance > max_search_distance_) {
                continue;
            }
            
            // Perform descriptor matching
            double match_score = computeDescriptorMatch(current_keyframe_id, candidate_id);
            
            if (match_score > descriptor_match_threshold_) {
                // Perform geometric verification
                LoopDetectionResult result = performGeometricVerification(current_keyframe_id, candidate_id, match_score);
                
                if (result.confidence > geometric_verification_threshold_) {
                    candidates.push_back(result);
                }
            }
        }
        
        // Select best candidate
        if (!candidates.empty()) {
            auto best_candidate = std::max_element(candidates.begin(), candidates.end(),
                [](const LoopDetectionResult& a, const LoopDetectionResult& b) {
                    return a.confidence < b.confidence;
                });
            
            if (best_candidate->confidence > min_loop_closure_score_) {
                processLoopDetection(*best_candidate);
            }
        }
        
        total_detections_++;
    }
    
    double computeDescriptorMatch(int keyframe1_id, int keyframe2_id)
    {
        if (keyframe_descriptors_.find(keyframe1_id) == keyframe_descriptors_.end() ||
            keyframe_descriptors_.find(keyframe2_id) == keyframe_descriptors_.end()) {
            return 0.0;
        }
        
        const Eigen::MatrixXd& desc1 = keyframe_descriptors_[keyframe1_id];
        const Eigen::MatrixXd& desc2 = keyframe_descriptors_[keyframe2_id];
        
        if (desc1.rows() == 0 || desc2.rows() == 0) {
            return 0.0;
        }
        
        // Simple nearest neighbor matching
        int good_matches = 0;
        int total_matches = 0;
        
        for (int i = 0; i < desc1.rows(); ++i) {
            double best_distance = std::numeric_limits<double>::max();
            double second_best_distance = std::numeric_limits<double>::max();
            
            for (int j = 0; j < desc2.rows(); ++j) {
                double distance = (desc1.row(i) - desc2.row(j)).norm();
                
                if (distance < best_distance) {
                    second_best_distance = best_distance;
                    best_distance = distance;
                } else if (distance < second_best_distance) {
                    second_best_distance = distance;
                }
            }
            
            // Lowe's ratio test
            if (best_distance / second_best_distance < 0.8) {
                good_matches++;
            }
            total_matches++;
        }
        
        return total_matches > 0 ? static_cast<double>(good_matches) / total_matches : 0.0;
    }
    
    LoopDetectionResult performGeometricVerification(int current_id, int candidate_id, double match_score)
    {
        LoopDetectionResult result;
        result.current_keyframe = current_id;
        result.loop_keyframe = candidate_id;
        result.confidence = match_score;
        result.timestamp = this->now();
        
        if (keyframes_.find(current_id) == keyframes_.end() ||
            keyframes_.find(candidate_id) == keyframes_.end()) {
            result.confidence = 0.0;
            return result;
        }
        
        const KeyFrame& current_keyframe = keyframes_[current_id];
        const KeyFrame& candidate_keyframe = keyframes_[candidate_id];
        
        // Convert scan points to PCL clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& point : current_keyframe.scan_points) {
            current_cloud->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
        }
        
        for (const auto& point : candidate_keyframe.scan_points) {
            candidate_cloud->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
        }
        
        if (current_cloud->size() < 10 || candidate_cloud->size() < 10) {
            result.confidence = 0.0;
            return result;
        }
        
        // Estimate correspondences
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> correspondence_estimator;
        pcl::Correspondences correspondences;
        
        correspondence_estimator.setInputSource(current_cloud);
        correspondence_estimator.setInputTarget(candidate_cloud);
        correspondence_estimator.determineCorrespondences(correspondences, max_correspondence_distance_);
        
        if (correspondences.size() < 10) {
            result.confidence = 0.0;
            return result;
        }
        
        // RANSAC-based geometric verification
        double best_score = 0.0;
        Pose2D best_transform;
        
        for (int iter = 0; iter < ransac_iterations_; ++iter) {
            // Select random correspondences
            if (correspondences.size() < 3) break;
            
            std::vector<int> indices(correspondences.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::shuffle(indices.begin(), indices.end(), std::mt19937{std::random_device{}()});
            
            // Use first 3 correspondences to estimate transform
            std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> sample_pairs;
            for (int i = 0; i < 3 && i < static_cast<int>(indices.size()); ++i) {
                const auto& corr = correspondences[indices[i]];
                sample_pairs.push_back({current_cloud->points[corr.index_query], 
                                        candidate_cloud->points[corr.index_match]});
            }
            
            // Estimate transform (simplified)
            Pose2D transform = estimateTransform(sample_pairs);
            
            // Count inliers
            int inliers = 0;
            for (const auto& corr : correspondences) {
                const auto& p1 = current_cloud->points[corr.index_query];
                const auto& p2 = candidate_cloud->points[corr.index_match];
                
                // Transform p1 using estimated transform
                double transformed_x = p1.x * cos(transform.theta) - p1.y * sin(transform.theta) + transform.x;
                double transformed_y = p1.x * sin(transform.theta) + p1.y * cos(transform.theta) + transform.y;
                
                double error = sqrt(pow(transformed_x - p2.x, 2) + pow(transformed_y - p2.y, 2));
                
                if (error < ransac_threshold_) {
                    inliers++;
                }
            }
            
            double score = static_cast<double>(inliers) / correspondences.size();
            if (score > best_score) {
                best_score = score;
                best_transform = transform;
            }
        }
        
        // Store correspondences for visualization
        for (size_t i = 0; i < correspondences.size() && i < 100; ++i) { // Limit for performance
            result.correspondences.push_back({correspondences[i].index_query, correspondences[i].index_match});
        }
        
        result.relative_pose = best_transform;
        result.confidence = match_score * best_score; // Combine descriptor and geometric scores
        
        return result;
    }
    
    Pose2D estimateTransform(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& point_pairs)
    {
        Pose2D transform;
        
        if (point_pairs.size() < 2) {
            return transform; // Identity transform
        }
        
        // Simple centroid-based estimation
        double cx1 = 0.0, cy1 = 0.0, cx2 = 0.0, cy2 = 0.0;
        
        for (const auto& pair : point_pairs) {
            cx1 += pair.first.x;
            cy1 += pair.first.y;
            cx2 += pair.second.x;
            cy2 += pair.second.y;
        }
        
        cx1 /= point_pairs.size();
        cy1 /= point_pairs.size();
        cx2 /= point_pairs.size();
        cy2 /= point_pairs.size();
        
        // Estimate translation
        transform.x = cx2 - cx1;
        transform.y = cy2 - cy1;
        
        // Estimate rotation (simplified)
        double sum_cross = 0.0, sum_dot = 0.0;
        
        for (const auto& pair : point_pairs) {
            double dx1 = pair.first.x - cx1;
            double dy1 = pair.first.y - cy1;
            double dx2 = pair.second.x - cx2;
            double dy2 = pair.second.y - cy2;
            
            sum_cross += dx1 * dy2 - dy1 * dx2;
            sum_dot += dx1 * dx2 + dy1 * dy2;
        }
        
        transform.theta = atan2(sum_cross, sum_dot);
        
        return transform;
    }
    
    void processLoopDetection(const LoopDetectionResult& detection)
    {
        // Store detection result
        loop_detections_.push_back(detection);
        
        // Temporal consistency check
        if (enable_temporal_consistency_) {
            if (!isTemporallyConsistent(detection)) {
                RCLCPP_DEBUG(this->get_logger(), "Loop detection rejected due to temporal inconsistency");
                return;
            }
        }
        
        // Publish detection result
        publishLoopDetection(detection);
        
        // Visualize loop closure
        visualizeLoopClosure(detection);
        
        successful_detections_++;
        
        RCLCPP_INFO(this->get_logger(), "Loop closure detected: %d -> %d (confidence: %.3f)",
                    detection.loop_keyframe, detection.current_keyframe, detection.confidence);
    }
    
    bool isTemporallyConsistent(const LoopDetectionResult& detection)
    {
        // Check if similar loop closures have been detected recently
        int consistent_detections = 0;
        auto current_time = this->now();
        
        for (const auto& prev_detection : loop_detections_) {
            if ((current_time - prev_detection.timestamp).seconds() < consistency_window_) {
                if (detection.loop_keyframe == prev_detection.loop_keyframe ||
                    abs(detection.loop_keyframe - prev_detection.loop_keyframe) < 3) {
                    consistent_detections++;
                }
            }
        }
        
        return consistent_detections >= 2; // Need at least 2 consistent detections
    }
    
    void publishLoopDetection(const LoopDetectionResult& detection)
    {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data = {
            detection.current_keyframe,
            detection.loop_keyframe,
            static_cast<int>(detection.confidence * 1000), // Send as integer with 3 decimal precision
            static_cast<int>(detection.relative_pose.x * 1000),
            static_cast<int>(detection.relative_pose.y * 1000),
            static_cast<int>(detection.relative_pose.theta * 1000)
        };
        
        loop_detection_pub_->publish(msg);
    }
    
    void visualizeLoopClosure(const LoopDetectionResult& detection)
    {
        if (keyframes_.find(detection.current_keyframe) == keyframes_.end() ||
            keyframes_.find(detection.loop_keyframe) == keyframes_.end()) {
            return;
        }
        
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        const KeyFrame& current_kf = keyframes_[detection.current_keyframe];
        const KeyFrame& loop_kf = keyframes_[detection.loop_keyframe];
        
        // Line marker connecting the two keyframes
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = map_frame_;
        line_marker.header.stamp = this->now();
        line_marker.ns = "loop_closures";
        line_marker.id = detection.current_keyframe * 10000 + detection.loop_keyframe;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        
        geometry_msgs::msg::Point p1, p2;
        p1.x = current_kf.pose.x;
        p1.y = current_kf.pose.y;
        p1.z = 0.5;
        p2.x = loop_kf.pose.x;
        p2.y = loop_kf.pose.y;
        p2.z = 0.5;
        
        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        
        line_marker.scale.x = 0.1;
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 0.8;
        
        marker_array.markers.push_back(line_marker);
        
        // Text marker showing confidence
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = map_frame_;
        text_marker.header.stamp = this->now();
        text_marker.ns = "loop_confidence";
        text_marker.id = detection.current_keyframe * 10000 + detection.loop_keyframe;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        text_marker.pose.position.x = (p1.x + p2.x) / 2.0;
        text_marker.pose.position.y = (p1.y + p2.y) / 2.0;
        text_marker.pose.position.z = 1.0;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.text = std::to_string(static_cast<int>(detection.confidence * 100)) + "%";
        text_marker.scale.z = 0.3;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        marker_array.markers.push_back(text_marker);
        
        loop_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        // Remove component_name field - not part of SystemHealth message
        
        // Check system health
        auto current_time = this->now();
        bool sensor_timeout = (current_time - last_sensor_time_).seconds() > 3.0;
        bool pose_timeout = current_pose_available_ && (current_time - last_pose_time_).seconds() > 2.0;
        
        if (sensor_timeout || pose_timeout) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(10001);
            health_msg.error_messages.push_back("Sensor data timeout");
        } else if (!current_pose_available_) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(10002);
            health_msg.error_messages.push_back("Waiting for pose data");
        } else if (keyframes_.size() < min_keyframe_distance_) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(10003);
            health_msg.error_messages.push_back("Accumulating keyframes");
        } else {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(0);
            health_msg.error_messages.push_back("");
        }
        
        // Replace with cpu_temperature = 35.0; // See SystemHealth.msg // Placeholder
        // Replace with appropriate field - memory_usage not in SystemHealth.msg // Placeholder
        // Replace with specific temperature fields: cpu_temperature, gpu_temperature, motor_temperature // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr keyframe_trigger_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr loop_detection_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::TimerBase::SharedPtr detection_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double detection_frequency_;
    bool use_pointcloud_;
    int min_keyframe_distance_;
    double max_search_distance_;
    double descriptor_match_threshold_;
    double geometric_verification_threshold_;
    double min_loop_closure_score_;
    double voxel_leaf_size_;
    double normal_search_radius_;
    double fpfh_search_radius_;
    double max_correspondence_distance_;
    int ransac_iterations_;
    double ransac_threshold_;
    bool enable_temporal_consistency_;
    int consistency_window_;
    std::string map_frame_;
    std::string base_frame_;
    
    // Loop detection state
    std::map<int, KeyFrame> keyframes_;
    std::unordered_map<int, Eigen::MatrixXd> keyframe_descriptors_;
    std::vector<LoopDetectionResult> loop_detections_;
    
    int next_keyframe_id_;
    Pose2D current_pose_;
    bool current_pose_available_;
    
    // Sensor data
    sensor_msgs::msg::LaserScan current_scan_;
    sensor_msgs::msg::PointCloud2 current_pointcloud_msg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pointcloud_{new pcl::PointCloud<pcl::PointXYZ>};
    
    // Status
    rclcpp::Time last_detection_time_;
    rclcpp::Time last_sensor_time_;
    rclcpp::Time last_pose_time_;
    
    // Statistics
    int total_detections_;
    int successful_detections_;
};

} // namespace tadeo_ecar_slam

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_slam::LoopDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}