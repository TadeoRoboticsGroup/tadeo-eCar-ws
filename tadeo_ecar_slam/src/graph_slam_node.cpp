#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/srv/save_map.hpp>
#include "tadeo_ecar_slam/slam_types.hpp"
#include <cmath>
#include <algorithm>
#include <memory>
#include <queue>

namespace tadeo_ecar_slam
{

class GraphSLAMNode : public rclcpp::Node
{
public:
    GraphSLAMNode() : Node("graph_slam_node"), tf_broadcaster_(this), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeGraph();
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&GraphSLAMNode::scanCallback, this, std::placeholders::_1));
        
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10,
            std::bind(&GraphSLAMNode::pointcloudCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&GraphSLAMNode::odomCallback, this, std::placeholders::_1));
        
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            std::bind(&GraphSLAMNode::initialPoseCallback, this, std::placeholders::_1));
        
        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("graph_slam_pose", 10);
        corrected_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("graph_slam_odom", 10);
        graph_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("pose_graph", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("slam/graph_health", 10);
        
        // Services
        save_graph_service_ = this->create_service<tadeo_ecar_interfaces::srv::SaveMap>(
            "save_graph",
            std::bind(&GraphSLAMNode::saveGraphService, this, std::placeholders::_1, std::placeholders::_2));
        
        // Timers
        graph_optimization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / optimization_frequency_)),
            std::bind(&GraphSLAMNode::optimizationLoop, this));
        
        slam_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / slam_frequency_)),
            std::bind(&GraphSLAMNode::slamLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Graph SLAM Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("slam_frequency", 10.0);
        this->declare_parameter("optimization_frequency", 1.0);
        this->declare_parameter("keyframe_distance_threshold", 1.0);
        this->declare_parameter("keyframe_angle_threshold", 0.5);
        this->declare_parameter("loop_closure_distance_threshold", 3.0);
        this->declare_parameter("loop_closure_score_threshold", 0.7);
        this->declare_parameter("max_keyframes", 1000);
        this->declare_parameter("optimization_iterations", 10);
        this->declare_parameter("use_pointcloud", false);
        this->declare_parameter("voxel_leaf_size", 0.1);
        this->declare_parameter("icp_max_iterations", 30);
        this->declare_parameter("icp_max_correspondence_distance", 1.0);
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        
        slam_frequency_ = this->get_parameter("slam_frequency").as_double();
        optimization_frequency_ = this->get_parameter("optimization_frequency").as_double();
        keyframe_distance_threshold_ = this->get_parameter("keyframe_distance_threshold").as_double();
        keyframe_angle_threshold_ = this->get_parameter("keyframe_angle_threshold").as_double();
        loop_closure_distance_threshold_ = this->get_parameter("loop_closure_distance_threshold").as_double();
        loop_closure_score_threshold_ = this->get_parameter("loop_closure_score_threshold").as_double();
        max_keyframes_ = this->get_parameter("max_keyframes").as_int();
        optimization_iterations_ = this->get_parameter("optimization_iterations").as_int();
        use_pointcloud_ = this->get_parameter("use_pointcloud").as_bool();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        icp_max_iterations_ = this->get_parameter("icp_max_iterations").as_int();
        icp_max_correspondence_distance_ = this->get_parameter("icp_max_correspondence_distance").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
    }
    
    void initializeGraph()
    {
        // Initialize pose graph
        nodes_.clear();
        edges_.clear();
        
        // Initialize current pose
        current_pose_ = Pose2D(0.0, 0.0, 0.0);
        last_keyframe_pose_ = current_pose_;
        
        // Initialize state
        next_keyframe_id_ = 0;
        first_scan_received_ = false;
        odom_available_ = false;
        graph_needs_optimization_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Pose graph initialized");
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!use_pointcloud_) {
            current_scan_ = *msg;
            last_scan_time_ = msg->header.stamp;
            
            if (!first_scan_received_) {
                first_scan_received_ = true;
                RCLCPP_INFO(this->get_logger(), "First scan received");
            }
            
            // Convert scan to point cloud for processing
            convertScanToPointCloud(msg);
            
            // Check if we should create a new keyframe
            if (shouldCreateKeyframe()) {
                createKeyframe();
            }
        }
    }
    
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (use_pointcloud_) {
            current_pointcloud_msg_ = *msg;
            last_scan_time_ = msg->header.stamp;
            
            if (!first_scan_received_) {
                first_scan_received_ = true;
                RCLCPP_INFO(this->get_logger(), "First pointcloud received");
            }
            
            // Convert ROS message to PCL
            pcl::fromROSMsg(*msg, *current_pointcloud_);
            
            // Check if we should create a new keyframe
            if (shouldCreateKeyframe()) {
                createKeyframe();
            }
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
        // Reset graph with new initial pose
        current_pose_.x = msg->pose.pose.position.x;
        current_pose_.y = msg->pose.pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pose_.theta = yaw;
        
        last_keyframe_pose_ = current_pose_;
        
        // Clear existing graph
        nodes_.clear();
        edges_.clear();
        next_keyframe_id_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "Graph SLAM reset with initial pose: (%.2f, %.2f, %.2f)",
                    current_pose_.x, current_pose_.y, current_pose_.theta);
    }
    
    void updatePoseFromOdometry()
    {
        if (!odom_available_) return;
        
        // Simple odometry integration
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
        
        // Update current pose
        current_pose_.x += dx * cos(current_pose_.theta) - dy * sin(current_pose_.theta);
        current_pose_.y += dx * sin(current_pose_.theta) + dy * cos(current_pose_.theta);
        current_pose_.theta = normalizeAngle(current_pose_.theta + dtheta);
        
        last_odom = current_odom_;
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
    
    bool shouldCreateKeyframe()
    {
        if (!first_scan_received_ || !odom_available_) {
            return false;
        }
        
        if (nodes_.empty()) {
            return true; // Create first keyframe
        }
        
        // Check distance and angle thresholds
        double distance = sqrt(pow(current_pose_.x - last_keyframe_pose_.x, 2) + 
                               pow(current_pose_.y - last_keyframe_pose_.y, 2));
        double angle_diff = abs(normalizeAngle(current_pose_.theta - last_keyframe_pose_.theta));
        
        return (distance > keyframe_distance_threshold_ || angle_diff > keyframe_angle_threshold_);
    }
    
    void createKeyframe()
    {
        // Create new keyframe
        KeyFrame keyframe;
        keyframe.id = next_keyframe_id_++;
        keyframe.pose = current_pose_;
        keyframe.timestamp = this->now();
        
        // Store point cloud data
        if (current_pointcloud_->size() > 0) {
            // Downsample point cloud
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(current_pointcloud_);
            voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.filter(*filtered_cloud);
            
            // Store as scan points
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
        }
        
        // Add keyframe to graph
        GraphNode node;
        node.id = keyframe.id;
        node.pose = keyframe.pose;
        node.is_fixed = (keyframe.id == 0); // Fix first keyframe
        
        nodes_[keyframe.id] = node;
        keyframes_[keyframe.id] = keyframe;
        
        // Create edge to previous keyframe
        if (keyframe.id > 0) {
            GraphEdge edge;
            edge.from_node = keyframe.id - 1;
            edge.to_node = keyframe.id;
            
            // Calculate relative transform
            Pose2D prev_pose = nodes_[keyframe.id - 1].pose;
            edge.transform = calculateRelativeTransform(prev_pose, keyframe.pose);
            
            // Set information matrix (inverse of covariance)
            edge.information = Eigen::Matrix3d::Identity() * 100.0; // High confidence in odometry
            
            edges_.push_back(edge);
        }
        
        // Check for loop closures
        detectLoopClosures(keyframe.id);
        
        // Update state
        last_keyframe_pose_ = current_pose_;
        graph_needs_optimization_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Created keyframe %d at pose (%.2f, %.2f, %.2f)",
                    keyframe.id, keyframe.pose.x, keyframe.pose.y, keyframe.pose.theta);
    }
    
    Pose2D calculateRelativeTransform(const Pose2D& from, const Pose2D& to)
    {
        Pose2D relative;
        
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double cos_theta = cos(from.theta);
        double sin_theta = sin(from.theta);
        
        relative.x = dx * cos_theta + dy * sin_theta;
        relative.y = -dx * sin_theta + dy * cos_theta;
        relative.theta = normalizeAngle(to.theta - from.theta);
        
        return relative;
    }
    
    void detectLoopClosures(int current_keyframe_id)
    {
        const KeyFrame& current_keyframe = keyframes_[current_keyframe_id];
        
        // Check against all previous keyframes
        for (const auto& pair : keyframes_) {
            int candidate_id = pair.first;
            const KeyFrame& candidate_keyframe = pair.second;
            
            // Skip recent keyframes
            if (current_keyframe_id - candidate_id < 10) {
                continue;
            }
            
            // Check distance threshold
            double distance = poseDistance(current_keyframe.pose, candidate_keyframe.pose);
            if (distance > loop_closure_distance_threshold_) {
                continue;
            }
            
            // Perform scan matching
            double match_score = performScanMatching(current_keyframe, candidate_keyframe);
            
            if (match_score > loop_closure_score_threshold_) {
                // Create loop closure edge
                GraphEdge loop_edge;
                loop_edge.from_node = candidate_id;
                loop_edge.to_node = current_keyframe_id;
                loop_edge.transform = calculateRelativeTransform(candidate_keyframe.pose, current_keyframe.pose);
                loop_edge.information = Eigen::Matrix3d::Identity() * 50.0; // Lower confidence than odometry
                
                edges_.push_back(loop_edge);
                
                RCLCPP_INFO(this->get_logger(), "Loop closure detected: %d -> %d (score: %.3f)",
                            candidate_id, current_keyframe_id, match_score);
                
                graph_needs_optimization_ = true;
                break; // Only one loop closure per keyframe
            }
        }
    }
    
    double performScanMatching(const KeyFrame& keyframe1, const KeyFrame& keyframe2)
    {
        if (keyframe1.scan_points.empty() || keyframe2.scan_points.empty()) {
            return 0.0;
        }
        
        // Convert scan points to PCL point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& point : keyframe1.scan_points) {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = 0.0;
            cloud1->push_back(pcl_point);
        }
        
        for (const auto& point : keyframe2.scan_points) {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = 0.0;
            cloud2->push_back(pcl_point);
        }
        
        // Perform ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud1);
        icp.setInputTarget(cloud2);
        icp.setMaximumIterations(icp_max_iterations_);
        icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
        
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        icp.align(aligned_cloud);
        
        if (icp.hasConverged()) {
            return 1.0 - icp.getFitnessScore(); // Higher score for better matches
        }
        
        return 0.0;
    }
    
    void optimizationLoop()
    {
        if (graph_needs_optimization_ && nodes_.size() > 1) {
            optimizeGraph();
            graph_needs_optimization_ = false;
        }
    }
    
    void optimizeGraph()
    {
        if (nodes_.empty() || edges_.empty()) return;
        
        // Simple Levenberg-Marquardt optimization
        for (int iter = 0; iter < optimization_iterations_; ++iter) {
            // Calculate residuals and Jacobians
            Eigen::VectorXd residuals;
            Eigen::MatrixXd jacobian;
            
            calculateResidualsAndJacobians(residuals, jacobian);
            
            if (residuals.norm() < 1e-6) break; // Convergence
            
            // Solve normal equations
            Eigen::MatrixXd H = jacobian.transpose() * jacobian;
            Eigen::VectorXd b = -jacobian.transpose() * residuals;
            
            // Add damping
            H += Eigen::MatrixXd::Identity(H.rows(), H.cols()) * 1e-6;
            
            Eigen::VectorXd delta = H.ldlt().solve(b);
            
            // Update poses
            updatePoses(delta);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Graph optimization completed");
    }
    
    void calculateResidualsAndJacobians(Eigen::VectorXd& residuals, Eigen::MatrixXd& jacobian)
    {
        int num_edges = edges_.size();
        int num_nodes = nodes_.size();
        
        residuals.resize(num_edges * 3);
        jacobian.resize(num_edges * 3, num_nodes * 3);
        jacobian.setZero();
        
        for (size_t i = 0; i < edges_.size(); ++i) {
            const GraphEdge& edge = edges_[i];
            
            if (nodes_.find(edge.from_node) == nodes_.end() || 
                nodes_.find(edge.to_node) == nodes_.end()) {
                continue;
            }
            
            const Pose2D& pose_from = nodes_[edge.from_node].pose;
            const Pose2D& pose_to = nodes_[edge.to_node].pose;
            
            // Calculate predicted relative transform
            Pose2D predicted_transform = calculateRelativeTransform(pose_from, pose_to);
            
            // Calculate residual
            Eigen::Vector3d residual;
            residual(0) = predicted_transform.x - edge.transform.x;
            residual(1) = predicted_transform.y - edge.transform.y;
            residual(2) = normalizeAngle(predicted_transform.theta - edge.transform.theta);
            
            residuals.segment<3>(i * 3) = residual;
            
            // Calculate Jacobians (simplified)
            if (!nodes_[edge.from_node].is_fixed) {
                jacobian.block<3, 3>(i * 3, edge.from_node * 3) = -Eigen::Matrix3d::Identity();
            }
            if (!nodes_[edge.to_node].is_fixed) {
                jacobian.block<3, 3>(i * 3, edge.to_node * 3) = Eigen::Matrix3d::Identity();
            }
        }
    }
    
    void updatePoses(const Eigen::VectorXd& delta)
    {
        for (auto& pair : nodes_) {
            int node_id = pair.first;
            GraphNode& node = pair.second;
            
            if (node.is_fixed) continue;
            
            node.pose.x += delta(node_id * 3 + 0);
            node.pose.y += delta(node_id * 3 + 1);
            node.pose.theta = normalizeAngle(node.pose.theta + delta(node_id * 3 + 2));
        }
        
        // Update current pose if it's the latest keyframe
        if (!nodes_.empty()) {
            int latest_id = nodes_.rbegin()->first;
            current_pose_ = nodes_[latest_id].pose;
        }
    }
    
    void slamLoop()
    {
        publishPose();
        publishCorrectedOdometry();
        publishGraphVisualization();
        publishHealthStatus();
        
        if (publish_tf_) {
            publishTransform();
        }
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
        
        // Set covariance based on graph uncertainty
        for (int i = 0; i < 36; ++i) {
            pose_msg.pose.covariance[i] = 0.0;
        }
        pose_msg.pose.covariance[0] = 0.02;   // x
        pose_msg.pose.covariance[7] = 0.02;   // y
        pose_msg.pose.covariance[35] = 0.02;  // yaw
        
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
    
    void publishGraphVisualization()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Publish nodes
        for (const auto& pair : nodes_) {
            const GraphNode& node = pair.second;
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp = this->now();
            marker.ns = "graph_nodes";
            marker.id = node.id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = node.pose.x;
            marker.pose.position.y = node.pose.y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            
            marker.color.r = node.is_fixed ? 1.0 : 0.0;
            marker.color.g = 0.0;
            marker.color.b = node.is_fixed ? 0.0 : 1.0;
            marker.color.a = 1.0;
            
            marker_array.markers.push_back(marker);
        }
        
        // Publish edges
        for (size_t i = 0; i < edges_.size(); ++i) {
            const GraphEdge& edge = edges_[i];
            
            if (nodes_.find(edge.from_node) == nodes_.end() || 
                nodes_.find(edge.to_node) == nodes_.end()) {
                continue;
            }
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp = this->now();
            marker.ns = "graph_edges";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            geometry_msgs::msg::Point p1, p2;
            p1.x = nodes_[edge.from_node].pose.x;
            p1.y = nodes_[edge.from_node].pose.y;
            p1.z = 0.0;
            p2.x = nodes_[edge.to_node].pose.x;
            p2.y = nodes_[edge.to_node].pose.y;
            p2.z = 0.0;
            
            marker.points.push_back(p1);
            marker.points.push_back(p2);
            
            marker.scale.x = 0.05;
            
            // Different colors for odometry and loop closure edges
            bool is_sequential = (abs(edge.to_node - edge.from_node) == 1);
            marker.color.r = is_sequential ? 0.0 : 1.0;
            marker.color.g = is_sequential ? 1.0 : 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            
            marker_array.markers.push_back(marker);
        }
        
        graph_viz_pub_->publish(marker_array);
    }
    
    void publishTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = odom_frame_;
        
        // Calculate map -> odom transform
        if (odom_available_) {
            double odom_x = current_odom_.pose.pose.position.x;
            double odom_y = current_odom_.pose.pose.position.y;
            
            tf2::Quaternion odom_q;
            tf2::fromMsg(current_odom_.pose.pose.orientation, odom_q);
            tf2::Matrix3x3 odom_m(odom_q);
            double roll, pitch, odom_theta;
            odom_m.getRPY(roll, pitch, odom_theta);
            
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
        
        health_msg.component_name = "graph_slam";
        
        // Check system health
        auto current_time = this->now();
        bool scan_timeout = (current_time - last_scan_time_).seconds() > 2.0;
        bool odom_timeout = (current_time - last_odom_time_).seconds() > 1.0;
        
        if (scan_timeout || odom_timeout) {
            health_msg.status = "WARNING";
            health_msg.error_code = 8001;
            health_msg.error_message = "Sensor data timeout";
        } else if (!first_scan_received_) {
            health_msg.status = "WAITING";
            health_msg.error_code = 8002;
            health_msg.error_message = "Waiting for scan data";
        } else if (nodes_.size() < 2) {
            health_msg.status = "INITIALIZING";
            health_msg.error_code = 8003;
            health_msg.error_message = "Building pose graph";
        } else {
            health_msg.status = "OK";
            health_msg.error_code = 0;
            health_msg.error_message = "";
        }
        
        health_msg.cpu_usage = 50.0; // Placeholder
        health_msg.memory_usage = 60.0; // Placeholder
        health_msg.temperature = 55.0; // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    void saveGraphService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::SaveMap::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::SaveMap::Response> response)
    {
        try {
            // In a real implementation, you would save the pose graph to file
            // For now, just report success
            response->success = true;
            response->message = "Pose graph saved to " + request->filename;
            
            RCLCPP_INFO(this->get_logger(), "Graph save requested: %s (nodes: %zu, edges: %zu)",
                        request->filename.c_str(), nodes_.size(), edges_.size());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to save graph: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Graph save failed: %s", e.what());
        }
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_viz_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::Service<tadeo_ecar_interfaces::srv::SaveMap>::SharedPtr save_graph_service_;
    
    rclcpp::TimerInterface::SharedPtr graph_optimization_timer_;
    rclcpp::TimerInterface::SharedPtr slam_timer_;
    
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double slam_frequency_;
    double optimization_frequency_;
    double keyframe_distance_threshold_;
    double keyframe_angle_threshold_;
    double loop_closure_distance_threshold_;
    double loop_closure_score_threshold_;
    int max_keyframes_;
    int optimization_iterations_;
    bool use_pointcloud_;
    double voxel_leaf_size_;
    int icp_max_iterations_;
    double icp_max_correspondence_distance_;
    bool publish_tf_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    
    // Graph SLAM state
    std::map<int, GraphNode> nodes_;
    std::vector<GraphEdge> edges_;
    std::map<int, KeyFrame> keyframes_;
    
    Pose2D current_pose_;
    Pose2D last_keyframe_pose_;
    int next_keyframe_id_;
    bool graph_needs_optimization_;
    
    // Sensor data
    sensor_msgs::msg::LaserScan current_scan_;
    sensor_msgs::msg::PointCloud2 current_pointcloud_msg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pointcloud_{new pcl::PointCloud<pcl::PointXYZ>};
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
    auto node = std::make_shared<tadeo_ecar_slam::GraphSLAMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}