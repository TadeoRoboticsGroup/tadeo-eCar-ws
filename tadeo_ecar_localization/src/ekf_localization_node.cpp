#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include "tadeo_ecar_localization/kalman_filter_base.hpp"
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <cmath>

namespace tadeo_ecar_localization
{

class ExtendedKalmanFilter : public KalmanFilterBase
{
public:
    ExtendedKalmanFilter() : KalmanFilterBase(STATE_SIZE, 6)
    {
        // Initialize state transition matrix F
        F_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
        
        // Initialize control input matrix B
        B_ = Eigen::MatrixXd::Zero(STATE_SIZE, 2);
        B_(STATE_VX, 0) = 1.0; // Linear acceleration control
        B_(STATE_VYAW, 1) = 1.0; // Angular acceleration control
    }

    void predict(double dt) override
    {
        // Update state transition matrix with current dt
        updateStateTransitionMatrix(dt);
        
        // Prediction step: x = F * x + B * u
        // For simplicity, we assume u = 0 (no control input in this implementation)
        state_ = F_ * state_;
        
        // Update process noise matrix based on dt
        updateProcessNoiseMatrix(dt);
        
        // Covariance prediction: P = F * P * F^T + Q
        covariance_ = F_ * covariance_ * F_.transpose() + process_noise_;
    }

    void update(const Eigen::VectorXd& measurement, const Eigen::MatrixXd& measurement_matrix) override
    {
        // Calculate innovation
        Eigen::VectorXd y = measurement - measurement_matrix * state_;
        
        // Calculate innovation covariance
        Eigen::MatrixXd S = measurement_matrix * covariance_ * measurement_matrix.transpose() + measurement_noise_;
        
        // Calculate Kalman gain
        Eigen::MatrixXd K = covariance_ * measurement_matrix.transpose() * S.inverse();
        
        // Update state
        state_ = state_ + K * y;
        
        // Update covariance (Joseph form for numerical stability)
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
        covariance_ = (I - K * measurement_matrix) * covariance_ * (I - K * measurement_matrix).transpose() + 
                      K * measurement_noise_ * K.transpose();
        
        // Normalize yaw angle
        normalizeYaw();
    }

private:
    void updateStateTransitionMatrix(double dt)
    {
        F_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
        
        // Position integration
        F_(STATE_X, STATE_VX) = dt;
        F_(STATE_Y, STATE_VY) = dt;
        F_(STATE_YAW, STATE_VYAW) = dt;
        
        // Velocity integration (with acceleration)
        F_(STATE_VX, STATE_AX) = dt;
        F_(STATE_VY, STATE_AY) = dt;
        
        // Second-order position integration
        F_(STATE_X, STATE_AX) = 0.5 * dt * dt;
        F_(STATE_Y, STATE_AY) = 0.5 * dt * dt;
    }
    
    void updateProcessNoiseMatrix(double dt)
    {
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        
        // Process noise values
        double pos_noise = 0.01;
        double vel_noise = 0.1;
        double acc_noise = 1.0;
        double yaw_noise = 0.01;
        double yaw_rate_noise = 0.1;
        
        process_noise_ = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
        
        // Position noise
        process_noise_(STATE_X, STATE_X) = pos_noise * dt4 / 4.0;
        process_noise_(STATE_Y, STATE_Y) = pos_noise * dt4 / 4.0;
        process_noise_(STATE_YAW, STATE_YAW) = yaw_noise * dt2;
        
        // Velocity noise
        process_noise_(STATE_VX, STATE_VX) = vel_noise * dt2;
        process_noise_(STATE_VY, STATE_VY) = vel_noise * dt2;
        process_noise_(STATE_VYAW, STATE_VYAW) = yaw_rate_noise * dt2;
        
        // Acceleration noise
        process_noise_(STATE_AX, STATE_AX) = acc_noise * dt2;
        process_noise_(STATE_AY, STATE_AY) = acc_noise * dt2;
        
        // Cross-correlations
        process_noise_(STATE_X, STATE_VX) = process_noise_(STATE_VX, STATE_X) = pos_noise * dt3 / 2.0;
        process_noise_(STATE_Y, STATE_VY) = process_noise_(STATE_VY, STATE_Y) = pos_noise * dt3 / 2.0;
        process_noise_(STATE_X, STATE_AX) = process_noise_(STATE_AX, STATE_X) = pos_noise * dt2 / 2.0;
        process_noise_(STATE_Y, STATE_AY) = process_noise_(STATE_AY, STATE_Y) = pos_noise * dt2 / 2.0;
        process_noise_(STATE_VX, STATE_AX) = process_noise_(STATE_AX, STATE_VX) = vel_noise * dt;
        process_noise_(STATE_VY, STATE_AY) = process_noise_(STATE_AY, STATE_VY) = vel_noise * dt;
    }
    
    void normalizeYaw()
    {
        while (state_(STATE_YAW) > M_PI) state_(STATE_YAW) -= 2.0 * M_PI;
        while (state_(STATE_YAW) < -M_PI) state_(STATE_YAW) += 2.0 * M_PI;
    }

    Eigen::MatrixXd F_; // State transition matrix
    Eigen::MatrixXd B_; // Control input matrix
};

class EKFLocalizationNode : public rclcpp::Node
{
public:
    EKFLocalizationNode() : Node("ekf_localization_node"), tf_broadcaster_(this)
    {
        loadParameters();
        initializeFilter();
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&EKFLocalizationNode::odomCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&EKFLocalizationNode::imuCallback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            std::bind(&EKFLocalizationNode::initialPoseCallback, this, std::placeholders::_1));
        
        // Publishers
        filtered_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10);
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "pose", 10);
        
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "localization/ekf_health", 10);
        
        // Timer for filter processing
        filter_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / filter_frequency_)),
            std::bind(&EKFLocalizationNode::filterLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "EKF Localization Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("filter_frequency", 50.0);
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_link_frame", "base_link");
        this->declare_parameter("use_imu", true);
        this->declare_parameter("use_odom", true);
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_yaw", 0.0);
        
        filter_frequency_ = this->get_parameter("filter_frequency").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_link_frame_ = this->get_parameter("base_link_frame").as_string();
        use_imu_ = this->get_parameter("use_imu").as_bool();
        use_odom_ = this->get_parameter("use_odom").as_bool();
        
        double initial_x = this->get_parameter("initial_x").as_double();
        double initial_y = this->get_parameter("initial_y").as_double();
        double initial_yaw = this->get_parameter("initial_yaw").as_double();
        
        initial_state_ << initial_x, initial_y, initial_yaw, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    
    void initializeFilter()
    {
        ekf_ = std::make_unique<ExtendedKalmanFilter>();
        
        // Set initial state
        ekf_->setState(initial_state_);
        
        // Set initial covariance
        Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
        initial_covariance.diagonal() << 1.0, 1.0, 0.1, 0.5, 0.5, 0.1, 1.0, 1.0;
        ekf_->setCovariance(initial_covariance);
        
        // Set measurement noise
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6);
        R.diagonal() << 0.1, 0.1, 0.05, 0.1, 0.1, 0.05; // x, y, yaw, vx, vy, vyaw
        ekf_->setMeasurementNoise(R);
        
        last_prediction_time_ = this->now();
        filter_initialized_ = true;
        
        RCLCPP_INFO(this->get_logger(), "EKF filter initialized");
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!use_odom_ || !filter_initialized_) return;
        
        // Create measurement vector [x, y, yaw, vx, vy, vyaw]
        Eigen::VectorXd measurement(6);
        
        // Extract position and orientation
        measurement(0) = msg->pose.pose.position.x;
        measurement(1) = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        measurement(2) = yaw;
        
        // Extract velocities
        measurement(3) = msg->twist.twist.linear.x;
        measurement(4) = msg->twist.twist.linear.y;
        measurement(5) = msg->twist.twist.angular.z;
        
        // Create measurement matrix (observing all states except accelerations)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, STATE_SIZE);
        H(0, STATE_X) = 1.0;
        H(1, STATE_Y) = 1.0;
        H(2, STATE_YAW) = 1.0;
        H(3, STATE_VX) = 1.0;
        H(4, STATE_VY) = 1.0;
        H(5, STATE_VYAW) = 1.0;
        
        // Store measurement for processing
        SensorMeasurement odom_measurement;
        odom_measurement.data = measurement;
        odom_measurement.covariance = H; // Store H matrix here for simplicity
        odom_measurement.type = MEASUREMENT_ODOM;
        odom_measurement.timestamp = msg->header.stamp;
        odom_measurement.frame_id = msg->header.frame_id;
        
        measurement_queue_.push_back(odom_measurement);
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!use_imu_ || !filter_initialized_) return;
        
        // Create measurement vector [ax, ay] (linear accelerations)
        Eigen::VectorXd measurement(2);
        measurement(0) = msg->linear_acceleration.x;
        measurement(1) = msg->linear_acceleration.y;
        
        // Create measurement matrix (observing accelerations)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, STATE_SIZE);
        H(0, STATE_AX) = 1.0;
        H(1, STATE_AY) = 1.0;
        
        // Store measurement for processing
        SensorMeasurement imu_measurement;
        imu_measurement.data = measurement;
        imu_measurement.covariance = H; // Store H matrix here for simplicity
        imu_measurement.type = MEASUREMENT_IMU;
        imu_measurement.timestamp = msg->header.stamp;
        imu_measurement.frame_id = msg->header.frame_id;
        
        measurement_queue_.push_back(imu_measurement);
    }
    
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Reset filter with new initial pose
        Eigen::VectorXd new_state = initial_state_;
        
        new_state(STATE_X) = msg->pose.pose.position.x;
        new_state(STATE_Y) = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        new_state(STATE_YAW) = yaw;
        
        ekf_->setState(new_state);
        
        // Set covariance from message
        Eigen::MatrixXd new_covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                if (i < 3 && j < 3) {
                    new_covariance(i, j) = msg->pose.covariance[i * 6 + j];
                }
            }
        }
        ekf_->setCovariance(new_covariance);
        
        RCLCPP_INFO(this->get_logger(), "Filter reset with initial pose: (%.2f, %.2f, %.2f)",
                    new_state(STATE_X), new_state(STATE_Y), new_state(STATE_YAW));
    }
    
    void filterLoop()
    {
        if (!filter_initialized_) return;
        
        auto current_time = this->now();
        double dt = (current_time - last_prediction_time_).seconds();
        
        if (dt > 0.0 && dt < 1.0) { // Reasonable time step
            // Prediction step
            ekf_->predict(dt);
            
            // Process measurements
            processMeasurements();
            
            // Publish results
            publishResults(current_time);
            
            // Publish health status
            publishHealthStatus();
        }
        
        last_prediction_time_ = current_time;
    }
    
    void processMeasurements()
    {
        // Process all queued measurements
        for (const auto& measurement : measurement_queue_) {
            if (measurement.type == MEASUREMENT_ODOM) {
                // Create measurement matrix for odometry
                Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, STATE_SIZE);
                H(0, STATE_X) = 1.0;
                H(1, STATE_Y) = 1.0;
                H(2, STATE_YAW) = 1.0;
                H(3, STATE_VX) = 1.0;
                H(4, STATE_VY) = 1.0;
                H(5, STATE_VYAW) = 1.0;
                
                ekf_->update(measurement.data, H);
            } else if (measurement.type == MEASUREMENT_IMU) {
                // Create measurement matrix for IMU
                Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, STATE_SIZE);
                H(0, STATE_AX) = 1.0;
                H(1, STATE_AY) = 1.0;
                
                ekf_->update(measurement.data, H);
            }
        }
        
        // Clear processed measurements
        measurement_queue_.clear();
    }
    
    void publishResults(const rclcpp::Time& timestamp)
    {
        const Eigen::VectorXd& state = ekf_->getState();
        const Eigen::MatrixXd& covariance = ekf_->getCovariance();
        
        // Publish filtered odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = map_frame_;
        odom_msg.child_frame_id = base_link_frame_;
        
        // Set pose
        odom_msg.pose.pose.position.x = state(STATE_X);
        odom_msg.pose.pose.position.y = state(STATE_Y);
        odom_msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, state(STATE_YAW));
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Set twist
        odom_msg.twist.twist.linear.x = state(STATE_VX);
        odom_msg.twist.twist.linear.y = state(STATE_VY);
        odom_msg.twist.twist.angular.z = state(STATE_VYAW);
        
        // Set covariances
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                if (i < 3 && j < 3) {
                    odom_msg.pose.covariance[i * 6 + j] = covariance(i, j);
                } else if (i >= 3 && j >= 3 && i < 6 && j < 6) {
                    odom_msg.twist.covariance[i * 6 + j] = covariance(i, j);
                }
            }
        }
        
        filtered_odom_pub_->publish(odom_msg);
        
        // Publish pose
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose;
        pose_pub_->publish(pose_msg);
        
        // Publish transform
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = timestamp;
            transform.header.frame_id = map_frame_;
            transform.child_frame_id = odom_frame_;
            
            transform.transform.translation.x = state(STATE_X);
            transform.transform.translation.y = state(STATE_Y);
            transform.transform.translation.z = 0.0;
            transform.transform.rotation = odom_msg.pose.pose.orientation;
            
            tf_broadcaster_.sendTransform(transform);
        }
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_link_frame_;
        
        // Check filter health based on covariance trace
        const Eigen::MatrixXd& cov = ekf_->getCovariance();
        double trace = cov.trace();
        
        // Set system component statuses based on filter health
        if (trace > 100.0) {
            health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.error_codes.push_back(6001);
            health_msg.error_messages.push_back("High uncertainty in localization");
        } else if (trace > 1000.0) {
            health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.error_codes.push_back(6002);
            health_msg.error_messages.push_back("Very high uncertainty - filter may have diverged");
        } else {
            health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        }
        
        // Set motor statuses
        health_msg.front_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.front_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set temperatures
        health_msg.cpu_temperature = 45.0; // Placeholder
        health_msg.gpu_temperature = 42.0; // Placeholder
        health_msg.motor_temperature = 38.0; // Placeholder
        
        // Set diagnostic info
        health_msg.diagnostic_info = "EKF localization running";
        health_msg.uptime_seconds = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::TimerBase::SharedPtr filter_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Parameters
    double filter_frequency_;
    bool publish_tf_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_link_frame_;
    bool use_imu_;
    bool use_odom_;
    
    // Filter
    std::unique_ptr<ExtendedKalmanFilter> ekf_;
    Eigen::VectorXd initial_state_;
    bool filter_initialized_;
    rclcpp::Time last_prediction_time_;
    
    // Measurements
    std::vector<SensorMeasurement> measurement_queue_;
};

} // namespace tadeo_ecar_localization

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_localization::EKFLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}