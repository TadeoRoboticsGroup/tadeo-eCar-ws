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
#include <vector>

namespace tadeo_ecar_localization
{

class UnscentedKalmanFilter : public KalmanFilterBase
{
public:
    UnscentedKalmanFilter() : KalmanFilterBase(STATE_SIZE, 6)
    {
        // UKF parameters
        alpha_ = 0.001;
        beta_ = 2.0;
        kappa_ = 0.0;
        lambda_ = alpha_ * alpha_ * (state_size_ + kappa_) - state_size_;
        
        // Calculate weights
        calculateWeights();
        
        // Initialize sigma points
        sigma_points_ = Eigen::MatrixXd::Zero(state_size_, 2 * state_size_ + 1);
        predicted_sigma_points_ = Eigen::MatrixXd::Zero(state_size_, 2 * state_size_ + 1);
        measurement_sigma_points_ = Eigen::MatrixXd::Zero(measurement_size_, 2 * state_size_ + 1);
    }

    void predict(double dt) override
    {
        // Generate sigma points
        generateSigmaPoints();
        
        // Propagate sigma points through process model
        for (int i = 0; i < 2 * state_size_ + 1; ++i) {
            predicted_sigma_points_.col(i) = processModel(sigma_points_.col(i), dt);
        }
        
        // Calculate predicted mean and covariance
        calculatePredictedMeanAndCovariance(dt);
    }

    void update(const Eigen::VectorXd& measurement, const Eigen::MatrixXd& measurement_matrix) override
    {
        // Generate measurement sigma points
        for (int i = 0; i < 2 * state_size_ + 1; ++i) {
            measurement_sigma_points_.col(i) = measurement_matrix * predicted_sigma_points_.col(i);
        }
        
        // Calculate measurement mean and covariance
        Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(measurement_size_);
        for (int i = 0; i < 2 * state_size_ + 1; ++i) {
            z_pred += weights_m_(i) * measurement_sigma_points_.col(i);
        }
        
        Eigen::MatrixXd S = measurement_noise_;
        Eigen::MatrixXd T = Eigen::MatrixXd::Zero(state_size_, measurement_size_);
        
        for (int i = 0; i < 2 * state_size_ + 1; ++i) {
            Eigen::VectorXd z_diff = measurement_sigma_points_.col(i) - z_pred;
            Eigen::VectorXd x_diff = predicted_sigma_points_.col(i) - state_;
            
            S += weights_c_(i) * z_diff * z_diff.transpose();
            T += weights_c_(i) * x_diff * z_diff.transpose();
        }
        
        // Calculate Kalman gain
        Eigen::MatrixXd K = T * S.inverse();
        
        // Update state and covariance
        Eigen::VectorXd innovation = measurement - z_pred;
        state_ = state_ + K * innovation;
        covariance_ = covariance_ - K * S * K.transpose();
        
        // Normalize yaw angle
        normalizeYaw();
    }

private:
    void calculateWeights()
    {
        weights_m_ = Eigen::VectorXd::Zero(2 * state_size_ + 1);
        weights_c_ = Eigen::VectorXd::Zero(2 * state_size_ + 1);
        
        // First weight
        weights_m_(0) = lambda_ / (state_size_ + lambda_);
        weights_c_(0) = lambda_ / (state_size_ + lambda_) + (1 - alpha_ * alpha_ + beta_);
        
        // Remaining weights
        for (int i = 1; i < 2 * state_size_ + 1; ++i) {
            weights_m_(i) = 1.0 / (2.0 * (state_size_ + lambda_));
            weights_c_(i) = weights_m_(i);
        }
    }
    
    void generateSigmaPoints()
    {
        // Calculate square root of (n + lambda) * P
        Eigen::MatrixXd A = ((state_size_ + lambda_) * covariance_).llt().matrixL();
        
        // First sigma point (mean)
        sigma_points_.col(0) = state_;
        
        // Positive sigma points
        for (int i = 0; i < state_size_; ++i) {
            sigma_points_.col(i + 1) = state_ + A.col(i);
        }
        
        // Negative sigma points
        for (int i = 0; i < state_size_; ++i) {
            sigma_points_.col(i + 1 + state_size_) = state_ - A.col(i);
        }
    }
    
    Eigen::VectorXd processModel(const Eigen::VectorXd& state, double dt)
    {
        Eigen::VectorXd next_state = state;
        
        // Extract state variables
        double x = state(STATE_X);
        double y = state(STATE_Y);
        double yaw = state(STATE_YAW);
        double vx = state(STATE_VX);
        double vy = state(STATE_VY);
        double vyaw = state(STATE_VYAW);
        double ax = state(STATE_AX);
        double ay = state(STATE_AY);
        
        // Nonlinear process model (constant acceleration)
        next_state(STATE_X) = x + vx * dt + 0.5 * ax * dt * dt;
        next_state(STATE_Y) = y + vy * dt + 0.5 * ay * dt * dt;
        next_state(STATE_YAW) = yaw + vyaw * dt;
        next_state(STATE_VX) = vx + ax * dt;
        next_state(STATE_VY) = vy + ay * dt;
        next_state(STATE_VYAW) = vyaw; // Assume constant angular velocity
        next_state(STATE_AX) = ax; // Assume constant acceleration
        next_state(STATE_AY) = ay;
        
        return next_state;
    }
    
    void calculatePredictedMeanAndCovariance(double dt)
    {
        // Calculate predicted mean
        state_ = Eigen::VectorXd::Zero(state_size_);
        for (int i = 0; i < 2 * state_size_ + 1; ++i) {
            state_ += weights_m_(i) * predicted_sigma_points_.col(i);
        }
        
        // Calculate predicted covariance
        covariance_ = Eigen::MatrixXd::Zero(state_size_, state_size_);
        for (int i = 0; i < 2 * state_size_ + 1; ++i) {
            Eigen::VectorXd diff = predicted_sigma_points_.col(i) - state_;
            covariance_ += weights_c_(i) * diff * diff.transpose();
        }
        
        // Add process noise
        updateProcessNoise(dt);
        covariance_ += process_noise_;
    }
    
    void updateProcessNoise(double dt)
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
        
        process_noise_ = Eigen::MatrixXd::Zero(state_size_, state_size_);
        
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
    }
    
    void normalizeYaw()
    {
        while (state_(STATE_YAW) > M_PI) state_(STATE_YAW) -= 2.0 * M_PI;
        while (state_(STATE_YAW) < -M_PI) state_(STATE_YAW) += 2.0 * M_PI;
    }

    // UKF parameters
    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;
    
    // Weights
    Eigen::VectorXd weights_m_;
    Eigen::VectorXd weights_c_;
    
    // Sigma points
    Eigen::MatrixXd sigma_points_;
    Eigen::MatrixXd predicted_sigma_points_;
    Eigen::MatrixXd measurement_sigma_points_;
};

class UKFLocalizationNode : public rclcpp::Node
{
public:
    UKFLocalizationNode() : Node("ukf_localization_node"), tf_broadcaster_(this)
    {
        loadParameters();
        initializeFilter();
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&UKFLocalizationNode::odomCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10,
            std::bind(&UKFLocalizationNode::imuCallback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10,
            std::bind(&UKFLocalizationNode::initialPoseCallback, this, std::placeholders::_1));
        
        // Publishers
        filtered_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odometry/ukf_filtered", 10);
        
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "ukf_pose", 10);
        
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "localization/ukf_health", 10);
        
        // Timer for filter processing
        filter_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / filter_frequency_)),
            std::bind(&UKFLocalizationNode::filterLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "UKF Localization Node initialized");
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
        ukf_ = std::make_unique<UnscentedKalmanFilter>();
        
        // Set initial state
        ukf_->setState(initial_state_);
        
        // Set initial covariance
        Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
        initial_covariance.diagonal() << 1.0, 1.0, 0.1, 0.5, 0.5, 0.1, 1.0, 1.0;
        ukf_->setCovariance(initial_covariance);
        
        // Set measurement noise
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6);
        R.diagonal() << 0.1, 0.1, 0.05, 0.1, 0.1, 0.05; // x, y, yaw, vx, vy, vyaw
        ukf_->setMeasurementNoise(R);
        
        last_prediction_time_ = this->now();
        filter_initialized_ = true;
        
        RCLCPP_INFO(this->get_logger(), "UKF filter initialized");
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
        odom_measurement.covariance = H;
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
        imu_measurement.covariance = H;
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
        
        ukf_->setState(new_state);
        
        RCLCPP_INFO(this->get_logger(), "UKF reset with initial pose: (%.2f, %.2f, %.2f)",
                    new_state(STATE_X), new_state(STATE_Y), new_state(STATE_YAW));
    }
    
    void filterLoop()
    {
        if (!filter_initialized_) return;
        
        auto current_time = this->now();
        double dt = (current_time - last_prediction_time_).seconds();
        
        if (dt > 0.0 && dt < 1.0) {
            // Prediction step
            ukf_->predict(dt);
            
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
        for (const auto& measurement : measurement_queue_) {
            ukf_->update(measurement.data, measurement.covariance);
        }
        measurement_queue_.clear();
    }
    
    void publishResults(const rclcpp::Time& timestamp)
    {
        const Eigen::VectorXd& state = ukf_->getState();
        const Eigen::MatrixXd& covariance = ukf_->getCovariance();
        
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
        
        // Set system component statuses
        health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set motor statuses
        health_msg.front_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.front_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set temperatures
        health_msg.cpu_temperature = 45.0; // Placeholder
        health_msg.gpu_temperature = 42.0; // Placeholder
        health_msg.motor_temperature = 40.0; // Placeholder
        
        // Set diagnostic info
        health_msg.diagnostic_info = "UKF localization running";
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
    std::unique_ptr<UnscentedKalmanFilter> ukf_;
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
    auto node = std::make_shared<tadeo_ecar_localization::UKFLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}