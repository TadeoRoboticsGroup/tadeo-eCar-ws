#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <tadeo_ecar_msgs/msg/system_health.hpp>

namespace tadeo_ecar_localization
{

class SensorFusionLocalizationNode : public rclcpp::Node
{
public:
    SensorFusionLocalizationNode() : Node("sensor_fusion_localization_node"), tf_broadcaster_(this)
    {
        loadParameters();
        initializeFusion();
        
        // Subscribers
        ekf_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10,
            std::bind(&SensorFusionLocalizationNode::ekfOdomCallback, this, std::placeholders::_1));
        
        ukf_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/ukf_filtered", 10,
            std::bind(&SensorFusionLocalizationNode::ukfOdomCallback, this, std::placeholders::_1));
        
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/fix", 10,
            std::bind(&SensorFusionLocalizationNode::gpsCallback, this, std::placeholders::_1));
        
        // Publishers
        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/fused", 10);
        fused_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose/fused", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("localization/fusion_health", 10);
        
        // Timer
        fusion_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / fusion_frequency_)),
            std::bind(&SensorFusionLocalizationNode::fusionLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Localization Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("fusion_frequency", 20.0);
        this->declare_parameter("ekf_weight", 0.6);
        this->declare_parameter("ukf_weight", 0.4);
        this->declare_parameter("gps_weight", 0.3);
        this->declare_parameter("use_gps", false);
        
        fusion_frequency_ = this->get_parameter("fusion_frequency").as_double();
        ekf_weight_ = this->get_parameter("ekf_weight").as_double();
        ukf_weight_ = this->get_parameter("ukf_weight").as_double();
        gps_weight_ = this->get_parameter("gps_weight").as_double();
        use_gps_ = this->get_parameter("use_gps").as_bool();
    }
    
    void initializeFusion()
    {
        fused_pose_ = Eigen::VectorXd::Zero(6); // x, y, yaw, vx, vy, vyaw
        fused_covariance_ = Eigen::MatrixXd::Identity(6, 6);
        
        ekf_available_ = false;
        ukf_available_ = false;
        gps_available_ = false;
        
        last_fusion_time_ = this->now();
    }
    
    void ekfOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ekf_odom_ = *msg;
        ekf_available_ = true;
        ekf_last_time_ = msg->header.stamp;
    }
    
    void ukfOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ukf_odom_ = *msg;
        ukf_available_ = true;
        ukf_last_time_ = msg->header.stamp;
    }
    
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (!use_gps_) return;
        
        gps_fix_ = *msg;
        gps_available_ = true;
        gps_last_time_ = msg->header.stamp;
    }
    
    void fusionLoop()
    {
        auto current_time = this->now();
        
        // Check data availability and freshness
        checkDataFreshness(current_time);
        
        if (ekf_available_ || ukf_available_) {
            performFusion();
            publishResults(current_time);
        }
        
        publishHealthStatus();
        last_fusion_time_ = current_time;
    }
    
    void checkDataFreshness(const rclcpp::Time& current_time)
    {
        double timeout = 1.0; // 1 second timeout
        
        if (ekf_available_ && (current_time - ekf_last_time_).seconds() > timeout) {
            ekf_available_ = false;
        }
        
        if (ukf_available_ && (current_time - ukf_last_time_).seconds() > timeout) {
            ukf_available_ = false;
        }
        
        if (gps_available_ && (current_time - gps_last_time_).seconds() > timeout) {
            gps_available_ = false;
        }
    }
    
    void performFusion()
    {
        double total_weight = 0.0;
        fused_pose_ = Eigen::VectorXd::Zero(6);
        fused_covariance_ = Eigen::MatrixXd::Zero(6, 6);
        
        // Weighted fusion of EKF and UKF estimates
        if (ekf_available_) {
            Eigen::VectorXd ekf_state(6);
            ekf_state << ekf_odom_.pose.pose.position.x,
                         ekf_odom_.pose.pose.position.y,
                         extractYaw(ekf_odom_.pose.pose.orientation),
                         ekf_odom_.twist.twist.linear.x,
                         ekf_odom_.twist.twist.linear.y,
                         ekf_odom_.twist.twist.angular.z;
            
            fused_pose_ += ekf_weight_ * ekf_state;
            total_weight += ekf_weight_;
        }
        
        if (ukf_available_) {
            Eigen::VectorXd ukf_state(6);
            ukf_state << ukf_odom_.pose.pose.position.x,
                         ukf_odom_.pose.pose.position.y,
                         extractYaw(ukf_odom_.pose.pose.orientation),
                         ukf_odom_.twist.twist.linear.x,
                         ukf_odom_.twist.twist.linear.y,
                         ukf_odom_.twist.twist.angular.z;
            
            fused_pose_ += ukf_weight_ * ukf_state;
            total_weight += ukf_weight_;
        }
        
        // Normalize by total weight
        if (total_weight > 0.0) {
            fused_pose_ /= total_weight;
        }
        
        // Simple covariance fusion (conservative approach)
        if (ekf_available_ && ukf_available_) {
            // Use minimum covariance approach
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    if (i < 3 && j < 3) {
                        double ekf_cov = ekf_odom_.pose.covariance[i * 6 + j];
                        double ukf_cov = ukf_odom_.pose.covariance[i * 6 + j];
                        fused_covariance_(i, j) = std::min(ekf_cov, ukf_cov);
                    } else if (i >= 3 && j >= 3) {
                        double ekf_cov = ekf_odom_.twist.covariance[(i-3) * 6 + (j-3)];
                        double ukf_cov = ukf_odom_.twist.covariance[(i-3) * 6 + (j-3)];
                        fused_covariance_(i, j) = std::min(ekf_cov, ukf_cov);
                    }
                }
            }
        } else if (ekf_available_) {
            // Use EKF covariance
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    if (i < 3 && j < 3) {
                        fused_covariance_(i, j) = ekf_odom_.pose.covariance[i * 6 + j];
                    } else if (i >= 3 && j >= 3) {
                        fused_covariance_(i, j) = ekf_odom_.twist.covariance[(i-3) * 6 + (j-3)];
                    }
                }
            }
        } else if (ukf_available_) {
            // Use UKF covariance
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    if (i < 3 && j < 3) {
                        fused_covariance_(i, j) = ukf_odom_.pose.covariance[i * 6 + j];
                    } else if (i >= 3 && j >= 3) {
                        fused_covariance_(i, j) = ukf_odom_.twist.covariance[(i-3) * 6 + (j-3)];
                    }
                }
            }
        }
    }
    
    double extractYaw(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion q;
        tf2::fromMsg(quat, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }
    
    void publishResults(const rclcpp::Time& timestamp)
    {
        // Publish fused odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        
        odom_msg.pose.pose.position.x = fused_pose_(0);
        odom_msg.pose.pose.position.y = fused_pose_(1);
        odom_msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, fused_pose_(2));
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        odom_msg.twist.twist.linear.x = fused_pose_(3);
        odom_msg.twist.twist.linear.y = fused_pose_(4);
        odom_msg.twist.twist.angular.z = fused_pose_(5);
        
        // Set covariance
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                if (i < 3 && j < 3) {
                    odom_msg.pose.covariance[i * 6 + j] = fused_covariance_(i, j);
                } else if (i >= 3 && j >= 3) {
                    odom_msg.twist.covariance[i * 6 + j] = fused_covariance_(i, j);
                }
            }
        }
        
        fused_odom_pub_->publish(odom_msg);
        
        // Publish fused pose
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header = odom_msg.header;
        pose_msg.pose = odom_msg.pose;
        fused_pose_pub_->publish(pose_msg);
        
        // Publish transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header = odom_msg.header;
        transform.child_frame_id = "odom";
        transform.transform.translation.x = fused_pose_(0);
        transform.transform.translation.y = fused_pose_(1);
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = odom_msg.pose.pose.orientation;
        
        tf_broadcaster_.sendTransform(transform);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = "base_link";
        
        // Set system component statuses based on sensor fusion health
        if (!ekf_available_ && !ukf_available_) {
            health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
            health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.error_codes.push_back(6003);
            health_msg.error_messages.push_back("No localization sources available");
        } else if (!ekf_available_ || !ukf_available_) {
            health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
            health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            health_msg.error_codes.push_back(6004);
            health_msg.error_messages.push_back("Only one localization source available");
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
        health_msg.cpu_temperature = 42.0; // Placeholder
        health_msg.gpu_temperature = 40.0; // Placeholder
        health_msg.motor_temperature = 38.0; // Placeholder
        
        // Set diagnostic info
        health_msg.diagnostic_info = "Sensor fusion localization running";
        health_msg.uptime_seconds = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ukf_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fused_pose_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::TimerBase::SharedPtr fusion_timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Parameters
    double fusion_frequency_;
    double ekf_weight_;
    double ukf_weight_;
    double gps_weight_;
    bool use_gps_;
    
    // State
    Eigen::VectorXd fused_pose_;
    Eigen::MatrixXd fused_covariance_;
    
    // Data availability
    bool ekf_available_;
    bool ukf_available_;
    bool gps_available_;
    
    // Data storage
    nav_msgs::msg::Odometry ekf_odom_;
    nav_msgs::msg::Odometry ukf_odom_;
    sensor_msgs::msg::NavSatFix gps_fix_;
    
    // Timestamps
    rclcpp::Time ekf_last_time_;
    rclcpp::Time ukf_last_time_;
    rclcpp::Time gps_last_time_;
    rclcpp::Time last_fusion_time_;
};

} // namespace tadeo_ecar_localization

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_localization::SensorFusionLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}