#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <cmath>
#include <deque>
#include <numeric>

namespace tadeo_ecar_perception
{

class ImuProcessorNode : public rclcpp::Node
{
public:
    ImuProcessorNode() : Node("imu_processor_node")
    {
        loadParameters();
        initializeFilters();
        
        // Subscriber
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw", 10,
            std::bind(&ImuProcessorNode::imuCallback, this, std::placeholders::_1));
        
        // Publishers
        filtered_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "imu/data", 10);
        
        euler_angles_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "imu/euler_angles", 10);
        
        linear_acceleration_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "imu/linear_acceleration", 10);
        
        angular_velocity_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "imu/angular_velocity", 10);
        
        roll_pub_ = this->create_publisher<std_msgs::msg::Float64>("imu/roll", 10);
        pitch_pub_ = this->create_publisher<std_msgs::msg::Float64>("imu/pitch", 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("imu/yaw", 10);
        
        lateral_acceleration_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "imu/lateral_acceleration", 10);
        
        longitudinal_acceleration_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "imu/longitudinal_acceleration", 10);
        
        tilt_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "imu/tilt_angle", 10);
        
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "perception/imu_health", 10);
        
        // Processing timer
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / processing_frequency_)),
            std::bind(&ImuProcessorNode::processingLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "IMU Processor Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("processing.frequency", 50.0);
        this->declare_parameter("filtering.enable_calibration", true);
        this->declare_parameter("filtering.calibration_samples", 1000);
        this->declare_parameter("filtering.enable_lowpass", true);
        this->declare_parameter("filtering.lowpass_alpha", 0.8);
        this->declare_parameter("filtering.enable_highpass", true);
        this->declare_parameter("filtering.highpass_alpha", 0.95);
        this->declare_parameter("thresholds.max_acceleration", 20.0);
        this->declare_parameter("thresholds.max_angular_velocity", 10.0);
        this->declare_parameter("thresholds.stability_threshold", 0.2);
        this->declare_parameter("gravity.magnitude", 9.81);
        
        processing_frequency_ = this->get_parameter("processing.frequency").as_double();
        enable_calibration_ = this->get_parameter("filtering.enable_calibration").as_bool();
        calibration_samples_ = this->get_parameter("filtering.calibration_samples").as_int();
        enable_lowpass_ = this->get_parameter("filtering.enable_lowpass").as_bool();
        lowpass_alpha_ = this->get_parameter("filtering.lowpass_alpha").as_double();
        enable_highpass_ = this->get_parameter("filtering.enable_highpass").as_bool();
        highpass_alpha_ = this->get_parameter("filtering.highpass_alpha").as_double();
        max_acceleration_ = this->get_parameter("thresholds.max_acceleration").as_double();
        max_angular_velocity_ = this->get_parameter("thresholds.max_angular_velocity").as_double();
        stability_threshold_ = this->get_parameter("thresholds.stability_threshold").as_double();
        gravity_magnitude_ = this->get_parameter("gravity.magnitude").as_double();
    }
    
    void initializeFilters()
    {
        is_calibrated_ = false;
        calibration_count_ = 0;
        
        // Initialize bias values
        accel_bias_ = {0.0, 0.0, 0.0};
        gyro_bias_ = {0.0, 0.0, 0.0};
        
        // Initialize filter states
        accel_filtered_ = {0.0, 0.0, 0.0};
        gyro_filtered_ = {0.0, 0.0, 0.0};
        accel_highpass_ = {0.0, 0.0, 0.0};
        
        // Initialize orientation
        current_orientation_.setRPY(0.0, 0.0, 0.0);
        
        last_imu_time_ = this->now();
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        current_imu_ = *msg;
        last_imu_time_ = this->now();
        
        // Process IMU data
        processImuData();
    }
    
    void processImuData()
    {
        // Calibration phase
        if (enable_calibration_ && !is_calibrated_) {
            calibrateImu();
            return;
        }
        
        // Apply bias correction
        auto corrected_imu = current_imu_;
        
        corrected_imu.linear_acceleration.x -= accel_bias_[0];
        corrected_imu.linear_acceleration.y -= accel_bias_[1];
        corrected_imu.linear_acceleration.z -= accel_bias_[2];
        
        corrected_imu.angular_velocity.x -= gyro_bias_[0];
        corrected_imu.angular_velocity.y -= gyro_bias_[1];
        corrected_imu.angular_velocity.z -= gyro_bias_[2];
        
        // Apply filters
        if (enable_lowpass_) {
            applyLowPassFilter(corrected_imu);
        }
        
        if (enable_highpass_) {
            applyHighPassFilter(corrected_imu);
        }
        
        // Validate data
        if (!validateImuData(corrected_imu)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "IMU data validation failed - values out of range");
            return;
        }
        
        // Update orientation estimate
        updateOrientation(corrected_imu);
        
        // Calculate derived values
        calculateDerivedValues(corrected_imu);
        
        // Publish processed data
        publishProcessedData(corrected_imu);
    }
    
    void calibrateImu()
    {
        if (calibration_count_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Starting IMU calibration - keep robot stationary");
            accel_bias_ = {0.0, 0.0, 0.0};
            gyro_bias_ = {0.0, 0.0, 0.0};
        }
        
        // Accumulate samples
        accel_bias_[0] += current_imu_.linear_acceleration.x;
        accel_bias_[1] += current_imu_.linear_acceleration.y;
        accel_bias_[2] += current_imu_.linear_acceleration.z - gravity_magnitude_; // Remove gravity
        
        gyro_bias_[0] += current_imu_.angular_velocity.x;
        gyro_bias_[1] += current_imu_.angular_velocity.y;
        gyro_bias_[2] += current_imu_.angular_velocity.z;
        
        calibration_count_++;
        
        if (calibration_count_ >= calibration_samples_) {
            // Calculate average bias
            accel_bias_[0] /= calibration_samples_;
            accel_bias_[1] /= calibration_samples_;
            accel_bias_[2] /= calibration_samples_;
            
            gyro_bias_[0] /= calibration_samples_;
            gyro_bias_[1] /= calibration_samples_;
            gyro_bias_[2] /= calibration_samples_;
            
            is_calibrated_ = true;
            RCLCPP_INFO(this->get_logger(), "IMU calibration completed");
            RCLCPP_INFO(this->get_logger(), "Accel bias: [%.4f, %.4f, %.4f]", 
                        accel_bias_[0], accel_bias_[1], accel_bias_[2]);
            RCLCPP_INFO(this->get_logger(), "Gyro bias: [%.4f, %.4f, %.4f]", 
                        gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
        }
    }
    
    void applyLowPassFilter(sensor_msgs::msg::Imu& imu)
    {
        // Low-pass filter for noise reduction
        accel_filtered_[0] = lowpass_alpha_ * accel_filtered_[0] + 
                             (1.0 - lowpass_alpha_) * imu.linear_acceleration.x;
        accel_filtered_[1] = lowpass_alpha_ * accel_filtered_[1] + 
                             (1.0 - lowpass_alpha_) * imu.linear_acceleration.y;
        accel_filtered_[2] = lowpass_alpha_ * accel_filtered_[2] + 
                             (1.0 - lowpass_alpha_) * imu.linear_acceleration.z;
        
        gyro_filtered_[0] = lowpass_alpha_ * gyro_filtered_[0] + 
                            (1.0 - lowpass_alpha_) * imu.angular_velocity.x;
        gyro_filtered_[1] = lowpass_alpha_ * gyro_filtered_[1] + 
                            (1.0 - lowpass_alpha_) * imu.angular_velocity.y;
        gyro_filtered_[2] = lowpass_alpha_ * gyro_filtered_[2] + 
                            (1.0 - lowpass_alpha_) * imu.angular_velocity.z;
        
        imu.linear_acceleration.x = accel_filtered_[0];
        imu.linear_acceleration.y = accel_filtered_[1];
        imu.linear_acceleration.z = accel_filtered_[2];
        
        imu.angular_velocity.x = gyro_filtered_[0];
        imu.angular_velocity.y = gyro_filtered_[1];
        imu.angular_velocity.z = gyro_filtered_[2];
    }
    
    void applyHighPassFilter(sensor_msgs::msg::Imu& imu)
    {
        // High-pass filter to remove gravity and low-frequency drift
        double new_accel[3] = {
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z
        };
        
        imu.linear_acceleration.x = highpass_alpha_ * (accel_highpass_[0] + new_accel[0] - accel_prev_[0]);
        imu.linear_acceleration.y = highpass_alpha_ * (accel_highpass_[1] + new_accel[1] - accel_prev_[1]);
        imu.linear_acceleration.z = highpass_alpha_ * (accel_highpass_[2] + new_accel[2] - accel_prev_[2]);
        
        accel_highpass_[0] = imu.linear_acceleration.x;
        accel_highpass_[1] = imu.linear_acceleration.y;
        accel_highpass_[2] = imu.linear_acceleration.z;
        
        accel_prev_[0] = new_accel[0];
        accel_prev_[1] = new_accel[1];
        accel_prev_[2] = new_accel[2];
    }
    
    bool validateImuData(const sensor_msgs::msg::Imu& imu)
    {
        // Check for reasonable acceleration values
        double accel_magnitude = sqrt(
            imu.linear_acceleration.x * imu.linear_acceleration.x +
            imu.linear_acceleration.y * imu.linear_acceleration.y +
            imu.linear_acceleration.z * imu.linear_acceleration.z
        );
        
        if (accel_magnitude > max_acceleration_) {
            return false;
        }
        
        // Check for reasonable angular velocity values
        double gyro_magnitude = sqrt(
            imu.angular_velocity.x * imu.angular_velocity.x +
            imu.angular_velocity.y * imu.angular_velocity.y +
            imu.angular_velocity.z * imu.angular_velocity.z
        );
        
        if (gyro_magnitude > max_angular_velocity_) {
            return false;
        }
        
        return true;
    }
    
    void updateOrientation(const sensor_msgs::msg::Imu& imu)
    {
        // Simple integration of angular velocity for orientation estimate
        auto current_time = this->now();
        double dt = (current_time - last_orientation_update_).seconds();
        
        if (dt > 0.0 && dt < 0.1) { // Reasonable time step
            tf2::Quaternion delta_rotation;
            delta_rotation.setRPY(
                imu.angular_velocity.x * dt,
                imu.angular_velocity.y * dt,
                imu.angular_velocity.z * dt
            );
            
            current_orientation_ = current_orientation_ * delta_rotation;
            current_orientation_.normalize();
        }
        
        last_orientation_update_ = current_time;
    }
    
    void calculateDerivedValues(const sensor_msgs::msg::Imu& imu)
    {
        // Extract Euler angles
        tf2::Matrix3x3 rotation_matrix(current_orientation_);
        rotation_matrix.getRPY(current_roll_, current_pitch_, current_yaw_);
        
        // Calculate lateral and longitudinal acceleration in vehicle frame
        // Assuming vehicle frame: x-forward, y-left, z-up
        lateral_acceleration_ = imu.linear_acceleration.y;
        longitudinal_acceleration_ = imu.linear_acceleration.x;
        
        // Calculate tilt angle (magnitude of roll and pitch)
        tilt_angle_ = sqrt(current_roll_ * current_roll_ + current_pitch_ * current_pitch_);
        
        // Update stability metrics
        updateStabilityMetrics(imu);
    }
    
    void updateStabilityMetrics(const sensor_msgs::msg::Imu& imu)
    {
        // Add current values to rolling window
        accel_window_.push_back(lateral_acceleration_);
        gyro_window_.push_back(imu.angular_velocity.z);
        
        // Maintain window size
        const size_t window_size = 50; // ~1 second at 50Hz
        if (accel_window_.size() > window_size) {
            accel_window_.pop_front();
        }
        if (gyro_window_.size() > window_size) {
            gyro_window_.pop_front();
        }
        
        // Calculate stability metrics
        if (accel_window_.size() >= 10) {
            double accel_variance = calculateVariance(accel_window_);
            double gyro_variance = calculateVariance(gyro_window_);
            
            stability_factor_ = 1.0 / (1.0 + accel_variance + gyro_variance);
            is_stable_ = (accel_variance < stability_threshold_) && 
                         (gyro_variance < stability_threshold_);
        }
    }
    
    double calculateVariance(const std::deque<double>& data)
    {
        if (data.size() < 2) return 0.0;
        
        double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();
        double variance = 0.0;
        
        for (double value : data) {
            variance += (value - mean) * (value - mean);
        }
        
        return variance / (data.size() - 1);
    }
    
    void publishProcessedData(const sensor_msgs::msg::Imu& imu)
    {
        // Publish filtered IMU data
        auto filtered_imu = imu;
        filtered_imu.orientation = tf2::toMsg(current_orientation_);
        filtered_imu_pub_->publish(filtered_imu);
        
        // Publish Euler angles
        auto euler_msg = geometry_msgs::msg::Vector3Stamped();
        euler_msg.header = imu.header;
        euler_msg.vector.x = current_roll_;
        euler_msg.vector.y = current_pitch_;
        euler_msg.vector.z = current_yaw_;
        euler_angles_pub_->publish(euler_msg);
        
        // Publish individual angles
        auto roll_msg = std_msgs::msg::Float64();
        roll_msg.data = current_roll_;
        roll_pub_->publish(roll_msg);
        
        auto pitch_msg = std_msgs::msg::Float64();
        pitch_msg.data = current_pitch_;
        pitch_pub_->publish(pitch_msg);
        
        auto yaw_msg = std_msgs::msg::Float64();
        yaw_msg.data = current_yaw_;
        yaw_pub_->publish(yaw_msg);
        
        // Publish derived accelerations
        auto lat_accel_msg = std_msgs::msg::Float64();
        lat_accel_msg.data = lateral_acceleration_;
        lateral_acceleration_pub_->publish(lat_accel_msg);
        
        auto lon_accel_msg = std_msgs::msg::Float64();
        lon_accel_msg.data = longitudinal_acceleration_;
        longitudinal_acceleration_pub_->publish(lon_accel_msg);
        
        // Publish tilt angle
        auto tilt_msg = std_msgs::msg::Float64();
        tilt_msg.data = tilt_angle_;
        tilt_angle_pub_->publish(tilt_msg);
        
        // Publish linear acceleration vector
        auto lin_accel_msg = geometry_msgs::msg::Vector3Stamped();
        lin_accel_msg.header = imu.header;
        lin_accel_msg.vector = imu.linear_acceleration;
        linear_acceleration_pub_->publish(lin_accel_msg);
        
        // Publish angular velocity vector
        auto ang_vel_msg = geometry_msgs::msg::Vector3Stamped();
        ang_vel_msg.header = imu.header;
        ang_vel_msg.vector = imu.angular_velocity;
        angular_velocity_pub_->publish(ang_vel_msg);
    }
    
    void processingLoop()
    {
        publishHealthStatus();
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = "imu_link";
        
        health_msg.component_name = "imu_processor";
        
        auto current_time = this->now();
        bool data_timeout = (current_time - last_imu_time_).seconds() > 1.0;
        
        if (data_timeout) {
            health_msg.status = "ERROR";
            health_msg.error_code = 3001;
            health_msg.error_message = "No IMU data received";
        } else if (!is_calibrated_) {
            health_msg.status = "CALIBRATING";
            health_msg.error_code = 3002;
            health_msg.error_message = "IMU calibration in progress";
        } else if (!is_stable_) {
            health_msg.status = "WARNING";
            health_msg.error_code = 3003;
            health_msg.error_message = "High vibration detected";
        } else {
            health_msg.status = "OK";
            health_msg.error_code = 0;
            health_msg.error_message = "";
        }
        
        health_msg.cpu_usage = 15.0; // Placeholder
        health_msg.memory_usage = 10.0; // Placeholder
        health_msg.temperature = 40.0; // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_angles_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr linear_acceleration_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angular_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lateral_acceleration_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr longitudinal_acceleration_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_angle_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    rclcpp::TimerInterface::SharedPtr processing_timer_;
    
    // Parameters
    double processing_frequency_;
    bool enable_calibration_;
    int calibration_samples_;
    bool enable_lowpass_;
    double lowpass_alpha_;
    bool enable_highpass_;
    double highpass_alpha_;
    double max_acceleration_;
    double max_angular_velocity_;
    double stability_threshold_;
    double gravity_magnitude_;
    
    // State variables
    sensor_msgs::msg::Imu current_imu_;
    bool is_calibrated_;
    int calibration_count_;
    std::array<double, 3> accel_bias_;
    std::array<double, 3> gyro_bias_;
    std::array<double, 3> accel_filtered_;
    std::array<double, 3> gyro_filtered_;
    std::array<double, 3> accel_highpass_;
    std::array<double, 3> accel_prev_;
    
    tf2::Quaternion current_orientation_;
    double current_roll_;
    double current_pitch_;
    double current_yaw_;
    double lateral_acceleration_;
    double longitudinal_acceleration_;
    double tilt_angle_;
    
    std::deque<double> accel_window_;
    std::deque<double> gyro_window_;
    double stability_factor_;
    bool is_stable_;
    
    rclcpp::Time last_imu_time_;
    rclcpp::Time last_orientation_update_;
};

} // namespace tadeo_ecar_perception

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_perception::ImuProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}