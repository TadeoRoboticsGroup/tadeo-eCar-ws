#ifndef TADEO_ECAR_LOCALIZATION_KALMAN_FILTER_BASE_HPP_
#define TADEO_ECAR_LOCALIZATION_KALMAN_FILTER_BASE_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace tadeo_ecar_localization
{

class KalmanFilterBase
{
public:
    KalmanFilterBase(int state_size, int measurement_size)
        : state_size_(state_size), measurement_size_(measurement_size)
    {
        state_ = Eigen::VectorXd::Zero(state_size);
        covariance_ = Eigen::MatrixXd::Identity(state_size, state_size);
        process_noise_ = Eigen::MatrixXd::Identity(state_size, state_size) * 0.01;
        measurement_noise_ = Eigen::MatrixXd::Identity(measurement_size, measurement_size) * 0.1;
    }

    virtual ~KalmanFilterBase() = default;

    // Pure virtual functions
    virtual void predict(double dt) = 0;
    virtual void update(const Eigen::VectorXd& measurement, const Eigen::MatrixXd& measurement_matrix) = 0;

    // Getters
    const Eigen::VectorXd& getState() const { return state_; }
    const Eigen::MatrixXd& getCovariance() const { return covariance_; }
    
    // Setters
    void setState(const Eigen::VectorXd& state) { state_ = state; }
    void setCovariance(const Eigen::MatrixXd& covariance) { covariance_ = covariance; }
    void setProcessNoise(const Eigen::MatrixXd& Q) { process_noise_ = Q; }
    void setMeasurementNoise(const Eigen::MatrixXd& R) { measurement_noise_ = R; }

protected:
    int state_size_;
    int measurement_size_;
    
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    Eigen::MatrixXd process_noise_;
    Eigen::MatrixXd measurement_noise_;
};

// State vector indices for vehicle state
enum StateIndex
{
    STATE_X = 0,      // Position X
    STATE_Y = 1,      // Position Y
    STATE_YAW = 2,    // Orientation (yaw)
    STATE_VX = 3,     // Linear velocity X
    STATE_VY = 4,     // Linear velocity Y
    STATE_VYAW = 5,   // Angular velocity (yaw rate)
    STATE_AX = 6,     // Linear acceleration X
    STATE_AY = 7,     // Linear acceleration Y
    STATE_SIZE = 8    // Total state size
};

// Measurement types
enum MeasurementType
{
    MEASUREMENT_ODOM = 0,
    MEASUREMENT_GPS = 1,
    MEASUREMENT_IMU = 2,
    MEASUREMENT_LIDAR = 3
};

struct SensorMeasurement
{
    Eigen::VectorXd data;
    Eigen::MatrixXd covariance;
    MeasurementType type;
    rclcpp::Time timestamp;
    std::string frame_id;
};

} // namespace tadeo_ecar_localization

#endif // TADEO_ECAR_LOCALIZATION_KALMAN_FILTER_BASE_HPP_