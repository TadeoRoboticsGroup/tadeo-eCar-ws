#ifndef TADEO_ECAR_CONTROL_WHEEL_CONTROLLER_HPP_
#define TADEO_ECAR_CONTROL_WHEEL_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tadeo_ecar_msgs/msg/wheel_states.hpp>
#include <tadeo_ecar_interfaces/srv/calibrate_wheels.hpp>

namespace tadeo_ecar_control
{

struct PIDController
{
    double kp, ki, kd;
    double prev_error;
    double integral;
    double max_output;
    double min_output;
    
    PIDController() : kp(0.0), ki(0.0), kd(0.0), prev_error(0.0), integral(0.0),
                      max_output(100.0), min_output(-100.0) {}
    
    double compute(double setpoint, double current_value, double dt);
    void reset();
};

struct WheelConfig
{
    std::string name;
    double max_velocity;
    double max_steering_angle;
    double steering_rate_limit;
    PIDController velocity_pid;
    PIDController steering_pid;
};

class WheelController : public rclcpp::Node
{
public:
    WheelController();
    
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void controlLoop();
    void publishWheelCommands();
    void publishWheelStates();
    void calibrateWheelsService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::CalibrateWheels::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::CalibrateWheels::Response> response);
    
    void loadParameters();
    void setupWheelConfigs();
    void calculateWheelVelocities(double linear_x, double linear_y, double angular_z);
    void calculateWheelSteering(double linear_x, double linear_y, double angular_z);
    
    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fl_wheel_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fr_wheel_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rl_wheel_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rr_wheel_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fl_steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fr_steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rl_steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rr_steering_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::WheelStates>::SharedPtr wheel_states_pub_;
    rclcpp::Service<tadeo_ecar_interfaces::srv::CalibrateWheels>::SharedPtr calibrate_service_;
    rclcpp::TimerInterface::SharedPtr control_timer_;
    
    // Robot parameters
    double wheelbase_;
    double track_width_;
    double wheel_radius_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double control_frequency_;
    
    // Wheel configurations
    std::map<std::string, WheelConfig> wheel_configs_;
    
    // Current states
    std::map<std::string, double> current_wheel_velocities_;
    std::map<std::string, double> current_wheel_angles_;
    std::map<std::string, double> target_wheel_velocities_;
    std::map<std::string, double> target_wheel_angles_;
    
    // Command velocities
    double target_linear_x_;
    double target_linear_y_;
    double target_angular_z_;
    
    // Safety and status
    bool is_calibrated_;
    bool emergency_stop_;
    rclcpp::Time last_cmd_time_;
    double cmd_timeout_;
};

} // namespace tadeo_ecar_control

#endif // TADEO_ECAR_CONTROL_WHEEL_CONTROLLER_HPP_