#include "tadeo_ecar_control/wheel_controller.hpp"
#include <cmath>
#include <algorithm>

namespace tadeo_ecar_control
{

double PIDController::compute(double setpoint, double current_value, double dt)
{
    if (dt <= 0.0) return 0.0;
    
    double error = setpoint - current_value;
    integral += error * dt;
    
    // Anti-windup
    integral = std::clamp(integral, min_output / ki, max_output / ki);
    
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    
    double output = kp * error + ki * integral + kd * derivative;
    return std::clamp(output, min_output, max_output);
}

void PIDController::reset()
{
    prev_error = 0.0;
    integral = 0.0;
}

WheelController::WheelController() : Node("wheel_controller")
{
    // Initialize parameters
    target_linear_x_ = 0.0;
    target_linear_y_ = 0.0;
    target_angular_z_ = 0.0;
    is_calibrated_ = false;
    emergency_stop_ = false;
    last_cmd_time_ = this->now();
    
    loadParameters();
    setupWheelConfigs();
    
    // Create subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&WheelController::cmdVelCallback, this, std::placeholders::_1));
    
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&WheelController::jointStateCallback, this, std::placeholders::_1));
    
    // Create publishers for wheel velocities
    fl_wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "fl_wheel_velocity_controller/command", 10);
    fr_wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "fr_wheel_velocity_controller/command", 10);
    rl_wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "rl_wheel_velocity_controller/command", 10);
    rr_wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "rr_wheel_velocity_controller/command", 10);
    
    // Create publishers for wheel steering
    fl_steering_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "fl_steering_controller/command", 10);
    fr_steering_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "fr_steering_controller/command", 10);
    rl_steering_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "rl_steering_controller/command", 10);
    rr_steering_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "rr_steering_controller/command", 10);
    
    // Create wheel states publisher
    wheel_states_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::WheelStates>(
        "wheel_states", 10);
    
    // Create calibration service
    calibrate_service_ = this->create_service<tadeo_ecar_interfaces::srv::CalibrateWheels>(
        "calibrate_wheels",
        std::bind(&WheelController::calibrateWheelsService, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // Create control timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
        std::bind(&WheelController::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Wheel Controller Node initialized");
}

void WheelController::loadParameters()
{
    this->declare_parameter("robot.wheelbase", 1.2);
    this->declare_parameter("robot.track_width", 1.0);
    this->declare_parameter("robot.wheel_radius", 0.15);
    this->declare_parameter("robot.max_linear_velocity", 2.0);
    this->declare_parameter("robot.max_angular_velocity", 1.5);
    this->declare_parameter("control.frequency", 50.0);
    this->declare_parameter("control.cmd_timeout", 0.5);
    
    wheelbase_ = this->get_parameter("robot.wheelbase").as_double();
    track_width_ = this->get_parameter("robot.track_width").as_double();
    wheel_radius_ = this->get_parameter("robot.wheel_radius").as_double();
    max_linear_velocity_ = this->get_parameter("robot.max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("robot.max_angular_velocity").as_double();
    control_frequency_ = this->get_parameter("control.frequency").as_double();
    cmd_timeout_ = this->get_parameter("control.cmd_timeout").as_double();
}

void WheelController::setupWheelConfigs()
{
    std::vector<std::string> wheel_names = {"fl", "fr", "rl", "rr"};
    
    for (const auto& name : wheel_names) {
        WheelConfig config;
        config.name = name;
        
        // Declare parameters for each wheel
        this->declare_parameter("wheels." + name + ".max_velocity", 10.0);
        this->declare_parameter("wheels." + name + ".max_steering_angle", 0.785); // 45 degrees
        this->declare_parameter("wheels." + name + ".steering_rate_limit", 2.0);
        this->declare_parameter("wheels." + name + ".velocity_pid.kp", 1.0);
        this->declare_parameter("wheels." + name + ".velocity_pid.ki", 0.1);
        this->declare_parameter("wheels." + name + ".velocity_pid.kd", 0.05);
        this->declare_parameter("wheels." + name + ".steering_pid.kp", 2.0);
        this->declare_parameter("wheels." + name + ".steering_pid.ki", 0.2);
        this->declare_parameter("wheels." + name + ".steering_pid.kd", 0.1);
        
        config.max_velocity = this->get_parameter("wheels." + name + ".max_velocity").as_double();
        config.max_steering_angle = this->get_parameter("wheels." + name + ".max_steering_angle").as_double();
        config.steering_rate_limit = this->get_parameter("wheels." + name + ".steering_rate_limit").as_double();
        
        config.velocity_pid.kp = this->get_parameter("wheels." + name + ".velocity_pid.kp").as_double();
        config.velocity_pid.ki = this->get_parameter("wheels." + name + ".velocity_pid.ki").as_double();
        config.velocity_pid.kd = this->get_parameter("wheels." + name + ".velocity_pid.kd").as_double();
        
        config.steering_pid.kp = this->get_parameter("wheels." + name + ".steering_pid.kp").as_double();
        config.steering_pid.ki = this->get_parameter("wheels." + name + ".steering_pid.ki").as_double();
        config.steering_pid.kd = this->get_parameter("wheels." + name + ".steering_pid.kd").as_double();
        
        wheel_configs_[name] = config;
        
        // Initialize current states
        current_wheel_velocities_[name] = 0.0;
        current_wheel_angles_[name] = 0.0;
        target_wheel_velocities_[name] = 0.0;
        target_wheel_angles_[name] = 0.0;
    }
}

void WheelController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (emergency_stop_) {
        return;
    }
    
    target_linear_x_ = std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    target_linear_y_ = std::clamp(msg->linear.y, -max_linear_velocity_, max_linear_velocity_);
    target_angular_z_ = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
    
    last_cmd_time_ = this->now();
    
    calculateWheelVelocities(target_linear_x_, target_linear_y_, target_angular_z_);
    calculateWheelSteering(target_linear_x_, target_linear_y_, target_angular_z_);
}

void WheelController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& joint_name = msg->name[i];
        
        // Parse wheel velocity joints
        if (joint_name.find("_wheel_joint") != std::string::npos) {
            std::string wheel_name = joint_name.substr(0, joint_name.find("_wheel_joint"));
            if (wheel_configs_.find(wheel_name) != wheel_configs_.end()) {
                if (i < msg->velocity.size()) {
                    current_wheel_velocities_[wheel_name] = msg->velocity[i];
                }
            }
        }
        
        // Parse steering joints
        if (joint_name.find("_steering_joint") != std::string::npos) {
            std::string wheel_name = joint_name.substr(0, joint_name.find("_steering_joint"));
            if (wheel_configs_.find(wheel_name) != wheel_configs_.end()) {
                if (i < msg->position.size()) {
                    current_wheel_angles_[wheel_name] = msg->position[i];
                }
            }
        }
    }
}

void WheelController::calculateWheelVelocities(double linear_x, double linear_y, double angular_z)
{
    // 4WD4WS inverse kinematics
    double half_wheelbase = wheelbase_ / 2.0;
    double half_track = track_width_ / 2.0;
    
    // Calculate wheel velocities for each corner
    target_wheel_velocities_["fl"] = (linear_x - angular_z * half_track) / wheel_radius_;
    target_wheel_velocities_["fr"] = (linear_x + angular_z * half_track) / wheel_radius_;
    target_wheel_velocities_["rl"] = (linear_x - angular_z * half_track) / wheel_radius_;
    target_wheel_velocities_["rr"] = (linear_x + angular_z * half_track) / wheel_radius_;
    
    // Limit wheel velocities
    for (auto& [name, velocity] : target_wheel_velocities_) {
        velocity = std::clamp(velocity, -wheel_configs_[name].max_velocity, 
                              wheel_configs_[name].max_velocity);
    }
}

void WheelController::calculateWheelSteering(double linear_x, double linear_y, double angular_z)
{
    if (std::abs(linear_x) < 1e-6 && std::abs(linear_y) < 1e-6 && std::abs(angular_z) < 1e-6) {
        // Zero motion - maintain current steering angles
        return;
    }
    
    double half_wheelbase = wheelbase_ / 2.0;
    double half_track = track_width_ / 2.0;
    
    if (std::abs(angular_z) < 1e-6) {
        // Straight line motion
        target_wheel_angles_["fl"] = 0.0;
        target_wheel_angles_["fr"] = 0.0;
        target_wheel_angles_["rl"] = 0.0;
        target_wheel_angles_["rr"] = 0.0;
    } else {
        // Calculate turning radius
        double turning_radius = linear_x / angular_z;
        
        // Ackermann steering geometry
        target_wheel_angles_["fl"] = std::atan(wheelbase_ / (turning_radius - half_track));
        target_wheel_angles_["fr"] = std::atan(wheelbase_ / (turning_radius + half_track));
        target_wheel_angles_["rl"] = -std::atan(wheelbase_ / (turning_radius - half_track));
        target_wheel_angles_["rr"] = -std::atan(wheelbase_ / (turning_radius + half_track));
    }
    
    // Limit steering angles
    for (auto& [name, angle] : target_wheel_angles_) {
        angle = std::clamp(angle, -wheel_configs_[name].max_steering_angle,
                           wheel_configs_[name].max_steering_angle);
    }
}

void WheelController::controlLoop()
{
    // Check command timeout
    auto current_time = this->now();
    if ((current_time - last_cmd_time_).seconds() > cmd_timeout_) {
        target_linear_x_ = 0.0;
        target_linear_y_ = 0.0;
        target_angular_z_ = 0.0;
        
        for (auto& [name, velocity] : target_wheel_velocities_) {
            velocity = 0.0;
        }
    }
    
    publishWheelCommands();
    publishWheelStates();
}

void WheelController::publishWheelCommands()
{
    auto fl_vel_msg = std_msgs::msg::Float64();
    auto fr_vel_msg = std_msgs::msg::Float64();
    auto rl_vel_msg = std_msgs::msg::Float64();
    auto rr_vel_msg = std_msgs::msg::Float64();
    
    fl_vel_msg.data = target_wheel_velocities_["fl"];
    fr_vel_msg.data = target_wheel_velocities_["fr"];
    rl_vel_msg.data = target_wheel_velocities_["rl"];
    rr_vel_msg.data = target_wheel_velocities_["rr"];
    
    fl_wheel_vel_pub_->publish(fl_vel_msg);
    fr_wheel_vel_pub_->publish(fr_vel_msg);
    rl_wheel_vel_pub_->publish(rl_vel_msg);
    rr_wheel_vel_pub_->publish(rr_vel_msg);
    
    auto fl_steer_msg = std_msgs::msg::Float64();
    auto fr_steer_msg = std_msgs::msg::Float64();
    auto rl_steer_msg = std_msgs::msg::Float64();
    auto rr_steer_msg = std_msgs::msg::Float64();
    
    fl_steer_msg.data = target_wheel_angles_["fl"];
    fr_steer_msg.data = target_wheel_angles_["fr"];
    rl_steer_msg.data = target_wheel_angles_["rl"];
    rr_steer_msg.data = target_wheel_angles_["rr"];
    
    fl_steering_pub_->publish(fl_steer_msg);
    fr_steering_pub_->publish(fr_steer_msg);
    rl_steering_pub_->publish(rl_steer_msg);
    rr_steering_pub_->publish(rr_steer_msg);
}

void WheelController::publishWheelStates()
{
    auto wheel_states_msg = tadeo_ecar_msgs::msg::WheelStates();
    wheel_states_msg.header.stamp = this->now();
    wheel_states_msg.header.frame_id = "base_link";
    
    wheel_states_msg.fl_velocity = current_wheel_velocities_["fl"];
    wheel_states_msg.fr_velocity = current_wheel_velocities_["fr"];
    wheel_states_msg.rl_velocity = current_wheel_velocities_["rl"];
    wheel_states_msg.rr_velocity = current_wheel_velocities_["rr"];
    
    wheel_states_msg.fl_steering_angle = current_wheel_angles_["fl"];
    wheel_states_msg.fr_steering_angle = current_wheel_angles_["fr"];
    wheel_states_msg.rl_steering_angle = current_wheel_angles_["rl"];
    wheel_states_msg.rr_steering_angle = current_wheel_angles_["rr"];
    
    // Calculate slip detection (simplified)
    for (const auto& [name, config] : wheel_configs_) {
        double velocity_error = std::abs(target_wheel_velocities_[name] - current_wheel_velocities_[name]);
        double slip_threshold = 0.5; // m/s
        
        if (name == "fl") wheel_states_msg.fl_slip_detected = velocity_error > slip_threshold;
        else if (name == "fr") wheel_states_msg.fr_slip_detected = velocity_error > slip_threshold;
        else if (name == "rl") wheel_states_msg.rl_slip_detected = velocity_error > slip_threshold;
        else if (name == "rr") wheel_states_msg.rr_slip_detected = velocity_error > slip_threshold;
    }
    
    wheel_states_pub_->publish(wheel_states_msg);
}

void WheelController::calibrateWheelsService(
    const std::shared_ptr<tadeo_ecar_interfaces::srv::CalibrateWheels::Request> request,
    std::shared_ptr<tadeo_ecar_interfaces::srv::CalibrateWheels::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Starting wheel calibration...");
    
    // Reset all PID controllers
    for (auto& [name, config] : wheel_configs_) {
        config.velocity_pid.reset();
        config.steering_pid.reset();
    }
    
    // Zero all wheel positions
    for (auto& [name, angle] : target_wheel_angles_) {
        angle = 0.0;
    }
    for (auto& [name, velocity] : target_wheel_velocities_) {
        velocity = 0.0;
    }
    
    is_calibrated_ = true;
    response->success = true;
    response->message = "Wheel calibration completed successfully";
    
    RCLCPP_INFO(this->get_logger(), "Wheel calibration completed");
}

} // namespace tadeo_ecar_control

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_control::WheelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}