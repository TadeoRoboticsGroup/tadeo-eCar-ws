#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tadeo_ecar_msgs/msg/wheel_states.hpp>
#include <tadeo_ecar_msgs/msg/robot_status.hpp>
#include <cmath>
#include <array>

namespace tadeo_ecar_control
{

class VehicleDynamicsNode : public rclcpp::Node
{
public:
    VehicleDynamicsNode() : Node("vehicle_dynamics_node")
    {
        loadParameters();
        initializeState();
        
        // Create subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&VehicleDynamicsNode::cmdVelCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10,
            std::bind(&VehicleDynamicsNode::imuCallback, this, std::placeholders::_1));
        
        wheel_states_sub_ = this->create_subscription<tadeo_ecar_msgs::msg::WheelStates>(
            "wheel_states", 10,
            std::bind(&VehicleDynamicsNode::wheelStatesCallback, this, std::placeholders::_1));
        
        // Create publishers
        dynamics_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "dynamics/cmd_vel", 10);
        
        lateral_acceleration_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "dynamics/lateral_acceleration", 10);
        
        slip_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "dynamics/slip_angle", 10);
        
        stability_factor_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "dynamics/stability_factor", 10);
        
        // Create timer for dynamics calculation
        dynamics_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / dynamics_frequency_)),
            std::bind(&VehicleDynamicsNode::dynamicsLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Vehicle Dynamics Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("vehicle.mass", 150.0);
        this->declare_parameter("vehicle.inertia", 50.0);
        this->declare_parameter("vehicle.wheelbase", 1.2);
        this->declare_parameter("vehicle.track_width", 1.0);
        this->declare_parameter("vehicle.center_of_gravity_height", 0.3);
        this->declare_parameter("vehicle.front_axle_distance", 0.6);
        this->declare_parameter("vehicle.rear_axle_distance", 0.6);
        this->declare_parameter("dynamics.frequency", 100.0);
        this->declare_parameter("dynamics.max_lateral_acceleration", 8.0);
        this->declare_parameter("dynamics.max_slip_angle", 0.349); // 20 degrees
        this->declare_parameter("dynamics.stability_threshold", 0.8);
        this->declare_parameter("tires.cornering_stiffness_front", 100000.0);
        this->declare_parameter("tires.cornering_stiffness_rear", 120000.0);
        this->declare_parameter("tires.friction_coefficient", 0.8);
        
        vehicle_mass_ = this->get_parameter("vehicle.mass").as_double();
        vehicle_inertia_ = this->get_parameter("vehicle.inertia").as_double();
        wheelbase_ = this->get_parameter("vehicle.wheelbase").as_double();
        track_width_ = this->get_parameter("vehicle.track_width").as_double();
        cg_height_ = this->get_parameter("vehicle.center_of_gravity_height").as_double();
        front_axle_dist_ = this->get_parameter("vehicle.front_axle_distance").as_double();
        rear_axle_dist_ = this->get_parameter("vehicle.rear_axle_distance").as_double();
        dynamics_frequency_ = this->get_parameter("dynamics.frequency").as_double();
        max_lateral_accel_ = this->get_parameter("dynamics.max_lateral_acceleration").as_double();
        max_slip_angle_ = this->get_parameter("dynamics.max_slip_angle").as_double();
        stability_threshold_ = this->get_parameter("dynamics.stability_threshold").as_double();
        cf_front_ = this->get_parameter("tires.cornering_stiffness_front").as_double();
        cf_rear_ = this->get_parameter("tires.cornering_stiffness_rear").as_double();
        friction_coeff_ = this->get_parameter("tires.friction_coefficient").as_double();
    }
    
    void initializeState()
    {
        current_velocity_ = 0.0;
        current_angular_velocity_ = 0.0;
        lateral_acceleration_ = 0.0;
        slip_angle_ = 0.0;
        stability_factor_ = 1.0;
        last_dynamics_time_ = this->now();
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_linear_velocity_ = msg->linear.x;
        target_angular_velocity_ = msg->angular.z;
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extract lateral acceleration from IMU
        measured_lateral_accel_ = msg->linear_acceleration.y;
        measured_angular_velocity_ = msg->angular_velocity.z;
    }
    
    void wheelStatesCallback(const tadeo_ecar_msgs::msg::WheelStates::SharedPtr msg)
    {
        current_wheel_states_ = *msg;
        
        // Calculate actual vehicle velocity from wheel states
        double avg_wheel_velocity = (msg->fl_velocity + msg->fr_velocity + 
                                     msg->rl_velocity + msg->rr_velocity) / 4.0;
        current_velocity_ = avg_wheel_velocity * 0.15; // wheel radius
        
        // Calculate steering angle
        current_steering_angle_ = (msg->fl_steering_angle + msg->fr_steering_angle) / 2.0;
    }
    
    void dynamicsLoop()
    {
        auto current_time = this->now();
        double dt = (current_time - last_dynamics_time_).seconds();
        
        if (dt <= 0.0) return;
        
        calculateVehicleDynamics(dt);
        applyStabilityControl();
        publishDynamicsData();
        
        last_dynamics_time_ = current_time;
    }
    
    void calculateVehicleDynamics(double dt)
    {
        // Calculate slip angle (simplified bicycle model)
        if (std::abs(current_velocity_) > 0.1) {
            double front_slip = current_steering_angle_ - 
                                (current_angular_velocity_ * front_axle_dist_) / current_velocity_;
            double rear_slip = -(current_angular_velocity_ * rear_axle_dist_) / current_velocity_;
            
            slip_angle_ = std::atan2(current_angular_velocity_ * rear_axle_dist_, current_velocity_);
        } else {
            slip_angle_ = 0.0;
        }
        
        // Calculate lateral acceleration
        if (std::abs(current_velocity_) > 0.1 && std::abs(current_steering_angle_) > 0.01) {
            lateral_acceleration_ = (current_velocity_ * current_velocity_ * 
                                     std::tan(current_steering_angle_)) / wheelbase_;
        } else {
            lateral_acceleration_ = 0.0;
        }
        
        // Calculate stability factor
        calculateStabilityFactor();
        
        // Limit values to physical constraints
        lateral_acceleration_ = std::clamp(lateral_acceleration_, -max_lateral_accel_, max_lateral_accel_);
        slip_angle_ = std::clamp(slip_angle_, -max_slip_angle_, max_slip_angle_);
    }
    
    void calculateStabilityFactor()
    {
        // Simplified stability factor calculation
        double lateral_accel_ratio = std::abs(lateral_acceleration_) / max_lateral_accel_;
        double slip_angle_ratio = std::abs(slip_angle_) / max_slip_angle_;
        double velocity_ratio = std::abs(current_velocity_) / 5.0; // Assume max safe speed 5 m/s
        
        // Combine factors (higher values indicate lower stability)
        double instability = std::max({lateral_accel_ratio, slip_angle_ratio, velocity_ratio});
        stability_factor_ = 1.0 - std::clamp(instability, 0.0, 1.0);
        
        // Check for wheel slip
        bool any_wheel_slip = current_wheel_states_.fl_slip_detected ||
                              current_wheel_states_.fr_slip_detected ||
                              current_wheel_states_.rl_slip_detected ||
                              current_wheel_states_.rr_slip_detected;
        
        if (any_wheel_slip) {
            stability_factor_ *= 0.7; // Reduce stability factor if slipping
        }
    }
    
    void applyStabilityControl()
    {
        auto modified_cmd = geometry_msgs::msg::Twist();
        
        // Start with target velocities
        modified_cmd.linear.x = target_linear_velocity_;
        modified_cmd.angular.z = target_angular_velocity_;
        
        // Apply stability control if needed
        if (stability_factor_ < stability_threshold_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Stability control active - factor: %.2f", stability_factor_);
            
            // Reduce velocities based on stability factor
            double reduction_factor = stability_factor_ / stability_threshold_;
            modified_cmd.linear.x *= reduction_factor;
            modified_cmd.angular.z *= reduction_factor;
            
            // Additional lateral acceleration limiting
            if (std::abs(lateral_acceleration_) > max_lateral_accel_ * 0.8) {
                double accel_reduction = (max_lateral_accel_ * 0.8) / std::abs(lateral_acceleration_);
                modified_cmd.linear.x *= accel_reduction;
                modified_cmd.angular.z *= accel_reduction;
            }
        }
        
        // Apply slip control
        if (current_wheel_states_.fl_slip_detected || current_wheel_states_.fr_slip_detected ||
            current_wheel_states_.rl_slip_detected || current_wheel_states_.rr_slip_detected) {
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                  "Wheel slip detected - reducing power");
            
            modified_cmd.linear.x *= 0.8;
            modified_cmd.angular.z *= 0.9;
        }
        
        dynamics_twist_pub_->publish(modified_cmd);
    }
    
    void publishDynamicsData()
    {
        // Publish lateral acceleration
        auto lat_accel_msg = std_msgs::msg::Float64();
        lat_accel_msg.data = lateral_acceleration_;
        lateral_acceleration_pub_->publish(lat_accel_msg);
        
        // Publish slip angle
        auto slip_msg = std_msgs::msg::Float64();
        slip_msg.data = slip_angle_;
        slip_angle_pub_->publish(slip_msg);
        
        // Publish stability factor
        auto stability_msg = std_msgs::msg::Float64();
        stability_msg.data = stability_factor_;
        stability_factor_pub_->publish(stability_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::WheelStates>::SharedPtr wheel_states_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr dynamics_twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lateral_acceleration_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slip_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stability_factor_pub_;
    rclcpp::TimerInterface::SharedPtr dynamics_timer_;
    
    // Vehicle parameters
    double vehicle_mass_;
    double vehicle_inertia_;
    double wheelbase_;
    double track_width_;
    double cg_height_;
    double front_axle_dist_;
    double rear_axle_dist_;
    double dynamics_frequency_;
    double max_lateral_accel_;
    double max_slip_angle_;
    double stability_threshold_;
    double cf_front_;
    double cf_rear_;
    double friction_coeff_;
    
    // State variables
    double current_velocity_;
    double current_angular_velocity_;
    double current_steering_angle_;
    double target_linear_velocity_;
    double target_angular_velocity_;
    double lateral_acceleration_;
    double slip_angle_;
    double stability_factor_;
    double measured_lateral_accel_;
    double measured_angular_velocity_;
    rclcpp::Time last_dynamics_time_;
    
    tadeo_ecar_msgs::msg::WheelStates current_wheel_states_;
};

} // namespace tadeo_ecar_control

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_control::VehicleDynamicsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}