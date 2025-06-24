#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tadeo_ecar_msgs/msg/wheel_states.hpp>
#include <tadeo_ecar_msgs/msg/robot_status.hpp>
#include <cmath>

namespace tadeo_ecar_control
{

class FourWheelSteeringController : public rclcpp::Node
{
public:
    FourWheelSteeringController() : Node("four_wheel_steering_controller")
    {
        loadParameters();
        
        // Initialize state
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        linear_velocity_ = 0.0;
        angular_velocity_ = 0.0;
        last_time_ = this->now();
        
        // Create subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&FourWheelSteeringController::cmdVelCallback, this, std::placeholders::_1));
        
        wheel_states_sub_ = this->create_subscription<tadeo_ecar_msgs::msg::WheelStates>(
            "wheel_states", 10,
            std::bind(&FourWheelSteeringController::wheelStatesCallback, this, std::placeholders::_1));
        
        // Create publishers
        cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "cmd_vel_out", 10);
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        robot_status_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::RobotStatus>(
            "robot_status", 10);
        
        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
            std::bind(&FourWheelSteeringController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Four Wheel Steering Controller initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("robot.wheelbase", 1.2);
        this->declare_parameter("robot.track_width", 1.0);
        this->declare_parameter("robot.wheel_radius", 0.15);
        this->declare_parameter("control.frequency", 50.0);
        this->declare_parameter("control.enable_4ws", true);
        this->declare_parameter("control.4ws_speed_threshold", 0.5);
        this->declare_parameter("control.max_steering_angle", 0.785);
        this->declare_parameter("odom.publish_tf", true);
        this->declare_parameter("odom.frame_id", "odom");
        this->declare_parameter("odom.child_frame_id", "base_link");
        
        wheelbase_ = this->get_parameter("robot.wheelbase").as_double();
        track_width_ = this->get_parameter("robot.track_width").as_double();
        wheel_radius_ = this->get_parameter("robot.wheel_radius").as_double();
        control_frequency_ = this->get_parameter("control.frequency").as_double();
        enable_4ws_ = this->get_parameter("control.enable_4ws").as_bool();
        speed_threshold_4ws_ = this->get_parameter("control.4ws_speed_threshold").as_double();
        max_steering_angle_ = this->get_parameter("control.max_steering_angle").as_double();
        publish_tf_ = this->get_parameter("odom.publish_tf").as_bool();
        odom_frame_id_ = this->get_parameter("odom.frame_id").as_string();
        base_frame_id_ = this->get_parameter("odom.child_frame_id").as_string();
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_linear_x_ = msg->linear.x;
        target_linear_y_ = msg->linear.y;
        target_angular_z_ = msg->angular.z;
        
        last_cmd_time_ = this->now();
        
        processCommand();
    }
    
    void wheelStatesCallback(const tadeo_ecar_msgs::msg::WheelStates::SharedPtr msg)
    {
        current_wheel_states_ = *msg;
        updateOdometry();
    }
    
    void processCommand()
    {
        auto cmd_out = geometry_msgs::msg::TwistStamped();
        cmd_out.header.stamp = this->now();
        cmd_out.header.frame_id = base_frame_id_;
        
        // Determine steering mode based on speed and user preference
        bool use_4ws = enable_4ws_ && (std::abs(target_linear_x_) < speed_threshold_4ws_);
        
        if (use_4ws) {
            // Four wheel steering mode for low speeds
            process4WSCommand(cmd_out.twist);
        } else {
            // Front wheel steering mode for higher speeds
            processFWSCommand(cmd_out.twist);
        }
        
        cmd_vel_out_pub_->publish(cmd_out);
    }
    
    void process4WSCommand(geometry_msgs::msg::Twist& cmd)
    {
        // 4WS provides enhanced maneuverability at low speeds
        // Rear wheels steer opposite to front wheels for crab walk capability
        
        cmd.linear.x = target_linear_x_;
        cmd.linear.y = target_linear_y_;
        
        // Modify angular velocity for 4WS characteristics
        if (std::abs(target_linear_y_) > 1e-3) {
            // Crab walk mode - reduce angular component
            cmd.angular.z = target_angular_z_ * 0.5;
        } else {
            // Normal turning with 4WS
            cmd.angular.z = target_angular_z_;
        }
        
        current_steering_mode_ = "4WS";
    }
    
    void processFWSCommand(geometry_msgs::msg::Twist& cmd)
    {
        // Front wheel steering mode
        cmd.linear.x = target_linear_x_;
        cmd.linear.y = 0.0; // No lateral movement in FWS mode
        cmd.angular.z = target_angular_z_;
        
        current_steering_mode_ = "FWS";
    }
    
    void updateOdometry()
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        
        if (dt <= 0.0) return;
        
        // Calculate vehicle velocity from wheel states
        calculateVehicleVelocity();
        
        // Integrate position
        double delta_x = linear_velocity_ * cos(theta_) * dt;
        double delta_y = linear_velocity_ * sin(theta_) * dt;
        double delta_theta = angular_velocity_ * dt;
        
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;
        
        // Normalize theta
        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
        
        publishOdometry(current_time);
        
        last_time_ = current_time;
    }
    
    void calculateVehicleVelocity()
    {
        // Calculate vehicle velocity from wheel velocities
        double avg_wheel_velocity = (current_wheel_states_.fl_velocity + 
                                     current_wheel_states_.fr_velocity +
                                     current_wheel_states_.rl_velocity + 
                                     current_wheel_states_.rr_velocity) / 4.0;
        
        linear_velocity_ = avg_wheel_velocity * wheel_radius_;
        
        // Calculate angular velocity from wheel steering angles and velocities
        double front_avg_angle = (current_wheel_states_.fl_steering_angle + 
                                  current_wheel_states_.fr_steering_angle) / 2.0;
        
        if (std::abs(front_avg_angle) > 1e-6 && std::abs(linear_velocity_) > 1e-6) {
            angular_velocity_ = linear_velocity_ * tan(front_avg_angle) / wheelbase_;
        } else {
            angular_velocity_ = 0.0;
        }
    }
    
    void publishOdometry(const rclcpp::Time& current_time)
    {
        // Create odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;
        
        // Position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);
        
        // Velocity
        odom_msg.twist.twist.linear.x = linear_velocity_;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_velocity_;
        
        // Covariance (simplified)
        for (int i = 0; i < 36; ++i) {
            odom_msg.pose.covariance[i] = 0.0;
            odom_msg.twist.covariance[i] = 0.0;
        }
        odom_msg.pose.covariance[0] = 0.01;   // x
        odom_msg.pose.covariance[7] = 0.01;   // y
        odom_msg.pose.covariance[35] = 0.01;  // yaw
        odom_msg.twist.covariance[0] = 0.01;  // linear.x
        odom_msg.twist.covariance[35] = 0.01; // angular.z
        
        odom_pub_->publish(odom_msg);
        
        // Publish transform
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = current_time;
            transform.header.frame_id = odom_frame_id_;
            transform.child_frame_id = base_frame_id_;
            
            transform.transform.translation.x = x_;
            transform.transform.translation.y = y_;
            transform.transform.translation.z = 0.0;
            transform.transform.rotation = odom_msg.pose.pose.orientation;
            
            tf_broadcaster_->sendTransform(transform);
        }
    }
    
    void controlLoop()
    {
        publishRobotStatus();
    }
    
    void publishRobotStatus()
    {
        auto status_msg = tadeo_ecar_msgs::msg::RobotStatus();
        status_msg.header.stamp = this->now();
        status_msg.header.frame_id = base_frame_id_;
        
        // Operational state
        status_msg.operational_state = "ACTIVE";
        status_msg.current_mode = current_steering_mode_;
        
        // Performance metrics
        status_msg.current_speed = linear_velocity_;
        status_msg.target_speed = target_linear_x_;
        status_msg.steering_angle = (current_wheel_states_.fl_steering_angle + 
                                     current_wheel_states_.fr_steering_angle) / 2.0;
        
        // Distance traveled (simplified integration)
        static double total_distance = 0.0;
        static auto last_status_time = this->now();
        double dt = (this->now() - last_status_time).seconds();
        total_distance += std::abs(linear_velocity_) * dt;
        status_msg.distance_traveled = total_distance;
        last_status_time = this->now();
        
        // System status
        status_msg.battery_voltage = 24.0; // Placeholder
        status_msg.battery_percentage = 85.0; // Placeholder
        status_msg.system_temperature = 35.0; // Placeholder
        
        robot_status_pub_->publish(status_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::WheelStates>::SharedPtr wheel_states_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_out_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::TimerInterface::SharedPtr control_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Parameters
    double wheelbase_;
    double track_width_;
    double wheel_radius_;
    double control_frequency_;
    bool enable_4ws_;
    double speed_threshold_4ws_;
    double max_steering_angle_;
    bool publish_tf_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    
    // State variables
    double x_, y_, theta_;
    double linear_velocity_, angular_velocity_;
    double target_linear_x_, target_linear_y_, target_angular_z_;
    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_time_;
    std::string current_steering_mode_;
    
    tadeo_ecar_msgs::msg::WheelStates current_wheel_states_;
};

} // namespace tadeo_ecar_control

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_control::FourWheelSteeringController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}