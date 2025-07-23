#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tadeo_ecar_interfaces/action/navigate_to_goal.hpp>
#include "tadeo_ecar_behavior/behavior_types.hpp"
#include <cmath>

namespace tadeo_ecar_behavior
{

// Acción para navegar a un objetivo específico
class NavigateToGoalAction : public BT::AsyncActionNode
{
public:
    NavigateToGoalAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("navigate_to_goal_bt_node");
        action_client_ = rclcpp_action::create_client<tadeo_ecar_interfaces::action::NavigateToGoal>(
            node_, "navigate_to_goal");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("goal_x", "X coordinate of the goal"),
            BT::InputPort<double>("goal_y", "Y coordinate of the goal"), 
            BT::InputPort<double>("goal_theta", "Orientation of the goal"),
            BT::InputPort<double>("timeout", 120.0, "Navigation timeout in seconds"),
            BT::OutputPort<std::string>("result", "Navigation result message")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            setOutput("result", "Navigation action server not available");
            return BT::NodeStatus::FAILURE;
        }

        double goal_x, goal_y, goal_theta;
        double timeout = 120.0;
        
        if (!getInput("goal_x", goal_x) || !getInput("goal_y", goal_y)) {
            setOutput("result", "Missing goal coordinates");
            return BT::NodeStatus::FAILURE;
        }
        
        getInput("goal_theta", goal_theta);
        getInput("timeout", timeout);

        auto goal_msg = tadeo_ecar_interfaces::action::NavigateToGoal::Goal();
        goal_msg.target_pose.header.frame_id = "map";
        goal_msg.target_pose.header.stamp = node_->get_clock()->now();
        goal_msg.target_pose.pose.position.x = goal_x;
        goal_msg.target_pose.pose.position.y = goal_y;
        goal_msg.target_pose.pose.position.z = 0.0;
        
        // Convert theta to quaternion
        goal_msg.target_pose.pose.orientation.x = 0.0;
        goal_msg.target_pose.pose.orientation.y = 0.0;
        goal_msg.target_pose.pose.orientation.z = sin(goal_theta / 2.0);
        goal_msg.target_pose.pose.orientation.w = cos(goal_theta / 2.0);
        
        // Set default parameters
        goal_msg.tolerance_distance = 0.2;
        goal_msg.tolerance_angle = 0.1;
        goal_msg.max_speed = 1.0;
        goal_msg.use_obstacle_avoidance = true;

        auto send_goal_options = rclcpp_action::Client<tadeo_ecar_interfaces::action::NavigateToGoal>::SendGoalOptions();
        send_goal_options.result_callback = [this](const auto& result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                navigation_result_ = "Navigation succeeded";
                navigation_success_ = true;
            } else {
                navigation_result_ = "Navigation failed";
                navigation_success_ = false;
            }
            navigation_complete_ = true;
        };

        navigation_complete_ = false;
        goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
        start_time_ = node_->get_clock()->now();

        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        if (goal_handle_future_.valid()) {
            auto goal_handle = goal_handle_future_.get();
            if (goal_handle) {
                action_client_->async_cancel_goal(goal_handle);
            }
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<tadeo_ecar_interfaces::action::NavigateToGoal>::SharedPtr action_client_;
    std::shared_future<rclcpp_action::Client<tadeo_ecar_interfaces::action::NavigateToGoal>::GoalHandle::SharedPtr> goal_handle_future_;
    bool navigation_complete_ = false;
    bool navigation_success_ = false;
    std::string navigation_result_;
    rclcpp::Time start_time_;
};

// Acción para navegar a un waypoint específico
class NavigateToWaypointAction : public BT::AsyncActionNode
{
public:
    NavigateToWaypointAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("navigate_to_waypoint_bt_node");
        action_client_ = rclcpp_action::create_client<tadeo_ecar_interfaces::action::NavigateToGoal>(
            node_, "navigate_to_goal");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("waypoint_id", "ID of the waypoint to navigate to"),
            BT::OutputPort<std::string>("result", "Navigation result message")
        };
    }

    BT::NodeStatus tick() override
    {
        // Implementation similar to NavigateToGoalAction
        // but retrieves waypoint coordinates from a waypoint database
        return BT::NodeStatus::SUCCESS; // Placeholder
    }

    void halt() override
    {
        // Implementation for canceling navigation
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<tadeo_ecar_interfaces::action::NavigateToGoal>::SharedPtr action_client_;
};

// Acción para limpiar costmaps
class ClearCostmapsAction : public BT::SyncActionNode
{
public:
    ClearCostmapsAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("clear_costmaps_bt_node");
        clear_global_client_ = node_->create_client<std_srvs::srv::Empty>("global_costmap/clear_entirely_global_costmap");
        clear_local_client_ = node_->create_client<std_srvs::srv::Empty>("local_costmap/clear_entirely_local_costmap");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("result", "Clear costmaps result")
        };
    }

    BT::NodeStatus tick() override
    {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        
        bool global_cleared = false;
        bool local_cleared = false;
        
        if (clear_global_client_->wait_for_service(std::chrono::seconds(2))) {
            auto future = clear_global_client_->async_send_request(request);
            if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                global_cleared = true;
            }
        }
        
        if (clear_local_client_->wait_for_service(std::chrono::seconds(2))) {
            auto future = clear_local_client_->async_send_request(request);
            if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                local_cleared = true;
            }
        }
        
        if (global_cleared && local_cleared) {
            setOutput("result", "Costmaps cleared successfully");
            return BT::NodeStatus::SUCCESS;
        } else {
            setOutput("result", "Failed to clear costmaps");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_global_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_local_client_;
};

// Acción de recovery por rotación
class RotateRecoveryAction : public BT::AsyncActionNode
{
public:
    RotateRecoveryAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("rotate_recovery_bt_node");
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("rotation_speed", 0.5, "Rotation speed in rad/s"),
            BT::InputPort<double>("rotation_angle", 6.28, "Total rotation angle in radians"),
            BT::OutputPort<std::string>("result", "Rotation recovery result")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!rotation_started_) {
            double rotation_speed, rotation_angle;
            getInput("rotation_speed", rotation_speed);
            getInput("rotation_angle", rotation_angle);
            
            rotation_duration_ = std::abs(rotation_angle / rotation_speed);
            start_time_ = node_->get_clock()->now();
            rotation_started_ = true;
            
            // Start rotation
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.angular.z = rotation_speed;
            cmd_vel_pub_->publish(cmd_vel);
        }
        
        auto current_time = node_->get_clock()->now();
        double elapsed = (current_time - start_time_).seconds();
        
        if (elapsed >= rotation_duration_) {
            // Stop rotation
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel_pub_->publish(cmd_vel);
            
            setOutput("result", "Rotation recovery completed");
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        // Stop rotation
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel_pub_->publish(cmd_vel);
        rotation_started_ = false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    bool rotation_started_ = false;
    double rotation_duration_;
    rclcpp::Time start_time_;
};

// Acción de recovery por retroceso
class BackupRecoveryAction : public BT::AsyncActionNode
{
public:
    BackupRecoveryAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("backup_recovery_bt_node");
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("backup_speed", 0.3, "Backup speed in m/s"),
            BT::InputPort<double>("backup_distance", 0.5, "Backup distance in meters"),
            BT::OutputPort<std::string>("result", "Backup recovery result")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!backup_started_) {
            double backup_speed, backup_distance;
            getInput("backup_speed", backup_speed);
            getInput("backup_distance", backup_distance);
            
            backup_duration_ = backup_distance / backup_speed;
            start_time_ = node_->get_clock()->now();
            backup_started_ = true;
            
            // Start backup
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = -backup_speed; // Negative for backward
            cmd_vel_pub_->publish(cmd_vel);
        }
        
        auto current_time = node_->get_clock()->now();
        double elapsed = (current_time - start_time_).seconds();
        
        if (elapsed >= backup_duration_) {
            // Stop backup
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel_pub_->publish(cmd_vel);
            
            setOutput("result", "Backup recovery completed");
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        // Stop backup
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel_pub_->publish(cmd_vel);
        backup_started_ = false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    bool backup_started_ = false;
    double backup_duration_;
    rclcpp::Time start_time_;
};

} // namespace tadeo_ecar_behavior