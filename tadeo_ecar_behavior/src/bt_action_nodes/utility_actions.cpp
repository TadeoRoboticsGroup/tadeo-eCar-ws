#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tadeo_ecar_behavior/behavior_types.hpp"
#include <chrono>
#include <thread>

namespace tadeo_ecar_behavior
{

// Acción para esperar un tiempo determinado
class WaitAction : public BT::AsyncActionNode
{
public:
    WaitAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("wait_bt_node");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("duration", 1.0, "Wait duration in seconds"),
            BT::OutputPort<std::string>("result", "Wait result message")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!wait_started_) {
            double duration = 1.0;
            getInput("duration", duration);
            
            start_time_ = node_->get_clock()->now();
            wait_duration_ = duration;
            wait_started_ = true;
            
            RCLCPP_DEBUG(node_->get_logger(), "Starting wait for %.2f seconds", duration);
        }
        
        auto current_time = node_->get_clock()->now();
        double elapsed = (current_time - start_time_).seconds();
        
        if (elapsed >= wait_duration_) {
            setOutput("result", "Wait completed successfully");
            wait_started_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        
        return BT::NodeStatus::RUNNING;
    }

    void halt() override
    {
        wait_started_ = false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool wait_started_ = false;
    double wait_duration_;
    rclcpp::Time start_time_;
};

// Acción para enviar alertas
class SendAlertAction : public BT::SyncActionNode
{
public:
    SendAlertAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("send_alert_bt_node");
        alert_pub_ = node_->create_publisher<std_msgs::msg::String>("system_alerts", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("message", "Alert message to send"),
            BT::InputPort<std::string>("level", "INFO", "Alert level: INFO, WARN, ERROR, CRITICAL"),
            BT::OutputPort<std::string>("result", "Alert sending result")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string message, level;
        
        if (!getInput("message", message)) {
            setOutput("result", "No alert message provided");
            return BT::NodeStatus::FAILURE;
        }
        
        getInput("level", level);
        
        // Crear mensaje de alerta
        std_msgs::msg::String alert_msg;
        alert_msg.data = "[" + level + "] " + message;
        
        // Publicar alerta
        alert_pub_->publish(alert_msg);
        
        // Log según el nivel
        if (level == "CRITICAL") {
            RCLCPP_ERROR(node_->get_logger(), "ALERT: %s", message.c_str());
        } else if (level == "ERROR") {
            RCLCPP_ERROR(node_->get_logger(), "ALERT: %s", message.c_str());
        } else if (level == "WARN") {
            RCLCPP_WARN(node_->get_logger(), "ALERT: %s", message.c_str());
        } else {
            RCLCPP_INFO(node_->get_logger(), "ALERT: %s", message.c_str());
        }
        
        setOutput("result", "Alert sent successfully");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
};

// Acción para detener el robot
class StopRobotAction : public BT::SyncActionNode
{
public:
    StopRobotAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("stop_robot_bt_node");
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("result", "Stop robot result")
        };
    }

    BT::NodeStatus tick() override
    {
        // Enviar comando de velocidad cero
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.y = 0.0;
        stop_cmd.linear.z = 0.0;
        stop_cmd.angular.x = 0.0;
        stop_cmd.angular.y = 0.0;
        stop_cmd.angular.z = 0.0;
        
        // Publicar múltiples veces para asegurar que se reciba
        for (int i = 0; i < 5; ++i) {
            cmd_vel_pub_->publish(stop_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(node_->get_logger(), "Robot stopped");
        setOutput("result", "Robot stopped successfully");
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

// Acción para establecer el estado del robot
class SetRobotStateAction : public BT::SyncActionNode
{
public:
    SetRobotStateAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("set_robot_state_bt_node");
        state_pub_ = node_->create_publisher<std_msgs::msg::String>("robot_state", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("state", "Robot state to set"),
            BT::OutputPort<std::string>("result", "Set state result")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string state;
        
        if (!getInput("state", state)) {
            setOutput("result", "No state provided");
            return BT::NodeStatus::FAILURE;
        }
        
        // Validar estado
        std::vector<std::string> valid_states = {
            "IDLE", "INITIALIZING", "NAVIGATING", "EXPLORING", 
            "PATROLLING", "CHARGING", "EMERGENCY", "MAINTENANCE",
            "MISSION_EXECUTING", "WAITING", "RECOVERING", "DOCKING"
        };
        
        if (std::find(valid_states.begin(), valid_states.end(), state) == valid_states.end()) {
            setOutput("result", "Invalid robot state: " + state);
            return BT::NodeStatus::FAILURE;
        }
        
        // Publicar nuevo estado
        std_msgs::msg::String state_msg;
        state_msg.data = state;
        state_pub_->publish(state_msg);
        
        RCLCPP_INFO(node_->get_logger(), "Robot state set to: %s", state.c_str());
        setOutput("result", "Robot state set to " + state);
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

// Acción para solicitar asistencia humana
class RequestHumanAssistanceAction : public BT::SyncActionNode
{
public:
    RequestHumanAssistanceAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("request_assistance_bt_node");
        assistance_pub_ = node_->create_publisher<std_msgs::msg::String>("human_assistance_request", 10);
        alert_pub_ = node_->create_publisher<std_msgs::msg::String>("system_alerts", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("reason", "Assistance needed", "Reason for assistance"),
            BT::InputPort<std::string>("urgency", "MEDIUM", "Urgency level: LOW, MEDIUM, HIGH, CRITICAL"),
            BT::OutputPort<std::string>("result", "Request assistance result")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string reason, urgency;
        
        getInput("reason", reason);
        getInput("urgency", urgency);
        
        // Crear mensaje de solicitud de asistencia
        std_msgs::msg::String assistance_msg;
        assistance_msg.data = "URGENCY:" + urgency + " REASON:" + reason + 
                             " TIMESTAMP:" + std::to_string(node_->get_clock()->now().seconds());
        
        // Crear alerta del sistema
        std_msgs::msg::String alert_msg;
        alert_msg.data = "[ASSISTANCE] Human assistance requested - " + reason;
        
        // Publicar solicitud y alerta
        assistance_pub_->publish(assistance_msg);
        alert_pub_->publish(alert_msg);
        
        // Log según urgencia
        if (urgency == "CRITICAL") {
            RCLCPP_ERROR(node_->get_logger(), "CRITICAL: Human assistance requested - %s", reason.c_str());
        } else if (urgency == "HIGH") {
            RCLCPP_WARN(node_->get_logger(), "HIGH: Human assistance requested - %s", reason.c_str());
        } else {
            RCLCPP_INFO(node_->get_logger(), "Human assistance requested - %s", reason.c_str());
        }
        
        setOutput("result", "Human assistance request sent");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr assistance_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
};

// Acción para verificar el número de reintentos
class CheckRetryCountCondition : public BT::ConditionNode
{
public:
    CheckRetryCountCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("check_retry_count_bt_node");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("max_retries", 3, "Maximum number of retries"),
            BT::BidirectionalPort<int>("retry_count", "Current retry count"),
            BT::OutputPort<bool>("can_retry", "Can retry status")
        };
    }

    BT::NodeStatus tick() override
    {
        int max_retries = 3;
        int retry_count = 0;
        
        getInput("max_retries", max_retries);
        getInput("retry_count", retry_count);
        
        bool can_retry = retry_count < max_retries;
        setOutput("can_retry", can_retry);
        
        if (can_retry) {
            // Incrementar contador para próxima vez
            setOutput("retry_count", retry_count + 1);
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Acción para reiniciar contador de reintentos
class ResetRetryCountAction : public BT::SyncActionNode
{
public:
    ResetRetryCountAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("reset_retry_count_bt_node");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<int>("retry_count", "Reset retry count"),
            BT::OutputPort<std::string>("result", "Reset result")
        };
    }

    BT::NodeStatus tick() override
    {
        setOutput("retry_count", 0);
        setOutput("result", "Retry count reset to 0");
        
        RCLCPP_DEBUG(node_->get_logger(), "Retry count reset");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Acción para log de eventos importantes
class LogEventAction : public BT::SyncActionNode
{
public:
    LogEventAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("log_event_bt_node");
        event_pub_ = node_->create_publisher<std_msgs::msg::String>("behavior_events", 10);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("event", "Event to log"),
            BT::InputPort<std::string>("level", "INFO", "Log level"),
            BT::OutputPort<std::string>("result", "Log result")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string event, level;
        
        if (!getInput("event", event)) {
            setOutput("result", "No event provided");
            return BT::NodeStatus::FAILURE;
        }
        
        getInput("level", level);
        
        // Crear mensaje de evento
        std_msgs::msg::String event_msg;
        event_msg.data = "[" + level + "] " + event + " | " + 
                        std::to_string(node_->get_clock()->now().seconds());
        
        // Publicar evento
        event_pub_->publish(event_msg);
        
        // Log según nivel
        if (level == "ERROR") {
            RCLCPP_ERROR(node_->get_logger(), "EVENT: %s", event.c_str());
        } else if (level == "WARN") {
            RCLCPP_WARN(node_->get_logger(), "EVENT: %s", event.c_str());
        } else if (level == "DEBUG") {
            RCLCPP_DEBUG(node_->get_logger(), "EVENT: %s", event.c_str());
        } else {
            RCLCPP_INFO(node_->get_logger(), "EVENT: %s", event.c_str());
        }
        
        setOutput("result", "Event logged successfully");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
};

} // namespace tadeo_ecar_behavior