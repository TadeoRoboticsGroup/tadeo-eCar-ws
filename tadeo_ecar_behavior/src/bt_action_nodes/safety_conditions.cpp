#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include "tadeo_ecar_behavior/behavior_types.hpp"

namespace tadeo_ecar_behavior
{

// Condición para verificar el estado de emergencia
class CheckEmergencyCondition : public BT::ConditionNode
{
public:
    CheckEmergencyCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("check_emergency_bt_node");
        emergency_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "emergency_stop", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                emergency_active_ = msg->data;
                last_update_ = node_->get_clock()->now();
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("emergency_status", "Current emergency status")
        };
    }

    BT::NodeStatus tick() override
    {
        // Verificar si los datos son recientes (últimos 2 segundos)
        auto current_time = node_->get_clock()->now();
        double time_since_update = (current_time - last_update_).seconds();
        
        if (time_since_update > 2.0) {
            // Datos obsoletos, asumir emergencia por seguridad
            setOutput("emergency_status", true);
            return BT::NodeStatus::FAILURE;
        }
        
        setOutput("emergency_status", emergency_active_);
        
        // Retornar SUCCESS si NO hay emergencia, FAILURE si hay emergencia
        return emergency_active_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
    bool emergency_active_ = false;
    rclcpp::Time last_update_;
};

// Condición para verificar el nivel de batería
class CheckBatteryCondition : public BT::ConditionNode
{
public:
    CheckBatteryCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("check_battery_bt_node");
        battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10,
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                if (msg->capacity > 0) {
                    battery_level_ = msg->charge / msg->capacity;
                } else {
                    battery_level_ = msg->percentage / 100.0;
                }
                last_update_ = node_->get_clock()->now();
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("threshold", 0.2, "Battery level threshold (0.0-1.0)"),
            BT::OutputPort<double>("battery_level", "Current battery level"),
            BT::OutputPort<bool>("battery_ok", "Battery status OK")
        };
    }

    BT::NodeStatus tick() override
    {
        double threshold = 0.2;
        getInput("threshold", threshold);
        
        // Verificar si los datos son recientes
        auto current_time = node_->get_clock()->now();
        double time_since_update = (current_time - last_update_).seconds();
        
        if (time_since_update > 5.0) {
            // Datos de batería obsoletos
            setOutput("battery_level", 0.0);
            setOutput("battery_ok", false);
            return BT::NodeStatus::FAILURE;
        }
        
        setOutput("battery_level", battery_level_);
        bool battery_ok = battery_level_ > threshold;
        setOutput("battery_ok", battery_ok);
        
        return battery_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    double battery_level_ = 1.0;
    rclcpp::Time last_update_;
};

// Condición para verificar si los sensores están funcionando
class CheckSensorsHealthyCondition : public BT::ConditionNode
{
public:
    CheckSensorsHealthyCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("check_sensors_bt_node");
        
        // Suscribirse a múltiples tópicos de salud del sistema
        laser_health_sub_ = node_->create_subscription<tadeo_ecar_msgs::msg::SystemHealth>(
            "perception/laser_health", 10,
            [this](const tadeo_ecar_msgs::msg::SystemHealth::SharedPtr msg) {
                laser_healthy_ = (msg->lidar_status == tadeo_ecar_msgs::msg::SystemHealth::HEALTHY);
                last_laser_update_ = node_->get_clock()->now();
            });
            
        camera_health_sub_ = node_->create_subscription<tadeo_ecar_msgs::msg::SystemHealth>(
            "perception/camera_health", 10,
            [this](const tadeo_ecar_msgs::msg::SystemHealth::SharedPtr msg) {
                camera_healthy_ = (msg->camera_status == tadeo_ecar_msgs::msg::SystemHealth::HEALTHY);
                last_camera_update_ = node_->get_clock()->now();
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("sensors_ok", "Sensors health status"),
            BT::OutputPort<std::string>("failed_sensors", "List of failed sensors")
        };
    }

    BT::NodeStatus tick() override
    {
        auto current_time = node_->get_clock()->now();
        std::vector<std::string> failed_sensors;
        
        // Verificar laser
        if ((current_time - last_laser_update_).seconds() > 5.0 || !laser_healthy_) {
            failed_sensors.push_back("laser");
        }
        
        // Verificar cámara
        if ((current_time - last_camera_update_).seconds() > 5.0 || !camera_healthy_) {
            failed_sensors.push_back("camera");
        }
        
        bool sensors_ok = failed_sensors.empty();
        setOutput("sensors_ok", sensors_ok);
        
        std::string failed_list;
        for (const auto& sensor : failed_sensors) {
            if (!failed_list.empty()) failed_list += ", ";
            failed_list += sensor;
        }
        setOutput("failed_sensors", failed_list);
        
        return sensors_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr laser_health_sub_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr camera_health_sub_;
    
    bool laser_healthy_ = true;
    bool camera_healthy_ = true;
    rclcpp::Time last_laser_update_;
    rclcpp::Time last_camera_update_;
};

// Condición para verificar si la navegación está lista
class CheckNavigationReadyCondition : public BT::ConditionNode
{
public:
    CheckNavigationReadyCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("check_navigation_ready_bt_node");
        
        nav_health_sub_ = node_->create_subscription<tadeo_ecar_msgs::msg::SystemHealth>(
            "navigation/controller_health", 10,
            [this](const tadeo_ecar_msgs::msg::SystemHealth::SharedPtr msg) {
                navigation_healthy_ = (msg->cpu_status == tadeo_ecar_msgs::msg::SystemHealth::HEALTHY);
                last_nav_update_ = node_->get_clock()->now();
            });
            
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_available_ = true;
                last_odom_update_ = node_->get_clock()->now();
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("navigation_ready", "Navigation system ready status")
        };
    }

    BT::NodeStatus tick() override
    {
        auto current_time = node_->get_clock()->now();
        
        // Verificar salud de navegación
        bool nav_ok = navigation_healthy_ && 
                     (current_time - last_nav_update_).seconds() < 5.0;
        
        // Verificar odometría
        bool odom_ok = odom_available_ && 
                      (current_time - last_odom_update_).seconds() < 2.0;
        
        bool navigation_ready = nav_ok && odom_ok;
        setOutput("navigation_ready", navigation_ready);
        
        return navigation_ready ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr nav_health_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    bool navigation_healthy_ = false;
    bool odom_available_ = false;
    rclcpp::Time last_nav_update_;
    rclcpp::Time last_odom_update_;
};

// Condición para verificar si hay una misión activa
class CheckActiveMissionCondition : public BT::ConditionNode
{
public:
    CheckActiveMissionCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("check_active_mission_bt_node");
        
        mission_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "mission_status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                mission_status_ = msg->data;
                last_mission_update_ = node_->get_clock()->now();
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("mission_active", "Mission active status"),
            BT::OutputPort<std::string>("mission_status", "Current mission status")
        };
    }

    BT::NodeStatus tick() override
    {
        auto current_time = node_->get_clock()->now();
        
        // Verificar si los datos de misión son recientes
        bool data_fresh = (current_time - last_mission_update_).seconds() < 5.0;
        
        bool mission_active = data_fresh && 
                             (mission_status_ == "EXECUTING" || 
                              mission_status_ == "LOADED" ||
                              mission_status_ == "PAUSED");
        
        setOutput("mission_active", mission_active);
        setOutput("mission_status", mission_status_);
        
        return mission_active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub_;
    std::string mission_status_ = "IDLE";
    rclcpp::Time last_mission_update_;
};

// Condición para verificar si el patrullaje está habilitado
class CheckPatrolModeCondition : public BT::ConditionNode
{
public:
    CheckPatrolModeCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("check_patrol_mode_bt_node");
        
        // Verificar parámetro de patrullaje o estado del blackboard
        patrol_enabled_ = false; // Por defecto deshabilitado
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("patrol_enabled", false, "Patrol mode enabled"),
            BT::OutputPort<bool>("patrol_active", "Patrol mode status")
        };
    }

    BT::NodeStatus tick() override
    {
        bool patrol_enabled = false;
        getInput("patrol_enabled", patrol_enabled);
        
        setOutput("patrol_active", patrol_enabled);
        
        return patrol_enabled ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool patrol_enabled_;
};

// Condición para verificar si la exploración está habilitada
class CheckExplorationModeCondition : public BT::ConditionNode
{
public:
    CheckExplorationModeCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("check_exploration_mode_bt_node");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("exploration_enabled", false, "Exploration mode enabled"),
            BT::OutputPort<bool>("exploration_active", "Exploration mode status")
        };
    }

    BT::NodeStatus tick() override
    {
        bool exploration_enabled = false;
        getInput("exploration_enabled", exploration_enabled);
        
        setOutput("exploration_active", exploration_enabled);
        
        return exploration_enabled ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Condición para verificar si hay un objetivo de navegación
class CheckNavigationGoalCondition : public BT::ConditionNode
{
public:
    CheckNavigationGoalCondition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("check_navigation_goal_bt_node");
        
        goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "move_base_simple/goal", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                has_goal_ = true;
                last_goal_time_ = node_->get_clock()->now();
            });
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<bool>("has_goal", "Navigation goal available"),
            BT::InputPort<double>("goal_timeout", 300.0, "Goal timeout in seconds")
        };
    }

    BT::NodeStatus tick() override
    {
        double goal_timeout = 300.0; // 5 minutos por defecto
        getInput("goal_timeout", goal_timeout);
        
        auto current_time = node_->get_clock()->now();
        double time_since_goal = (current_time - last_goal_time_).seconds();
        
        bool goal_valid = has_goal_ && (time_since_goal < goal_timeout);
        setOutput("has_goal", goal_valid);
        
        return goal_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    bool has_goal_ = false;
    rclcpp::Time last_goal_time_;
};

} // namespace tadeo_ecar_behavior