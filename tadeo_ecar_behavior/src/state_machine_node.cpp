#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include "tadeo_ecar_behavior/behavior_types.hpp"
#include <memory>
#include <map>
#include <chrono>

namespace tadeo_ecar_behavior
{

class StateMachineNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    StateMachineNode() : LifecycleNode("state_machine_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing State Machine Node");
        declareParameters();
    }

protected:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Configuring State Machine Node");
        
        loadParameters();
        createSubscribers();
        createPublishers();
        initializeStateMachine();
        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Activating State Machine Node");
        
        state_pub_->on_activate();
        transition_pub_->on_activate();
        health_pub_->on_activate();
        
        state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency_)),
            std::bind(&StateMachineNode::updateStateMachine, this));
        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating State Machine Node");
        
        state_pub_->on_deactivate();
        transition_pub_->on_deactivate();
        health_pub_->on_deactivate();
        state_timer_.reset();
        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void declareParameters()
    {
        this->declare_parameter("update_frequency", 5.0);
        this->declare_parameter("enable_auto_transitions", true);
        this->declare_parameter("critical_battery_level", 0.15);
        this->declare_parameter("low_battery_level", 0.25);
        this->declare_parameter("base_frame", "base_link");
    }
    
    void loadParameters()
    {
        update_frequency_ = this->get_parameter("update_frequency").as_double();
        enable_auto_transitions_ = this->get_parameter("enable_auto_transitions").as_bool();
        critical_battery_level_ = this->get_parameter("critical_battery_level").as_double();
        low_battery_level_ = this->get_parameter("low_battery_level").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
    }
    
    void createSubscribers()
    {
        // Estado de emergencia
        emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "emergency_stop", 10,
            std::bind(&StateMachineNode::emergencyCallback, this, std::placeholders::_1));
        
        // Estado de batería
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10,
            std::bind(&StateMachineNode::batteryCallback, this, std::placeholders::_1));
        
        // Comandos de estado
        state_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "state_command", 10,
            std::bind(&StateMachineNode::stateCommandCallback, this, std::placeholders::_1));
        
        // Estado de navegación
        navigation_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "navigation_status", 10,
            std::bind(&StateMachineNode::navigationStatusCallback, this, std::placeholders::_1));
        
        // Estado de misión
        mission_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "mission_status", 10,
            std::bind(&StateMachineNode::missionStatusCallback, this, std::placeholders::_1));
    }
    
    void createPublishers()
    {
        state_pub_ = this->create_publisher<std_msgs::msg::String>("robot_state", 10);
        transition_pub_ = this->create_publisher<std_msgs::msg::String>("state_transitions", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("behavior/state_health", 10);
    }
    
    void initializeStateMachine()
    {
        current_state_ = RobotState::IDLE;
        previous_state_ = RobotState::UNKNOWN;
        last_transition_time_ = this->now();
        
        // Definir transiciones válidas
        setupValidTransitions();
        
        RCLCPP_INFO(this->get_logger(), "State machine initialized in IDLE state");
    }
    
    void setupValidTransitions()
    {
        // Definir todas las transiciones válidas del estado
        valid_transitions_[RobotState::IDLE] = {
            RobotState::INITIALIZING,
            RobotState::NAVIGATING,
            RobotState::EXPLORING,
            RobotState::PATROLLING,
            RobotState::CHARGING,
            RobotState::EMERGENCY,
            RobotState::MAINTENANCE
        };
        
        valid_transitions_[RobotState::INITIALIZING] = {
            RobotState::IDLE,
            RobotState::EMERGENCY
        };
        
        valid_transitions_[RobotState::NAVIGATING] = {
            RobotState::IDLE,
            RobotState::MISSION_EXECUTING,
            RobotState::EMERGENCY,
            RobotState::RECOVERING,
            RobotState::CHARGING
        };
        
        valid_transitions_[RobotState::EXPLORING] = {
            RobotState::IDLE,
            RobotState::NAVIGATING,
            RobotState::EMERGENCY,
            RobotState::RECOVERING,
            RobotState::CHARGING
        };
        
        valid_transitions_[RobotState::PATROLLING] = {
            RobotState::IDLE,
            RobotState::NAVIGATING,
            RobotState::EMERGENCY,
            RobotState::RECOVERING,
            RobotState::CHARGING
        };
        
        valid_transitions_[RobotState::CHARGING] = {
            RobotState::IDLE,
            RobotState::DOCKING,
            RobotState::EMERGENCY
        };
        
        valid_transitions_[RobotState::EMERGENCY] = {
            RobotState::IDLE,
            RobotState::RECOVERING,
            RobotState::MAINTENANCE
        };
        
        valid_transitions_[RobotState::MAINTENANCE] = {
            RobotState::IDLE,
            RobotState::EMERGENCY
        };
        
        valid_transitions_[RobotState::MISSION_EXECUTING] = {
            RobotState::IDLE,
            RobotState::NAVIGATING,
            RobotState::EMERGENCY,
            RobotState::RECOVERING,
            RobotState::WAITING
        };
        
        valid_transitions_[RobotState::WAITING] = {
            RobotState::IDLE,
            RobotState::MISSION_EXECUTING,
            RobotState::EMERGENCY
        };
        
        valid_transitions_[RobotState::RECOVERING] = {
            RobotState::IDLE,
            RobotState::NAVIGATING,
            RobotState::EMERGENCY
        };
        
        valid_transitions_[RobotState::DOCKING] = {
            RobotState::CHARGING,
            RobotState::IDLE,
            RobotState::EMERGENCY
        };
    }
    
    void updateStateMachine()
    {
        // Verificar transiciones automáticas si están habilitadas
        if (enable_auto_transitions_) {
            checkAutoTransitions();
        }
        
        // Publicar estado actual
        publishCurrentState();
        publishHealthStatus();
    }
    
    void checkAutoTransitions()
    {
        RobotState new_state = current_state_;
        
        // Prioridad 1: Emergencia
        if (emergency_active_) {
            new_state = RobotState::EMERGENCY;
        }
        // Prioridad 2: Batería crítica
        else if (battery_level_ < critical_battery_level_ && current_state_ != RobotState::CHARGING) {
            new_state = RobotState::CHARGING;
        }
        // Prioridad 3: Transiciones basadas en estado actual
        else {
            switch (current_state_) {
                case RobotState::EMERGENCY:
                    if (!emergency_active_) {
                        new_state = RobotState::RECOVERING;
                    }
                    break;
                    
                case RobotState::RECOVERING:
                    // Después de recovery, volver a idle
                    if (isRecoveryComplete()) {
                        new_state = RobotState::IDLE;
                    }
                    break;
                    
                case RobotState::CHARGING:
                    // Salir de carga cuando batería esté suficientemente cargada
                    if (battery_level_ > 0.9) {
                        new_state = RobotState::IDLE;
                    }
                    break;
                    
                case RobotState::NAVIGATING:
                    // Transición basada en estado de navegación
                    if (navigation_status_ == "IDLE") {
                        new_state = RobotState::IDLE;
                    } else if (navigation_status_.find("MISSION") != std::string::npos) {
                        new_state = RobotState::MISSION_EXECUTING;
                    }
                    break;
                    
                case RobotState::MISSION_EXECUTING:
                    if (mission_status_ == "COMPLETED") {
                        new_state = RobotState::IDLE;
                    } else if (mission_status_ == "FAILED") {
                        new_state = RobotState::RECOVERING;
                    }
                    break;
                    
                default:
                    // No hay transiciones automáticas para otros estados
                    break;
            }
        }
        
        // Ejecutar transición si es necesaria y válida
        if (new_state != current_state_) {
            requestStateTransition(new_state);
        }
    }
    
    bool isRecoveryComplete()
    {
        // Verificar si recovery ha terminado
        auto current_time = this->now();
        double recovery_duration = (current_time - last_transition_time_).seconds();
        
        // Recovery completo después de 10 segundos y sin emergencia
        return recovery_duration > 10.0 && !emergency_active_;
    }
    
    void requestStateTransition(RobotState new_state)
    {
        if (isValidTransition(current_state_, new_state)) {
            executeStateTransition(new_state);
        } else {
            RCLCPP_WARN(this->get_logger(), 
                        "Invalid transition from %s to %s",
                        robotStateToString(current_state_).c_str(),
                        robotStateToString(new_state).c_str());
        }
    }
    
    bool isValidTransition(RobotState from, RobotState to)
    {
        auto it = valid_transitions_.find(from);
        if (it == valid_transitions_.end()) {
            return false;
        }
        
        const auto& valid_states = it->second;
        return std::find(valid_states.begin(), valid_states.end(), to) != valid_states.end();
    }
    
    void executeStateTransition(RobotState new_state)
    {
        previous_state_ = current_state_;
        current_state_ = new_state;
        last_transition_time_ = this->now();
        
        // Log transición
        RCLCPP_INFO(this->get_logger(), 
                    "State transition: %s -> %s",
                    robotStateToString(previous_state_).c_str(),
                    robotStateToString(current_state_).c_str());
        
        // Publicar transición
        publishStateTransition();
        
        // Ejecutar acciones específicas del estado
        executeStateActions(new_state);
    }
    
    void executeStateActions(RobotState state)
    {
        switch (state) {
            case RobotState::EMERGENCY:
                RCLCPP_ERROR(this->get_logger(), "EMERGENCY STATE ACTIVATED");
                break;
                
            case RobotState::CHARGING:
                RCLCPP_INFO(this->get_logger(), "Initiating charging sequence");
                break;
                
            case RobotState::RECOVERING:
                RCLCPP_INFO(this->get_logger(), "Starting recovery procedures");
                break;
                
            case RobotState::IDLE:
                RCLCPP_INFO(this->get_logger(), "Robot returned to idle state");
                break;
                
            default:
                // No hay acciones específicas para otros estados
                break;
        }
    }
    
    void publishCurrentState()
    {
        std_msgs::msg::String state_msg;
        state_msg.data = robotStateToString(current_state_);
        state_pub_->publish(state_msg);
    }
    
    void publishStateTransition()
    {
        std_msgs::msg::String transition_msg;
        transition_msg.data = robotStateToString(previous_state_) + " -> " + 
                             robotStateToString(current_state_);
        transition_pub_->publish(transition_msg);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        // Remove component_name field - not part of SystemHealth message
        
        // Determinar estado de salud basado en el estado actual
        if (current_state_ == RobotState::EMERGENCY) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(20001);
            health_msg.error_messages.push_back("Robot in emergency state");
        } else if (current_state_ == RobotState::RECOVERING) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(20002);
            health_msg.error_messages.push_back("Robot recovering from failure");
        } else if (battery_level_ < critical_battery_level_) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(20003);
            health_msg.error_messages.push_back("Critical battery level");
        } else if (battery_level_ < low_battery_level_) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(20004);
            health_msg.error_messages.push_back("Low battery level");
        } else {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(0);
            health_msg.error_messages.push_back("");
        }
        
        // Replace with cpu_temperature = 3.0; // See SystemHealth.msg // Placeholder
        // Replace with appropriate field - memory_usage not in SystemHealth.msg // Placeholder
        // Replace with specific temperature fields: cpu_temperature, gpu_temperature, motor_temperature // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    // Callbacks
    void emergencyCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        emergency_active_ = msg->data;
        
        if (emergency_active_) {
            RCLCPP_ERROR(this->get_logger(), "Emergency signal received");
            requestStateTransition(RobotState::EMERGENCY);
        }
    }
    
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        if (msg->capacity > 0) {
            battery_level_ = msg->charge / msg->capacity;
        } else {
            battery_level_ = msg->percentage / 100.0;
        }
        
        last_battery_update_ = this->now();
    }
    
    void stateCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        
        // Parsear comando de estado
        if (command.substr(0, 12) == "TRANSITION:") {
            std::string target_state = command.substr(12);
            
            // Convertir string a RobotState
            RobotState new_state = stringToRobotState(target_state);
            if (new_state != RobotState::UNKNOWN) {
                requestStateTransition(new_state);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown state in command: %s", target_state.c_str());
            }
        }
    }
    
    void navigationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        navigation_status_ = msg->data;
    }
    
    void missionStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        mission_status_ = msg->data;
    }
    
    RobotState stringToRobotState(const std::string& state_str)
    {
        if (state_str == "IDLE") return RobotState::IDLE;
        if (state_str == "INITIALIZING") return RobotState::INITIALIZING;
        if (state_str == "NAVIGATING") return RobotState::NAVIGATING;
        if (state_str == "EXPLORING") return RobotState::EXPLORING;
        if (state_str == "PATROLLING") return RobotState::PATROLLING;
        if (state_str == "CHARGING") return RobotState::CHARGING;
        if (state_str == "EMERGENCY") return RobotState::EMERGENCY;
        if (state_str == "MAINTENANCE") return RobotState::MAINTENANCE;
        if (state_str == "MISSION_EXECUTING") return RobotState::MISSION_EXECUTING;
        if (state_str == "WAITING") return RobotState::WAITING;
        if (state_str == "RECOVERING") return RobotState::RECOVERING;
        if (state_str == "DOCKING") return RobotState::DOCKING;
        return RobotState::UNKNOWN;
    }
    
    // Variables miembro
    double update_frequency_;
    bool enable_auto_transitions_;
    double critical_battery_level_;
    double low_battery_level_;
    std::string base_frame_;
    
    // Estado actual
    RobotState current_state_;
    RobotState previous_state_;
    rclcpp::Time last_transition_time_;
    
    // Estado del sistema
    bool emergency_active_ = false;
    double battery_level_ = 1.0;
    std::string navigation_status_ = "IDLE";
    std::string mission_status_ = "IDLE";
    rclcpp::Time last_battery_update_;
    
    // Transiciones válidas
    std::map<RobotState, std::vector<RobotState>> valid_transitions_;
    
    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_status_sub_;
    
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr transition_pub_;
    rclcpp_lifecycle::LifecyclePublisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::TimerBase::SharedPtr state_timer_;
};

} // namespace tadeo_ecar_behavior

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_behavior::StateMachineNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}