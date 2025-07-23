#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/srv/execute_behavior.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include "tadeo_ecar_behavior/behavior_types.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <algorithm>
#include <filesystem>
#include <sstream>
#include <iomanip>

namespace tadeo_ecar_behavior
{

class BehaviorManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    BehaviorManagerNode() : LifecycleNode("behavior_manager_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Behavior Manager Node");
        
        // Declarar parámetros
        declareParameters();
    }

protected:
    // Lifecycle Node callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Configuring Behavior Manager Node");
        
        loadParameters();
        initializeBehaviorTree();
        createSubscribers();
        createPublishers();
        createServices();
        loadBehaviorTreeFiles();
        
        RCLCPP_INFO(this->get_logger(), "Behavior Manager Node configured");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Activating Behavior Manager Node");
        
        // Activar publicadores lifecycle
        behavior_status_pub_->on_activate();
        robot_state_pub_->on_activate();
        behavior_metrics_pub_->on_activate();
        behavior_viz_pub_->on_activate();
        health_pub_->on_activate();
        
        // Inicializar contexto del robot
        initializeRobotContext();
        
        // Crear timer principal
        behavior_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / behavior_frequency_)),
            std::bind(&BehaviorManagerNode::behaviorLoop, this));
        
        // Crear timer de salud
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&BehaviorManagerNode::publishHealthStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "Behavior Manager Node activated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating Behavior Manager Node");
        
        // Detener comportamientos activos
        stopAllBehaviors();
        
        // Desactivar publicadores
        behavior_status_pub_->on_deactivate();
        robot_state_pub_->on_deactivate();
        behavior_metrics_pub_->on_deactivate();
        behavior_viz_pub_->on_deactivate();
        health_pub_->on_deactivate();
        
        // Cancelar timers
        behavior_timer_.reset();
        health_timer_.reset();
        
        RCLCPP_INFO(this->get_logger(), "Behavior Manager Node deactivated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up Behavior Manager Node");
        
        // Limpiar recursos
        cleanupResources();
        
        RCLCPP_INFO(this->get_logger(), "Behavior Manager Node cleaned up");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    void declareParameters()
    {
        this->declare_parameter("behavior_frequency", 10.0);
        this->declare_parameter("default_tree_file", "");
        this->declare_parameter("behavior_trees_path", "/opt/ros/humble/share/tadeo_ecar_behavior/behavior_trees/");
        this->declare_parameter("enable_behavior_logging", true);
        this->declare_parameter("enable_groot_monitoring", false);
        this->declare_parameter("groot_publisher_port", 1666);
        this->declare_parameter("behavior_timeout_default", 30.0);
        this->declare_parameter("max_concurrent_behaviors", 5);
        this->declare_parameter("auto_recovery_enabled", true);
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("map_frame", "map");
    }
    
    void loadParameters()
    {
        behavior_frequency_ = this->get_parameter("behavior_frequency").as_double();
        default_tree_file_ = this->get_parameter("default_tree_file").as_string();
        behavior_trees_path_ = this->get_parameter("behavior_trees_path").as_string();
        enable_behavior_logging_ = this->get_parameter("enable_behavior_logging").as_bool();
        enable_groot_monitoring_ = this->get_parameter("enable_groot_monitoring").as_bool();
        groot_publisher_port_ = this->get_parameter("groot_publisher_port").as_int();
        behavior_timeout_default_ = this->get_parameter("behavior_timeout_default").as_double();
        max_concurrent_behaviors_ = this->get_parameter("max_concurrent_behaviors").as_int();
        auto_recovery_enabled_ = this->get_parameter("auto_recovery_enabled").as_bool();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
    }
    
    void initializeBehaviorTree()
    {
        // Crear factory de BehaviorTree
        bt_factory_ = std::make_shared<BT::BehaviorTreeFactory>();
        
        // Registrar nodos personalizados
        registerCustomNodes();
        
        // Configurar logging si está habilitado
        if (enable_behavior_logging_) {
            bt_logger_ = std::make_unique<BT::StdCoutLogger>(*current_tree_);
        }
        
        // Configurar Groot monitoring si está habilitado
        if (enable_groot_monitoring_) {
            bt_groot_publisher_ = std::make_unique<BT::PublisherZMQ>(*current_tree_, groot_publisher_port_);
        }
    }
    
    void registerCustomNodes()
    {
        // Registrar nodos de acción personalizados
        bt_factory_->registerNodeType<NavigateToGoalAction>("NavigateToGoal");
        bt_factory_->registerNodeType<CheckBatteryCondition>("CheckBattery");
        bt_factory_->registerNodeType<CheckEmergencyCondition>("CheckEmergency");
        bt_factory_->registerNodeType<ExecutePatrolAction>("ExecutePatrol");
        bt_factory_->registerNodeType<DockingAction>("DockToStation");
        bt_factory_->registerNodeType<ExplorationAction>("ExploreArea");
        bt_factory_->registerNodeType<RecoveryAction>("RecoveryBehavior");
        bt_factory_->registerNodeType<WaitAction>("Wait");
        bt_factory_->registerNodeType<SendAlertAction>("SendAlert");
        
        RCLCPP_INFO(this->get_logger(), "Custom BehaviorTree nodes registered");
    }
    
    void createSubscribers()
    {
        // Suscriptor de comandos de comportamiento
        behavior_command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "behavior_command", 10,
            std::bind(&BehaviorManagerNode::behaviorCommandCallback, this, std::placeholders::_1));
        
        // Suscriptor de odometría
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&BehaviorManagerNode::odomCallback, this, std::placeholders::_1));
        
        // Suscriptor de estado de batería
        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10,
            std::bind(&BehaviorManagerNode::batteryCallback, this, std::placeholders::_1));
        
        // Suscriptor de parada de emergencia
        emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "emergency_stop", 10,
            std::bind(&BehaviorManagerNode::emergencyStopCallback, this, std::placeholders::_1));
        
        // Suscriptor de estado de navegación
        navigation_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "navigation_status", 10,
            std::bind(&BehaviorManagerNode::navigationStatusCallback, this, std::placeholders::_1));
        
        // Suscriptor de salud del sistema
        system_health_sub_ = this->create_subscription<tadeo_ecar_msgs::msg::SystemHealth>(
            "system_health", 10,
            std::bind(&BehaviorManagerNode::systemHealthCallback, this, std::placeholders::_1));
    }
    
    void createPublishers()
    {
        // Publicador de estado de comportamientos
        behavior_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "behavior_status", 10);
        
        // Publicador de estado del robot
        robot_state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "robot_state", 10);
        
        // Publicador de métricas de comportamientos
        behavior_metrics_pub_ = this->create_publisher<std_msgs::msg::String>(
            "behavior_metrics", 10);
        
        // Publicador de visualización
        behavior_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "behavior_visualization", 10);
        
        // Publicador de salud
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "behavior/manager_health", 10);
    }
    
    void createServices()
    {
        // Servicio para ejecutar comportamientos
        execute_behavior_service_ = this->create_service<tadeo_ecar_interfaces::srv::ExecuteBehavior>(
            "execute_behavior",
            std::bind(&BehaviorManagerNode::executeBehaviorService, this, 
                     std::placeholders::_1, std::placeholders::_2));
    }
    
    void loadBehaviorTreeFiles()
    {
        try {
            // Crear directorio si no existe
            std::filesystem::create_directories(behavior_trees_path_);
            
            // Cargar árbol por defecto si está especificado
            if (!default_tree_file_.empty()) {
                std::string tree_path = behavior_trees_path_ + default_tree_file_;
                if (std::filesystem::exists(tree_path)) {
                    loadBehaviorTree(tree_path);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Default tree file not found: %s", tree_path.c_str());
                    createDefaultBehaviorTree();
                }
            } else {
                createDefaultBehaviorTree();
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load behavior trees: %s", e.what());
            createDefaultBehaviorTree();
        }
    }
    
    void createDefaultBehaviorTree()
    {
        // Crear árbol de comportamientos por defecto
        std::string default_tree_xml = R"(
        <root main_tree_to_execute="MainBehavior">
            <BehaviorTree ID="MainBehavior">
                <Sequence>
                    <CheckEmergency />
                    <Fallback>
                        <CheckBattery threshold="0.2" />
                        <DockToStation />
                    </Fallback>
                    <Fallback>
                        <NavigateToGoal />
                        <RecoveryBehavior />
                    </Fallback>
                </Sequence>
            </BehaviorTree>
        </root>
        )";
        
        try {
            current_tree_ = std::make_unique<BT::Tree>(bt_factory_->createTreeFromText(default_tree_xml));
            current_tree_name_ = "MainBehavior";
            RCLCPP_INFO(this->get_logger(), "Default behavior tree created");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create default tree: %s", e.what());
        }
    }
    
    void loadBehaviorTree(const std::string& tree_file)
    {
        try {
            current_tree_ = std::make_unique<BT::Tree>(bt_factory_->createTreeFromFile(tree_file));
            current_tree_name_ = std::filesystem::path(tree_file).stem().string();
            
            // Reinicializar loggers si están habilitados
            if (enable_behavior_logging_) {
                bt_logger_ = std::make_unique<BT::StdCoutLogger>(*current_tree_);
            }
            
            if (enable_groot_monitoring_) {
                bt_groot_publisher_ = std::make_unique<BT::PublisherZMQ>(*current_tree_, groot_publisher_port_);
            }
            
            RCLCPP_INFO(this->get_logger(), "Behavior tree loaded: %s", tree_file.c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load tree from %s: %s", tree_file.c_str(), e.what());
        }
    }
    
    void initializeRobotContext()
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        
        robot_context_.current_state = RobotState::IDLE;
        robot_context_.battery_level = 1.0;
        robot_context_.emergency_active = false;
        robot_context_.sensors_healthy = true;
        robot_context_.navigation_ready = false;
        robot_context_.last_update = this->now();
        
        // Inicializar métricas
        behavior_metrics_.session_start_time = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Robot context initialized");
    }
    
    void behaviorLoop()
    {
        // Actualizar contexto del robot
        updateRobotContext();
        
        // Procesar comandos pendientes
        processCommandQueue();
        
        // Ejecutar tick del árbol de comportamientos si está activo
        if (current_tree_ && tree_active_) {
            executeBehaviorTreeTick();
        }
        
        // Actualizar comportamientos activos
        updateActiveBehaviors();
        
        // Publicar estado
        publishBehaviorStatus();
        publishRobotState();
        publishBehaviorMetrics();
        publishBehaviorVisualization();
        
        // Verificar si necesita recovery automático
        if (auto_recovery_enabled_) {
            checkAutoRecovery();
        }
    }
    
    void executeBehaviorTreeTick()
    {
        try {
            // Actualizar blackboard con contexto actual
            updateBlackboard();
            
            // Ejecutar tick del árbol
            BT::NodeStatus tree_status = current_tree_->tickRoot();
            
            // Procesar resultado del tick
            handleTreeTickResult(tree_status);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error during behavior tree tick: %s", e.what());
            tree_active_ = false;
        }
    }
    
    void updateBlackboard()
    {
        if (!current_tree_) return;
        
        auto blackboard = current_tree_->rootBlackboard();
        
        std::lock_guard<std::mutex> lock(context_mutex_);
        
        // Actualizar datos del robot en el blackboard
        blackboard->set("robot_state", robotStateToString(robot_context_.current_state));
        blackboard->set("battery_level", robot_context_.battery_level);
        blackboard->set("emergency_active", robot_context_.emergency_active);
        blackboard->set("sensors_healthy", robot_context_.sensors_healthy);
        blackboard->set("navigation_ready", robot_context_.navigation_ready);
        
        // Pose actual
        blackboard->set("current_x", robot_context_.current_pose.pose.position.x);
        blackboard->set("current_y", robot_context_.current_pose.pose.position.y);
        
        // Velocidad actual
        blackboard->set("current_linear_vel", robot_context_.current_velocity.linear.x);
        blackboard->set("current_angular_vel", robot_context_.current_velocity.angular.z);
        
        // Timestamp
        blackboard->set("last_update", robot_context_.last_update.seconds());
    }
    
    void handleTreeTickResult(BT::NodeStatus status)
    {
        static BT::NodeStatus last_status = BT::NodeStatus::IDLE;
        
        if (status != last_status) {
            switch (status) {
                case BT::NodeStatus::RUNNING:
                    RCLCPP_DEBUG(this->get_logger(), "Behavior tree is running");
                    break;
                case BT::NodeStatus::SUCCESS:
                    RCLCPP_INFO(this->get_logger(), "Behavior tree completed successfully");
                    behavior_metrics_.successful_behaviors++;
                    behavior_metrics_.updateMetrics();
                    break;
                case BT::NodeStatus::FAILURE:
                    RCLCPP_WARN(this->get_logger(), "Behavior tree failed");
                    behavior_metrics_.failed_behaviors++;
                    behavior_metrics_.updateMetrics();
                    break;
                case BT::NodeStatus::IDLE:
                    RCLCPP_DEBUG(this->get_logger(), "Behavior tree is idle");
                    break;
            }
            last_status = status;
        }
        
        // Reiniciar árbol si terminó (éxito o fallo)
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
            // Pequeña pausa antes de reiniciar
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            current_tree_->haltTree();
        }
    }
    
    void updateActiveBehaviors()
    {
        std::lock_guard<std::mutex> lock(behaviors_mutex_);
        
        auto now = this->now();
        auto it = active_behaviors_.begin();
        
        while (it != active_behaviors_.end()) {
            auto& behavior = it->second;
            
            // Verificar timeout
            double elapsed = (now - behavior.start_time).seconds();
            if (elapsed > behavior.parameters.timeout_seconds) {
                RCLCPP_WARN(this->get_logger(), "Behavior %s timed out after %.2f seconds", 
                           behavior.behavior_id.c_str(), elapsed);
                behavior.current_status = BehaviorStatus::FAILURE;
                it = active_behaviors_.erase(it);
                continue;
            }
            
            behavior.last_update = now;
            ++it;
        }
    }
    
    void processCommandQueue()
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        
        while (!command_queue_.empty()) {
            auto command = command_queue_.front();
            command_queue_.pop();
            
            processCommand(command);
        }
    }
    
    void processCommand(const BehaviorCommand& command)
    {
        if (command.command_type == "start_tree") {
            if (!command.tree_name.empty()) {
                std::string tree_path = behavior_trees_path_ + command.tree_name + ".xml";
                loadBehaviorTree(tree_path);
            }
            tree_active_ = true;
            RCLCPP_INFO(this->get_logger(), "Started behavior tree: %s", current_tree_name_.c_str());
            
        } else if (command.command_type == "stop_tree") {
            tree_active_ = false;
            if (current_tree_) {
                current_tree_->haltTree();
            }
            RCLCPP_INFO(this->get_logger(), "Stopped behavior tree");
            
        } else if (command.command_type == "reload_tree") {
            if (current_tree_) {
                current_tree_->haltTree();
            }
            if (!default_tree_file_.empty()) {
                std::string tree_path = behavior_trees_path_ + default_tree_file_;
                loadBehaviorTree(tree_path);
            }
            RCLCPP_INFO(this->get_logger(), "Reloaded behavior tree");
            
        } else if (command.command_type == "emergency_stop") {
            handleEmergencyCommand();
            
        } else if (command.command_type == "reset") {
            resetBehaviorSystem();
            
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown command type: %s", command.command_type.c_str());
        }
    }
    
    void handleEmergencyCommand()
    {
        RCLCPP_WARN(this->get_logger(), "Emergency command received - stopping all behaviors");
        
        // Detener árbol activo
        tree_active_ = false;
        if (current_tree_) {
            current_tree_->haltTree();
        }
        
        // Cancelar comportamientos activos
        stopAllBehaviors();
        
        // Actualizar estado del robot
        {
            std::lock_guard<std::mutex> lock(context_mutex_);
            robot_context_.current_state = RobotState::EMERGENCY;
            robot_context_.emergency_active = true;
        }
    }
    
    void resetBehaviorSystem()
    {
        RCLCPP_INFO(this->get_logger(), "Resetting behavior system");
        
        // Detener todo
        tree_active_ = false;
        if (current_tree_) {
            current_tree_->haltTree();
        }
        
        stopAllBehaviors();
        
        // Reinicializar contexto
        initializeRobotContext();
        
        // Recargar árbol por defecto
        if (!default_tree_file_.empty()) {
            std::string tree_path = behavior_trees_path_ + default_tree_file_;
            loadBehaviorTree(tree_path);
        }
    }
    
    void stopAllBehaviors()
    {
        std::lock_guard<std::mutex> lock(behaviors_mutex_);
        
        for (auto& [id, behavior] : active_behaviors_) {
            behavior.current_status = BehaviorStatus::CANCELLED;
            RCLCPP_INFO(this->get_logger(), "Cancelled behavior: %s", id.c_str());
        }
        
        active_behaviors_.clear();
    }
    
    void checkAutoRecovery()
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        
        // Verificar si el robot necesita recovery automático
        if (robot_context_.current_state == RobotState::EMERGENCY && 
            !robot_context_.emergency_active &&
            auto_recovery_enabled_) {
            
            // Intentar recovery automático
            RCLCPP_INFO(this->get_logger(), "Attempting auto recovery");
            robot_context_.current_state = RobotState::RECOVERING;
            
            // Reactivar árbol de comportamientos
            tree_active_ = true;
        }
    }
    
    void updateRobotContext()
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        robot_context_.last_update = this->now();
        
        // Actualizar estado basado en condiciones
        if (!robot_context_.emergency_active && robot_context_.current_state == RobotState::EMERGENCY) {
            robot_context_.current_state = RobotState::IDLE;
        }
    }
    
    // Callbacks
    void behaviorCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        BehaviorCommand command;
        
        // Parsear comando simple
        std::string cmd_str = msg->data;
        size_t colon_pos = cmd_str.find(':');
        
        if (colon_pos != std::string::npos) {
            command.command_type = cmd_str.substr(0, colon_pos);
            command.behavior_name = cmd_str.substr(colon_pos + 1);
        } else {
            command.command_type = cmd_str;
        }
        
        command.timestamp = this->now();
        
        // Agregar a cola de comandos
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_queue_.push(command);
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        
        robot_context_.current_pose.pose = msg->pose.pose;
        robot_context_.current_pose.header = msg->header;
        robot_context_.current_velocity = msg->twist.twist;
    }
    
    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        
        if (msg->capacity > 0) {
            robot_context_.battery_level = msg->charge / msg->capacity;
        } else {
            robot_context_.battery_level = msg->percentage / 100.0;
        }
        
        // Cambiar estado si batería baja
        if (robot_context_.battery_level < 0.2 && robot_context_.current_state != RobotState::CHARGING) {
            robot_context_.current_state = RobotState::CHARGING;
        }
    }
    
    void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        
        robot_context_.emergency_active = msg->data;
        
        if (msg->data) {
            robot_context_.current_state = RobotState::EMERGENCY;
            handleEmergencyCommand();
        }
    }
    
    void navigationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        
        std::string status = msg->data;
        
        if (status.find("NAVIGATING") != std::string::npos) {
            robot_context_.navigation_ready = true;
            if (robot_context_.current_state == RobotState::IDLE) {
                robot_context_.current_state = RobotState::NAVIGATING;
            }
        } else if (status.find("IDLE") != std::string::npos) {
            if (robot_context_.current_state == RobotState::NAVIGATING) {
                robot_context_.current_state = RobotState::IDLE;
            }
        }
    }
    
    void systemHealthCallback(const tadeo_ecar_msgs::msg::SystemHealth::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        
        // Check sensor status fields instead of component_name
        robot_context_.sensors_healthy = 
            (msg->lidar_status == tadeo_ecar_msgs::msg::SystemHealth::HEALTHY) &&
            (msg->camera_status == tadeo_ecar_msgs::msg::SystemHealth::HEALTHY) &&
            (msg->imu_status == tadeo_ecar_msgs::msg::SystemHealth::HEALTHY) &&
            (msg->gps_status == tadeo_ecar_msgs::msg::SystemHealth::HEALTHY);
    }
    
    void executeBehaviorService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::ExecuteBehavior::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::ExecuteBehavior::Response> response)
    {
        try {
            BehaviorCommand command;
            command.command_type = "start_tree";
            command.tree_name = request->behavior_name;
            command.priority = BehaviorPriority::NORMAL; // Default priority
            command.timestamp = this->now();
            
            // Agregar parámetros si los hay
            for (const auto& param : request->parameters) {
                // Simple key=value parsing
                size_t eq_pos = param.find('=');
                if (eq_pos != std::string::npos) {
                    std::string key = param.substr(0, eq_pos);
                    std::string value = param.substr(eq_pos + 1);
                    command.parameters[key] = value;
                }
            }
            
            // Procesar comando
            processCommand(command);
            
            response->success = true;
            response->message = "Behavior execution started: " + request->behavior_name;
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to execute behavior: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
    
    // Publishers
    void publishBehaviorStatus()
    {
        std_msgs::msg::String status_msg;
        
        std::string status = "Tree: " + current_tree_name_ + 
                           " | Active: " + (tree_active_ ? "true" : "false") +
                           " | Behaviors: " + std::to_string(active_behaviors_.size());
        
        status_msg.data = status;
        behavior_status_pub_->publish(status_msg);
    }
    
    void publishRobotState()
    {
        std_msgs::msg::String state_msg;
        
        {
            std::lock_guard<std::mutex> lock(context_mutex_);
            state_msg.data = robotStateToString(robot_context_.current_state);
        }
        
        robot_state_pub_->publish(state_msg);
    }
    
    void publishBehaviorMetrics()
    {
        std_msgs::msg::String metrics_msg;
        
        std::ostringstream oss;
        oss << "Executed: " << behavior_metrics_.total_behaviors_executed
            << " | Success: " << behavior_metrics_.successful_behaviors
            << " | Failed: " << behavior_metrics_.failed_behaviors
            << " | Success Rate: " << std::fixed << std::setprecision(1) 
            << (behavior_metrics_.success_rate * 100.0) << "%";
        
        metrics_msg.data = oss.str();
        behavior_metrics_pub_->publish(metrics_msg);
    }
    
    void publishBehaviorVisualization()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Crear marcador de estado del robot
        visualization_msgs::msg::Marker state_marker;
        state_marker.header.frame_id = base_frame_;
        state_marker.header.stamp = this->now();
        state_marker.ns = "behavior_manager";
        state_marker.id = 0;
        state_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        state_marker.action = visualization_msgs::msg::Marker::ADD;
        
        state_marker.pose.position.x = 0.0;
        state_marker.pose.position.y = 0.0;
        state_marker.pose.position.z = 1.5;
        state_marker.pose.orientation.w = 1.0;
        
        {
            std::lock_guard<std::mutex> lock(context_mutex_);
            state_marker.text = "State: " + robotStateToString(robot_context_.current_state);
        }
        
        state_marker.scale.z = 0.3;
        state_marker.color.r = 1.0;
        state_marker.color.g = 1.0;
        state_marker.color.b = 1.0;
        state_marker.color.a = 1.0;
        
        marker_array.markers.push_back(state_marker);
        
        behavior_viz_pub_->publish(marker_array);
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        // Remove component_name field - not part of SystemHealth message
        
        {
            std::lock_guard<std::mutex> lock(context_mutex_);
            
            if (robot_context_.emergency_active) {
                health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::CRITICAL;
                health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::CRITICAL;
                health_msg.error_codes.push_back(19001);
                health_msg.error_messages.push_back("Emergency state active");
            } else if (!tree_active_) {
                health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::WARNING;
                health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
                health_msg.error_codes.push_back(19002);
                health_msg.error_messages.push_back("Behavior tree not active");
            } else if (robot_context_.current_state == RobotState::UNKNOWN) {
                health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
                health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::ERROR;
                health_msg.error_codes.push_back(19003);
                health_msg.error_messages.push_back("Unknown robot state");
            } else {
                health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
                health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
            }
        }
        
        // Set proper temperature fields
        health_msg.cpu_temperature = 38.0; // Placeholder
        health_msg.gpu_temperature = 35.0; // Placeholder
        health_msg.motor_temperature = 32.0; // Placeholder
        
        // Set other status fields
        health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.lidar_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.camera_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.imu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.gps_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set motor status fields
        health_msg.front_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.front_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_left_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.rear_right_motor_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set diagnostic info and uptime
        health_msg.diagnostic_info = "Behavior manager running";
        health_msg.uptime_seconds = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        
        health_pub_->publish(health_msg);
    }
    
    void cleanupResources()
    {
        // Limpiar recursos
        if (current_tree_) {
            current_tree_->haltTree();
            current_tree_.reset();
        }
        
        bt_logger_.reset();
        bt_groot_publisher_.reset();
        bt_factory_.reset();
        
        active_behaviors_.clear();
        
        while (!command_queue_.empty()) {
            command_queue_.pop();
        }
    }
    
    // Custom BehaviorTree Action Nodes
    class NavigateToGoalAction : public BT::SyncActionNode
    {
    public:
        NavigateToGoalAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) {}
        
        static BT::PortsList providedPorts() {
            return {
                BT::InputPort<double>("goal_x"),
                BT::InputPort<double>("goal_y"),
                BT::InputPort<double>("goal_theta")
            };
        }
        
        BT::NodeStatus tick() override {
            // Implementación simplificada
            return BT::NodeStatus::SUCCESS;
        }
    };
    
    class CheckBatteryCondition : public BT::ConditionNode
    {
    public:
        CheckBatteryCondition(const std::string& name, const BT::NodeConfiguration& config)
            : BT::ConditionNode(name, config) {}
        
        static BT::PortsList providedPorts() {
            return { BT::InputPort<double>("threshold") };
        }
        
        BT::NodeStatus tick() override {
            double threshold = 0.2;
            getInput("threshold", threshold);
            
            double battery_level;
            if (getInput("battery_level", battery_level)) {
                return battery_level > threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::FAILURE;
        }
    };
    
    class CheckEmergencyCondition : public BT::ConditionNode
    {
    public:
        CheckEmergencyCondition(const std::string& name, const BT::NodeConfiguration& config)
            : BT::ConditionNode(name, config) {}
        
        static BT::PortsList providedPorts() {
            return {};
        }
        
        BT::NodeStatus tick() override {
            bool emergency_active;
            if (getInput("emergency_active", emergency_active)) {
                return emergency_active ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::SUCCESS;
        }
    };
    
    // Placeholder action classes para otros nodos
    class ExecutePatrolAction : public BT::SyncActionNode
    {
    public:
        ExecutePatrolAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) {}
        
        static BT::PortsList providedPorts() {
            return {};
        }
        
        BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    };
    
    class DockingAction : public BT::SyncActionNode
    {
    public:
        DockingAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) {}
        
        static BT::PortsList providedPorts() {
            return {};
        }
        
        BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    };
    
    class ExplorationAction : public BT::SyncActionNode
    {
    public:
        ExplorationAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) {}
        
        static BT::PortsList providedPorts() {
            return {};
        }
        
        BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    };
    
    class RecoveryAction : public BT::SyncActionNode
    {
    public:
        RecoveryAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) {}
        
        static BT::PortsList providedPorts() {
            return {};
        }
        
        BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    };
    
    class WaitAction : public BT::SyncActionNode
    {
    public:
        WaitAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) {}
        
        static BT::PortsList providedPorts() {
            return {};
        }
        
        BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    };
    
    class SendAlertAction : public BT::SyncActionNode
    {
    public:
        SendAlertAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) {}
        
        static BT::PortsList providedPorts() {
            return {};
        }
        
        BT::NodeStatus tick() override { return BT::NodeStatus::SUCCESS; }
    };
    
    // Variables miembro
    // Parámetros
    double behavior_frequency_;
    std::string default_tree_file_;
    std::string behavior_trees_path_;
    bool enable_behavior_logging_;
    bool enable_groot_monitoring_;
    int groot_publisher_port_;
    double behavior_timeout_default_;
    int max_concurrent_behaviors_;
    bool auto_recovery_enabled_;
    std::string base_frame_;
    std::string map_frame_;
    
    // BehaviorTree components
    std::shared_ptr<BT::BehaviorTreeFactory> bt_factory_;
    std::unique_ptr<BT::Tree> current_tree_;
    std::unique_ptr<BT::StdCoutLogger> bt_logger_;
    std::unique_ptr<BT::PublisherZMQ> bt_groot_publisher_;
    std::string current_tree_name_;
    bool tree_active_ = false;
    
    // Estado del sistema
    RobotContext robot_context_;
    BehaviorMetrics behavior_metrics_;
    std::map<std::string, ActiveBehavior> active_behaviors_;
    std::queue<BehaviorCommand> command_queue_;
    
    // Mutexes para thread safety
    std::mutex context_mutex_;
    std::mutex behaviors_mutex_;
    std::mutex command_mutex_;
    
    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr behavior_command_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_sub_;
    rclcpp::Subscription<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr system_health_sub_;
    
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr behavior_status_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr behavior_metrics_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr behavior_viz_pub_;
    rclcpp_lifecycle::LifecyclePublisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::Service<tadeo_ecar_interfaces::srv::ExecuteBehavior>::SharedPtr execute_behavior_service_;
    
    rclcpp::TimerBase::SharedPtr behavior_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
};

} // namespace tadeo_ecar_behavior

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_behavior::BehaviorManagerNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}