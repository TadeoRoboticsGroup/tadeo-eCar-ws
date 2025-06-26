# Capítulo 12: Behavior Trees en ROS2

## Tabla de Contenidos

1. [Introducción a Behavior Trees](#introducción-a-behavior-trees)
2. [Conceptos Fundamentales](#conceptos-fundamentales)
3. [BehaviorTree.CPP](#behaviortreecpp)
4. [Nodos Básicos](#nodos-básicos)
5. [Nodos Personalizados](#nodos-personalizados)
6. [Behavior Trees para eCar](#behavior-trees-para-ecar)
7. [Integración con Nav2](#integración-con-nav2)
8. [Debugging y Visualización](#debugging-y-visualización)

## Introducción a Behavior Trees

### ¿Qué son los Behavior Trees?

Los Behavior Trees (BT) son una metodología para estructurar la lógica de comportamiento de robots y sistemas autónomos de manera modular, reutilizable y fácil de entender.

```
Behavior Tree = Estructura Jerárquica de Decisiones

Ventajas sobre FSM (Finite State Machines):
✓ Modularidad y reutilización
✓ Composición fácil de comportamientos
✓ Debugging visual intuitivo
✓ Extensibilidad sin modificar código existente
✓ Manejo natural de concurrencia
```

### Comparación: FSM vs Behavior Trees

**Finite State Machine (FSM)**
```
Estados: [Idle, Moving, Charging, Error]

Problemas:
- Crecimiento exponencial de transiciones
- Difícil reutilización
- Comportamientos anidados complejos
- Debugging complicado
```

**Behavior Tree**
```
Estructura de árbol con nodos reutilizables

Ventajas:
- Composición modular
- Subtrees reutilizables
- Visualización clara
- Fácil mantenimiento
```

### Aplicaciones en el eCar

**1. Navegación Inteligente**
```xml
<Sequence name="NavigateToGoal">
  <Condition name="IsBatteryOK"/>
  <Condition name="IsPathClear"/>
  <Action name="ComputePath"/>
  <Action name="FollowPath"/>
</Sequence>
```

**2. Comportamientos de Seguridad**
```xml
<Fallback name="SafetyBehavior">
  <Condition name="IsEmergencyStop"/>
  <Action name="EmergencyBrake"/>
  <Action name="NormalOperation"/>
</Fallback>
```

**3. Gestión de Tareas**
```xml
<Parallel name="MultitaskingBehavior">
  <Action name="NavigateToWaypoint"/>
  <Action name="MonitorBattery"/>
  <Action name="UpdateMap"/>
</Parallel>
```

## Conceptos Fundamentales

### Estructura de un Behavior Tree

```
                    Root
                     |
              Control Node
               /           \
        Action Node    Condition Node
```

### Estados de Nodos

Todo nodo en un BT retorna uno de estos estados:

```cpp
enum class NodeStatus {
    SUCCESS,    // ✓ Tarea completada exitosamente
    FAILURE,    // ✗ Tarea falló
    RUNNING     // ⏳ Tarea en progreso
};
```

### Tipos de Nodos

**1. Control Nodes (Nodos de Control)**
- Secuence: Ejecuta hijos en orden hasta que uno falle
- Fallback: Ejecuta hijos hasta que uno tenga éxito
- Parallel: Ejecuta múltiples hijos simultáneamente

**2. Decorator Nodes (Nodos Decoradores)**
- Inverter: Invierte el resultado del hijo
- Retry: Reintenta el hijo N veces
- Timeout: Limita tiempo de ejecución

**3. Leaf Nodes (Nodos Hoja)**
- Action: Ejecuta una acción específica
- Condition: Verifica una condición

### Flujo de Ejecución

```
1. Tick → Nodo recibe señal de ejecución
2. Procesamiento → Nodo ejecuta su lógica
3. Return Status → Nodo retorna SUCCESS/FAILURE/RUNNING
4. Parent Decision → Nodo padre decide qué hacer
```

## BehaviorTree.CPP

### Instalación y Configuración

```bash
# Instalar BehaviorTree.CPP v3 (compatible con ROS2)
sudo apt install ros-humble-behaviortree-cpp-v3

# O compilar desde fuente
cd ~/ros2_ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
git checkout v3.8  # Versión estable
cd ~/ros2_ws
colcon build --packages-select behaviortree_cpp_v3
```

### Estructura Básica de un BT

```cpp
// include/tadeo_ecar_behavior/basic_bt_example.hpp
#ifndef TADEO_ECAR_BEHAVIOR__BASIC_BT_EXAMPLE_HPP_
#define TADEO_ECAR_BEHAVIOR__BASIC_BT_EXAMPLE_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace tadeo_ecar_behavior
{
class BasicBTExample : public rclcpp::Node
{
public:
  BasicBTExample();
  
private:
  void executeBehaviorTree();
  void registerNodes();
  
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  rclcpp::TimerInterface::SharedPtr timer_;
};
}  // namespace tadeo_ecar_behavior

#endif  // TADEO_ECAR_BEHAVIOR__BASIC_BT_EXAMPLE_HPP_
```

```cpp
// src/basic_bt_example.cpp
#include "tadeo_ecar_behavior/basic_bt_example.hpp"
#include <fstream>

namespace tadeo_ecar_behavior
{
BasicBTExample::BasicBTExample() : Node("basic_bt_example")
{
  // Registrar nodos personalizados
  registerNodes();
  
  // Crear BT desde XML
  std::string bt_xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence name="RobotBehavior">
          <Action ID="CheckBattery"/>
          <Action ID="StartMotors"/>
          <Fallback name="NavigationFallback">
            <Action ID="NavigateToGoal"/>
            <Action ID="FindAlternatePath"/>
          </Fallback>
        </Sequence>
      </BehaviorTree>
    </root>
  )";
  
  try {
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(bt_xml));
    
    // Timer para ejecutar BT periódicamente
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&BasicBTExample::executeBehaviorTree, this));
    
    RCLCPP_INFO(this->get_logger(), "Behavior Tree initialized successfully");
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create BT: %s", e.what());
  }
}

void BasicBTExample::registerNodes()
{
  // Registrar acciones personalizadas
  factory_.registerSimpleAction("CheckBattery", 
    [this](BT::TreeNode& node) {
      RCLCPP_INFO(this->get_logger(), "Checking battery level...");
      // Simular verificación de batería
      return BT::NodeStatus::SUCCESS;
    });
  
  factory_.registerSimpleAction("StartMotors",
    [this](BT::TreeNode& node) {
      RCLCPP_INFO(this->get_logger(), "Starting motors...");
      // Simular inicio de motores
      return BT::NodeStatus::SUCCESS;
    });
  
  factory_.registerSimpleAction("NavigateToGoal",
    [this](BT::TreeNode& node) {
      RCLCPP_INFO(this->get_logger(), "Navigating to goal...");
      // Simular navegación
      static int attempt = 0;
      attempt++;
      
      if (attempt < 3) {
        return BT::NodeStatus::RUNNING;
      } else {
        attempt = 0;
        return BT::NodeStatus::SUCCESS;
      }
    });
  
  factory_.registerSimpleAction("FindAlternatePath",
    [this](BT::TreeNode& node) {
      RCLCPP_INFO(this->get_logger(), "Finding alternate path...");
      return BT::NodeStatus::SUCCESS;
    });
}

void BasicBTExample::executeBehaviorTree()
{
  if (tree_) {
    BT::NodeStatus status = tree_->tickRoot();
    
    switch (status) {
      case BT::NodeStatus::SUCCESS:
        RCLCPP_INFO(this->get_logger(), "Behavior Tree completed successfully");
        timer_->cancel();  // Detener ejecución
        break;
        
      case BT::NodeStatus::FAILURE:
        RCLCPP_ERROR(this->get_logger(), "Behavior Tree failed");
        timer_->cancel();
        break;
        
      case BT::NodeStatus::RUNNING:
        // Continuar ejecución
        break;
    }
  }
}
}  // namespace tadeo_ecar_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tadeo_ecar_behavior::BasicBTExample>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## Nodos Básicos

### Control Nodes

**1. Sequence Node**
```cpp
// Ejecuta hijos en orden. Si uno falla, toda la secuencia falla
<Sequence name="StartupSequence">
  <Action ID="InitializeSensors"/>
  <Action ID="CalibrateWheels"/>
  <Action ID="StartNavigation"/>
</Sequence>
```

**2. Fallback Node**
```cpp
// Ejecuta hijos hasta que uno tenga éxito
<Fallback name="BatteryManagement">
  <Condition ID="IsBatteryFull"/>
  <Action ID="FindChargingStation"/>
  <Action ID="EmergencyShutdown"/>
</Fallback>
```

**3. Parallel Node**
```cpp
// Ejecuta múltiples hijos simultáneamente
<Parallel success_threshold="2" failure_threshold="1">
  <Action ID="Navigate"/>
  <Action ID="MonitorSensors"/>
  <Action ID="UpdateMap"/>
</Parallel>
```

### Decorator Nodes

**1. Inverter**
```cpp
// Invierte el resultado del hijo
<Inverter>
  <Condition ID="IsObstacleAhead"/>
</Inverter>
```

**2. Retry**
```cpp
// Reintenta N veces
<RetryUntilSuccessful num_attempts="3">
  <Action ID="ConnectToWiFi"/>
</RetryUntilSuccessful>
```

**3. Timeout**
```cpp
// Limita tiempo de ejecución
<Timeout msec="5000">
  <Action ID="NavigateToGoal"/>
</Timeout>
```

### Blackboard (Pizarra)

El Blackboard es la memoria compartida entre nodos:

```cpp
// Ejemplo de uso del Blackboard
class SetGoalAction : public BT::SyncActionNode
{
public:
  SetGoalAction(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::Pose>("goal_pose")
    };
  }
  
  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::Pose goal;
    goal.position.x = 5.0;
    goal.position.y = 3.0;
    
    // Escribir al blackboard
    setOutput("goal_pose", goal);
    
    return BT::NodeStatus::SUCCESS;
  }
};

class NavigateAction : public BT::StatefulActionNode
{
public:
  NavigateAction(const std::string& name, const BT::NodeConfiguration& config)
    : StatefulActionNode(name, config) {}
  
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Pose>("goal_pose")
    };
  }
  
  BT::NodeStatus onStart() override
  {
    // Leer del blackboard
    auto goal = getInput<geometry_msgs::msg::Pose>("goal_pose");
    if (!goal) {
      return BT::NodeStatus::FAILURE;
    }
    
    goal_pose_ = goal.value();
    return BT::NodeStatus::RUNNING;
  }
  
  BT::NodeStatus onRunning() override
  {
    // Lógica de navegación
    return BT::NodeStatus::SUCCESS;
  }
  
  void onHalted() override
  {
    // Cleanup
  }
  
private:
  geometry_msgs::msg::Pose goal_pose_;
};
```

## Nodos Personalizados

### Nodos Específicos para eCar

```cpp
// include/tadeo_ecar_behavior/ecar_bt_nodes.hpp
#ifndef TADEO_ECAR_BEHAVIOR__ECAR_BT_NODES_HPP_
#define TADEO_ECAR_BEHAVIOR__ECAR_BT_NODES_HPP_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

namespace tadeo_ecar_behavior
{

// Condition: Verificar nivel de batería
class IsBatteryOK : public BT::ConditionNode
{
public:
  IsBatteryOK(const std::string& name, const BT::NodeConfiguration& config);
  
  BT::NodeStatus tick() override;
  
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery_percentage", 20.0, "Minimum battery level")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  sensor_msgs::msg::BatteryState::SharedPtr last_battery_state_;
};

// Action: Activar modo 4WS (Four Wheel Steering)
class Enable4WS : public BT::SyncActionNode
{
public:
  Enable4WS(const std::string& name, const BT::NodeConfiguration& config);
  
  BT::NodeStatus tick() override;
  
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("enable", true, "Enable or disable 4WS mode")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_4ws_pub_;
};

// Action: Movimiento lateral específico del eCar
class LateralMove : public BT::StatefulActionNode
{
public:
  LateralMove(const std::string& name, const BT::NodeConfiguration& config);
  
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", "Lateral distance to move (meters)"),
      BT::InputPort<double>("velocity", 0.5, "Lateral velocity (m/s)"),
      BT::InputPort<double>("timeout", 10.0, "Timeout in seconds")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Time start_time_;
  double target_distance_;
  double lateral_velocity_;
  double timeout_;
  double moved_distance_;
};

// Condition: Verificar si las ruedas están calibradas
class AreWheelsCalibrated : public BT::ConditionNode
{
public:
  AreWheelsCalibrated(const std::string& name, const BT::NodeConfiguration& config);
  
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  // Aquí iría la lógica específica para verificar calibración
};

// Action: Secuencia de emergencia específica del eCar
class EmergencyStop : public BT::SyncActionNode
{
public:
  EmergencyStop(const std::string& name, const BT::NodeConfiguration& config);
  
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
};

}  // namespace tadeo_ecar_behavior

#endif  // TADEO_ECAR_BEHAVIOR__ECAR_BT_NODES_HPP_
```

```cpp
// src/ecar_bt_nodes.cpp
#include "tadeo_ecar_behavior/ecar_bt_nodes.hpp"

namespace tadeo_ecar_behavior
{

// IsBatteryOK Implementation
IsBatteryOK::IsBatteryOK(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("is_battery_ok_bt_node");
  
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      last_battery_state_ = msg;
    });
}

BT::NodeStatus IsBatteryOK::tick()
{
  double min_battery_percentage;
  if (!getInput("min_battery_percentage", min_battery_percentage)) {
    return BT::NodeStatus::FAILURE;
  }
  
  if (!last_battery_state_) {
    RCLCPP_WARN(node_->get_logger(), "No battery state received yet");
    return BT::NodeStatus::FAILURE;
  }
  
  if (last_battery_state_->percentage >= min_battery_percentage) {
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), 
                "Battery low: %.1f%% < %.1f%%",
                last_battery_state_->percentage, min_battery_percentage);
    return BT::NodeStatus::FAILURE;
  }
}

// Enable4WS Implementation
Enable4WS::Enable4WS(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("enable_4ws_bt_node");
  
  enable_4ws_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "enable_4ws", 10);
}

BT::NodeStatus Enable4WS::tick()
{
  bool enable = true;
  getInput("enable", enable);
  
  auto msg = std_msgs::msg::Bool();
  msg.data = enable;
  enable_4ws_pub_->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), 
              "4WS mode %s", enable ? "enabled" : "disabled");
  
  return BT::NodeStatus::SUCCESS;
}

// LateralMove Implementation
LateralMove::LateralMove(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("lateral_move_bt_node");
  
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", 10);
}

BT::NodeStatus LateralMove::onStart()
{
  if (!getInput("distance", target_distance_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [distance]");
    return BT::NodeStatus::FAILURE;
  }
  
  getInput("velocity", lateral_velocity_);
  getInput("timeout", timeout_);
  
  start_time_ = node_->get_clock()->now();
  moved_distance_ = 0.0;
  
  RCLCPP_INFO(node_->get_logger(), 
              "Starting lateral move: %.2fm at %.2fm/s",
              target_distance_, lateral_velocity_);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LateralMove::onRunning()
{
  rclcpp::Time current_time = node_->get_clock()->now();
  double elapsed = (current_time - start_time_).seconds();
  
  // Check timeout
  if (elapsed > timeout_) {
    RCLCPP_WARN(node_->get_logger(), "Lateral move timed out");
    return BT::NodeStatus::FAILURE;
  }
  
  // Calculate how far we should have moved
  double expected_distance = lateral_velocity_ * elapsed;
  
  // Check if we've reached the target
  if (std::abs(expected_distance) >= std::abs(target_distance_)) {
    // Stop movement
    auto stop_cmd = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(stop_cmd);
    
    RCLCPP_INFO(node_->get_logger(), "Lateral move completed");
    return BT::NodeStatus::SUCCESS;
  }
  
  // Continue lateral movement
  auto cmd_vel = geometry_msgs::msg::Twist();
  cmd_vel.linear.y = (target_distance_ > 0) ? lateral_velocity_ : -lateral_velocity_;
  cmd_vel_pub_->publish(cmd_vel);
  
  return BT::NodeStatus::RUNNING;
}

void LateralMove::onHalted()
{
  // Stop robot
  auto stop_cmd = geometry_msgs::msg::Twist();
  cmd_vel_pub_->publish(stop_cmd);
  
  RCLCPP_INFO(node_->get_logger(), "Lateral move halted");
}

// AreWheelsCalibrated Implementation
AreWheelsCalibrated::AreWheelsCalibrated(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("are_wheels_calibrated_bt_node");
}

BT::NodeStatus AreWheelsCalibrated::tick()
{
  // En una implementación real, esto verificaría el estado de calibración
  // Por ahora, simulamos que siempre están calibradas
  RCLCPP_DEBUG(node_->get_logger(), "Checking wheel calibration status");
  return BT::NodeStatus::SUCCESS;
}

// EmergencyStop Implementation
EmergencyStop::EmergencyStop(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("emergency_stop_bt_node");
  
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", 10);
  emergency_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "emergency_stop", 10);
}

BT::NodeStatus EmergencyStop::tick()
{
  // Stop all movement immediately
  auto stop_cmd = geometry_msgs::msg::Twist();
  cmd_vel_pub_->publish(stop_cmd);
  
  // Signal emergency stop
  auto emergency_msg = std_msgs::msg::Bool();
  emergency_msg.data = true;
  emergency_pub_->publish(emergency_msg);
  
  RCLCPP_ERROR(node_->get_logger(), "EMERGENCY STOP ACTIVATED");
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace tadeo_ecar_behavior
```

### Registro de Nodos Personalizados

```cpp
// src/ecar_bt_factory.cpp
#include "tadeo_ecar_behavior/ecar_bt_nodes.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

namespace tadeo_ecar_behavior
{
void registerECarBTNodes(BT::BehaviorTreeFactory& factory)
{
  // Register condition nodes
  factory.registerNodeType<IsBatteryOK>("IsBatteryOK");
  factory.registerNodeType<AreWheelsCalibrated>("AreWheelsCalibrated");
  
  // Register action nodes
  factory.registerNodeType<Enable4WS>("Enable4WS");
  factory.registerNodeType<LateralMove>("LateralMove");
  factory.registerNodeType<EmergencyStop>("EmergencyStop");
  
  // Agregar más nodos según sea necesario
}
}  // namespace tadeo_ecar_behavior
```

## Behavior Trees para eCar

### BT Principal del eCar

```xml
<!-- behavior_trees/ecar_main_behavior.xml -->
<root main_tree_to_execute="ECarMainBehavior">
  
  <BehaviorTree ID="ECarMainBehavior">
    <Sequence name="ECarStartup">
      
      <!-- Pre-flight checks -->
      <SubTree ID="PreFlightChecks"/>
      
      <!-- Main operation loop -->
      <ReactiveSequence name="MainOperationLoop">
        
        <!-- Safety monitoring (always active) -->
        <Parallel success_threshold="1" failure_threshold="1">
          
          <!-- Safety monitor -->
          <SubTree ID="SafetyMonitor"/>
          
          <!-- Main behavior selection -->
          <Fallback name="BehaviorSelection">
            
            <!-- Emergency situations -->
            <SubTree ID="EmergencyBehavior"/>
            
            <!-- Battery management -->
            <SubTree ID="BatteryManagement"/>
            
            <!-- Mission execution -->
            <SubTree ID="MissionExecution"/>
            
            <!-- Idle behavior -->
            <SubTree ID="IdleBehavior"/>
            
          </Fallback>
          
        </Parallel>
        
      </ReactiveSequence>
      
    </Sequence>
  </BehaviorTree>
  
  <!-- Subtrees -->
  
  <BehaviorTree ID="PreFlightChecks">
    <Sequence name="SystemChecks">
      <Action ID="IsBatteryOK" min_battery_percentage="30.0"/>
      <Action ID="AreWheelsCalibrated"/>
      <Condition ID="IsLidarActive"/>
      <Condition ID="IsIMUActive"/>
      <Action ID="InitializeNavigation"/>
    </Sequence>
  </BehaviorTree>
  
  <BehaviorTree ID="SafetyMonitor">
    <ReactiveSequence name="SafetyChecks">
      
      <!-- Critical safety conditions -->
      <Inverter>
        <Condition ID="IsEmergencyStopPressed"/>
      </Inverter>
      
      <Condition ID="IsBatteryOK" min_battery_percentage="5.0"/>
      
      <Condition ID="IsLidarOperational"/>
      
      <!-- If any safety condition fails, trigger emergency -->
      <Action ID="EmergencyStop"/>
      
    </ReactiveSequence>
  </BehaviorTree>
  
  <BehaviorTree ID="EmergencyBehavior">
    <Sequence name="EmergencyResponse">
      
      <!-- Check if in emergency state -->
      <Condition ID="IsEmergencyActive"/>
      
      <!-- Emergency actions -->
      <Parallel success_threshold="3" failure_threshold="1">
        <Action ID="EmergencyStop"/>
        <Action ID="SendEmergencyAlert"/>
        <Action ID="LogEmergencyEvent"/>
      </Parallel>
      
      <!-- Wait for manual reset -->
      <Action ID="WaitForManualReset"/>
      
    </Sequence>
  </BehaviorTree>
  
  <BehaviorTree ID="BatteryManagement">
    <Sequence name="BatteryCheck">
      
      <!-- Check if battery is low -->
      <Inverter>
        <Condition ID="IsBatteryOK" min_battery_percentage="25.0"/>
      </Inverter>
      
      <!-- Battery management actions -->
      <Fallback name="BatteryActions">
        
        <!-- Try to find charging station -->
        <Sequence name="GoToCharging">
          <Action ID="FindChargingStation"/>
          <Action ID="NavigateToChargingStation"/>
          <Action ID="DockToCharger"/>
        </Sequence>
        
        <!-- If can't find charger, return to base -->
        <Sequence name="ReturnToBase">
          <Action ID="NavigateToBase"/>
          <Action ID="LowPowerMode"/>
        </Sequence>
        
        <!-- Last resort: emergency shutdown -->
        <Action ID="EmergencyShutdown"/>
        
      </Fallback>
      
    </Sequence>
  </BehaviorTree>
  
  <BehaviorTree ID="MissionExecution">
    <Sequence name="ExecuteMission">
      
      <!-- Check if there's an active mission -->
      <Condition ID="HasActiveMission"/>
      
      <!-- Mission type selection -->
      <Fallback name="MissionTypeSelector">
        
        <!-- Navigation mission -->
        <Sequence name="NavigationMission">
          <Condition ID="IsNavigationMission"/>
          <SubTree ID="NavigationBehavior"/>
        </Sequence>
        
        <!-- Patrol mission -->
        <Sequence name="PatrolMission">
          <Condition ID="IsPatrolMission"/>
          <SubTree ID="PatrolBehavior"/>
        </Sequence>
        
        <!-- Mapping mission -->
        <Sequence name="MappingMission">
          <Condition ID="IsMappingMission"/>
          <SubTree ID="MappingBehavior"/>
        </Sequence>
        
        <!-- Custom mission -->
        <SubTree ID="CustomMissionBehavior"/>
        
      </Fallback>
      
    </Sequence>
  </BehaviorTree>
  
  <BehaviorTree ID="NavigationBehavior">
    <Sequence name="Navigate">
      
      <!-- Get navigation goal -->
      <Action ID="GetNavigationGoal"/>
      
      <!-- Enhanced navigation with 4WD4WS capabilities -->
      <Fallback name="NavigationStrategy">
        
        <!-- Standard navigation -->
        <Sequence name="StandardNavigation">
          <Action ID="ComputePath"/>
          <Action ID="FollowPath"/>
        </Sequence>
        
        <!-- 4WS enhanced navigation for tight spaces -->
        <Sequence name="EnhancedNavigation">
          <Action ID="Enable4WS" enable="true"/>
          <Action ID="ComputeOmnidirectionalPath"/>
          <Action ID="FollowOmnidirectionalPath"/>
          <Action ID="Enable4WS" enable="false"/>
        </Sequence>
        
        <!-- Recovery behaviors -->
        <SubTree ID="NavigationRecovery"/>
        
      </Fallback>
      
    </Sequence>
  </BehaviorTree>
  
  <BehaviorTree ID="NavigationRecovery">
    <Sequence name="RecoveryActions">
      
      <!-- Clear costmaps -->
      <Action ID="ClearCostmaps"/>
      
      <!-- Try lateral movement (4WS advantage) -->
      <Action ID="LateralMove" distance="0.5" velocity="0.2"/>
      
      <!-- Rotate to gain new perspective -->
      <Action ID="RotateRobot" angle="1.57"/>
      
      <!-- Back up -->
      <Action ID="BackUp" distance="0.3"/>
      
      <!-- Last resort: request human intervention -->
      <Action ID="RequestHumanIntervention"/>
      
    </Sequence>
  </BehaviorTree>
  
  <BehaviorTree ID="PatrolBehavior">
    <Sequence name="Patrol">
      
      <Action ID="GetPatrolWaypoints"/>
      
      <ForEach>
        <Sequence name="VisitWaypoint">
          <Action ID="NavigateToWaypoint"/>
          <Action ID="PerformInspection"/>
          <Action ID="UpdatePatrolLog"/>
        </Sequence>
      </ForEach>
      
    </Sequence>
  </BehaviorTree>
  
  <BehaviorTree ID="MappingBehavior">
    <Parallel success_threshold="2" failure_threshold="1">
      
      <!-- SLAM mapping -->
      <Action ID="StartSLAM"/>
      
      <!-- Exploration behavior -->
      <Sequence name="Exploration">
        <Action ID="FindUnexploredArea"/>
        <Action ID="NavigateToUnexplored"/>
        <Action ID="MapArea"/>
      </Sequence>
      
      <!-- Map quality monitoring -->
      <Action ID="MonitorMapQuality"/>
      
    </Parallel>
  </BehaviorTree>
  
  <BehaviorTree ID="IdleBehavior">
    <Sequence name="IdleMode">
      
      <!-- System maintenance while idle -->
      <Parallel success_threshold="1" failure_threshold="0">
        <Action ID="MonitorSystems"/>
        <Action ID="OptimizePosition"/>
        <Action ID="UpdateFirmware"/>
      </Parallel>
      
      <!-- Wait for new mission -->
      <Action ID="WaitForMission"/>
      
    </Sequence>
  </BehaviorTree>
  
</root>
```

### Configuración de Comportamientos

```yaml
# config/behavior_tree_config.yaml
behavior_tree:
  ros__parameters:
    # BT execution parameters
    bt_file_path: "behavior_trees/ecar_main_behavior.xml"
    default_server_timeout: 20
    bt_loop_duration: 10
    
    # Node-specific parameters
    battery_monitoring:
      critical_level: 10.0
      low_level: 25.0
      normal_level: 80.0
      check_interval: 5.0
      
    navigation:
      goal_tolerance: 0.2
      path_tolerance: 0.5
      max_retries: 3
      use_4ws_for_tight_spaces: true
      lateral_move_threshold: 0.3
      
    safety:
      emergency_stop_timeout: 1.0
      safety_check_interval: 0.5
      lidar_timeout: 2.0
      imu_timeout: 1.0
      
    mapping:
      exploration_radius: 2.0
      min_map_quality: 0.7
      save_map_interval: 300.0  # 5 minutes
      
    patrol:
      waypoint_tolerance: 0.3
      inspection_duration: 10.0
      patrol_speed: 0.8
```

## Integración con Nav2

### BT Navigator Personalizado

```cpp
// include/tadeo_ecar_behavior/ecar_bt_navigator.hpp
#ifndef TADEO_ECAR_BEHAVIOR__ECAR_BT_NAVIGATOR_HPP_
#define TADEO_ECAR_BEHAVIOR__ECAR_BT_NAVIGATOR_HPP_

#include <nav2_bt_navigator/bt_navigator.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace tadeo_ecar_behavior
{
class ECarBTNavigator : public nav2_bt_navigator::BtNavigator
{
public:
  ECarBTNavigator();

protected:
  void configure() override;
  void registerCustomNodes();
  
private:
  void loadECarBehaviorTree();
  void setupECarSpecificParameters();
};
}  // namespace tadeo_ecar_behavior

#endif  // TADEO_ECAR_BEHAVIOR__ECAR_BT_NAVIGATOR_HPP_
```

```cpp
// src/ecar_bt_navigator.cpp
#include "tadeo_ecar_behavior/ecar_bt_navigator.hpp"
#include "tadeo_ecar_behavior/ecar_bt_nodes.hpp"

namespace tadeo_ecar_behavior
{
ECarBTNavigator::ECarBTNavigator() : nav2_bt_navigator::BtNavigator()
{
}

void ECarBTNavigator::configure()
{
  // Call parent configure
  nav2_bt_navigator::BtNavigator::configure();
  
  // Register eCar-specific nodes
  registerCustomNodes();
  
  // Load eCar-specific behavior tree
  loadECarBehaviorTree();
  
  // Setup eCar-specific parameters
  setupECarSpecificParameters();
  
  RCLCPP_INFO(get_logger(), "eCar BT Navigator configured");
}

void ECarBTNavigator::registerCustomNodes()
{
  // Register all eCar-specific BT nodes
  registerECarBTNodes(factory_);
  
  // Register additional nodes for 4WD4WS capabilities
  factory_.registerSimpleCondition("IsOmnidirectionalModeEnabled",
    [this](BT::TreeNode& node) {
      // Check if 4WS mode is enabled
      return BT::NodeStatus::SUCCESS;  // Placeholder
    });
  
  factory_.registerSimpleAction("EnableOmnidirectionalMode",
    [this](BT::TreeNode& node) {
      RCLCPP_INFO(get_logger(), "Enabling omnidirectional mode");
      // Enable 4WS mode
      return BT::NodeStatus::SUCCESS;
    });
  
  factory_.registerSimpleAction("DisableOmnidirectionalMode",
    [this](BT::TreeNode& node) {
      RCLCPP_INFO(get_logger(), "Disabling omnidirectional mode");
      // Disable 4WS mode
      return BT::NodeStatus::SUCCESS;
    });
}

void ECarBTNavigator::loadECarBehaviorTree()
{
  // Load eCar-specific behavior tree for navigation
  std::string ecar_bt_file = "ecar_navigation_behavior.xml";
  
  // This would load the eCar-specific navigation BT
  // that includes 4WS capabilities and recovery behaviors
}

void ECarBTNavigator::setupECarSpecificParameters()
{
  // Declare eCar-specific parameters
  declare_parameter("ecar.use_4ws", true);
  declare_parameter("ecar.lateral_velocity_limit", 1.0);
  declare_parameter("ecar.omnidirectional_recovery", true);
  
  // Get parameters
  bool use_4ws = get_parameter("ecar.use_4ws").as_bool();
  double lateral_vel_limit = get_parameter("ecar.lateral_velocity_limit").as_double();
  
  RCLCPP_INFO(get_logger(), "eCar 4WS mode: %s", use_4ws ? "enabled" : "disabled");
  RCLCPP_INFO(get_logger(), "Lateral velocity limit: %.2f m/s", lateral_vel_limit);
}
}  // namespace tadeo_ecar_behavior
```

### BT para Navegación Avanzada

```xml
<!-- behavior_trees/ecar_navigation_behavior.xml -->
<root main_tree_to_execute="ECarNavigation">
  
  <BehaviorTree ID="ECarNavigation">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      
      <PipelineSequence name="NavigateWithEnhancedCapabilities">
        
        <!-- Pre-navigation setup -->
        <Sequence name="NavigationSetup">
          <Action ID="IsBatteryOK" min_battery_percentage="15.0"/>
          <Action ID="AreWheelsCalibrated"/>
          <Action ID="CheckSensorStatus"/>
        </Sequence>
        
        <!-- Path planning with eCar considerations -->
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="2" name="ComputePathToPose">
            
            <!-- Try standard planning first -->
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            
            <!-- Recovery: try with omnidirectional planning -->
            <ReactiveFallback name="PlanningRecovery">
              <GoalUpdated/>
              
              <!-- Enable 4WS for better maneuverability -->
              <Sequence name="OmnidirectionalPlanning">
                <Action ID="Enable4WS" enable="true"/>
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="OmnidirectionalPlanner"/>
              </Sequence>
              
              <!-- Clear obstacles -->
              <ClearEntireCostmap name="ClearGlobalCostmap" 
                                 service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
            
          </RecoveryNode>
        </RateController>
        
        <!-- Path following with enhanced capabilities -->
        <RecoveryNode number_of_retries="3" name="FollowPath">
          
          <!-- Try standard path following -->
          <FollowPath path="{path}" controller_id="FollowPath"/>
          
          <!-- Enhanced recovery for eCar -->
          <ReactiveFallback name="FollowPathRecovery">
            <GoalUpdated/>
            
            <!-- Use 4WS lateral movement for obstacles -->
            <Sequence name="LateralAvoidance">
              <Action ID="Enable4WS" enable="true"/>
              <Action ID="LateralMove" distance="0.5" velocity="0.3"/>
              <Action ID="Enable4WS" enable="false"/>
            </Sequence>
            
            <!-- Clear local costmap -->
            <ClearEntireCostmap name="ClearLocalCostmap" 
                               service_name="local_costmap/clear_entirely_local_costmap"/>
          </ReactiveFallback>
          
        </RecoveryNode>
        
      </PipelineSequence>
      
      <!-- Advanced recovery behaviors for eCar -->
      <ReactiveFallback name="ECarRecoveryFallback">
        <GoalUpdated/>
        
        <RoundRobin name="ECarRecoveryActions">
          
          <!-- 4WS-specific recovery maneuvers -->
          <Sequence name="OmnidirectionalRecovery">
            <Action ID="Enable4WS" enable="true"/>
            <Action ID="LateralMove" distance="1.0" velocity="0.2"/>
            <Spin spin_dist="0.785"/>  <!-- 45 degrees -->
            <Action ID="LateralMove" distance="-0.5" velocity="0.2"/>
            <Action ID="Enable4WS" enable="false"/>
          </Sequence>
          
          <!-- Traditional recovery -->
          <Sequence name="StandardRecovery">
            <ClearEntireCostmap name="ClearBothCostmaps" 
                               service_name="local_costmap/clear_entirely_local_costmap"/>
            <Spin spin_dist="1.57"/>
            <Wait wait_duration="3"/>
            <BackUp backup_dist="0.30" backup_speed="0.05"/>
          </Sequence>
          
          <!-- Emergency stop and wait -->
          <Sequence name="EmergencyRecovery">
            <Action ID="EmergencyStop"/>
            <Wait wait_duration="5"/>
            <Action ID="RequestHumanIntervention"/>
          </Sequence>
          
        </RoundRobin>
      </ReactiveFallback>
      
    </RecoveryNode>
  </BehaviorTree>
  
</root>
```

## Debugging y Visualización

### Groot (Editor Gráfico)

```bash
# Instalar Groot para visualización y edición de BTs
sudo apt install ros-humble-groot

# O compilar desde fuente
cd ~/ros2_ws/src
git clone https://github.com/BehaviorTree/Groot.git
cd Groot
git checkout ros2
cd ~/ros2_ws
colcon build --packages-select groot

# Ejecutar Groot
ros2 run groot groot
```

### Monitor de BT en Debugging

```cpp
// include/tadeo_ecar_behavior/bt_monitor.hpp
#ifndef TADEO_ECAR_BEHAVIOR__BT_MONITOR_HPP_
#define TADEO_ECAR_BEHAVIOR__BT_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <std_msgs/msg/string.hpp>

namespace tadeo_ecar_behavior
{
class BTMonitor : public rclcpp::Node
{
public:
  BTMonitor();

private:
  void publishBTStatus();
  void setupBTLogging();
  
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_status_pub_;
  rclcpp::TimerInterface::SharedPtr status_timer_;
  
  BT::BehaviorTreeFactory factory_;
};
}  // namespace tadeo_ecar_behavior

#endif  // TADEO_ECAR_BEHAVIOR__BT_MONITOR_HPP_
```

```cpp
// src/bt_monitor.cpp
#include "tadeo_ecar_behavior/bt_monitor.hpp"
#include "tadeo_ecar_behavior/ecar_bt_nodes.hpp"

namespace tadeo_ecar_behavior
{
BTMonitor::BTMonitor() : Node("bt_monitor")
{
  // Register eCar nodes
  registerECarBTNodes(factory_);
  
  // Setup BT logging for Groot
  setupBTLogging();
  
  // Create publishers
  bt_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "bt_status", 10);
  
  // Status timer
  status_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&BTMonitor::publishBTStatus, this));
  
  RCLCPP_INFO(this->get_logger(), "BT Monitor initialized");
}

void BTMonitor::setupBTLogging()
{
  // Create ZMQ publisher for Groot
  zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(tree_.get());
  
  RCLCPP_INFO(this->get_logger(), "BT logging setup for Groot visualization");
}

void BTMonitor::publishBTStatus()
{
  if (!tree_) {
    return;
  }
  
  // Create status message
  auto status_msg = std_msgs::msg::String();
  
  // Get tree status
  BT::NodeStatus status = tree_->rootNode()->status();
  
  std::string status_str;
  switch (status) {
    case BT::NodeStatus::SUCCESS:
      status_str = "SUCCESS";
      break;
    case BT::NodeStatus::FAILURE:
      status_str = "FAILURE";
      break;
    case BT::NodeStatus::RUNNING:
      status_str = "RUNNING";
      break;
    case BT::NodeStatus::IDLE:
      status_str = "IDLE";
      break;
  }
  
  status_msg.data = "BT Status: " + status_str;
  bt_status_pub_->publish(status_msg);
}
}  // namespace tadeo_ecar_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tadeo_ecar_behavior::BTMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### Herramientas de Análisis

```python
#!/usr/bin/env python3
# scripts/bt_analyzer.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from collections import defaultdict

class BTAnalyzer(Node):
    def __init__(self):
        super().__init__('bt_analyzer')
        
        # Subscribers
        self.bt_status_sub = self.create_subscription(
            String, 'bt_status', self.bt_status_callback, 10)
        
        # Analysis data
        self.status_history = []
        self.node_execution_times = defaultdict(list)
        self.failure_counts = defaultdict(int)
        self.success_counts = defaultdict(int)
        
        # Timer for analysis
        self.analysis_timer = self.create_timer(10.0, self.perform_analysis)
        
        self.get_logger().info('BT Analyzer initialized')
    
    def bt_status_callback(self, msg):
        """Store BT status for analysis"""
        timestamp = time.time()
        self.status_history.append({
            'timestamp': timestamp,
            'status': msg.data
        })
        
        # Keep only last 1000 entries
        if len(self.status_history) > 1000:
            self.status_history.pop(0)
    
    def perform_analysis(self):
        """Analyze BT performance"""
        if len(self.status_history) < 2:
            return
        
        # Calculate execution statistics
        total_executions = len(self.status_history)
        
        # Count status occurrences
        status_counts = defaultdict(int)
        for entry in self.status_history:
            status_counts[entry['status']] += 1
        
        # Calculate success rate
        success_count = status_counts.get('BT Status: SUCCESS', 0)
        failure_count = status_counts.get('BT Status: FAILURE', 0)
        running_count = status_counts.get('BT Status: RUNNING', 0)
        
        total_completed = success_count + failure_count
        success_rate = (success_count / total_completed * 100) if total_completed > 0 else 0
        
        # Calculate average execution time
        execution_times = []
        for i in range(1, len(self.status_history)):
            if (self.status_history[i-1]['status'] == 'BT Status: RUNNING' and 
                self.status_history[i]['status'] in ['BT Status: SUCCESS', 'BT Status: FAILURE']):
                execution_time = self.status_history[i]['timestamp'] - self.status_history[i-1]['timestamp']
                execution_times.append(execution_time)
        
        avg_execution_time = sum(execution_times) / len(execution_times) if execution_times else 0
        
        # Log analysis results
        self.get_logger().info(
            f'BT Analysis - Executions: {total_executions}, '
            f'Success Rate: {success_rate:.1f}%, '
            f'Avg Execution Time: {avg_execution_time:.2f}s'
        )
        
        # Detect performance issues
        if success_rate < 80 and total_completed > 10:
            self.get_logger().warn(f'Low BT success rate: {success_rate:.1f}%')
        
        if avg_execution_time > 30.0:
            self.get_logger().warn(f'High BT execution time: {avg_execution_time:.2f}s')
    
    def generate_report(self):
        """Generate comprehensive BT analysis report"""
        if not self.status_history:
            return "No BT data available for analysis"
        
        # Performance metrics
        total_executions = len(self.status_history)
        
        status_counts = defaultdict(int)
        for entry in self.status_history:
            status_counts[entry['status']] += 1
        
        # Calculate timing statistics
        execution_times = []
        for i in range(1, len(self.status_history)):
            if 'RUNNING' in self.status_history[i-1]['status']:
                dt = self.status_history[i]['timestamp'] - self.status_history[i-1]['timestamp']
                execution_times.append(dt)
        
        report = "=== BT Performance Analysis ===\n\n"
        report += f"Total Status Updates: {total_executions}\n"
        
        for status, count in status_counts.items():
            percentage = (count / total_executions) * 100
            report += f"{status}: {count} ({percentage:.1f}%)\n"
        
        if execution_times:
            avg_time = sum(execution_times) / len(execution_times)
            min_time = min(execution_times)
            max_time = max(execution_times)
            
            report += f"\nExecution Time Statistics:\n"
            report += f"  Average: {avg_time:.2f}s\n"
            report += f"  Min: {min_time:.2f}s\n"
            report += f"  Max: {max_time:.2f}s\n"
        
        return report

def main(args=None):
    rclpy.init(args=args)
    analyzer = BTAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        # Generate final report
        report = analyzer.generate_report()
        print(report)
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Scripts de Testing

```bash
#!/bin/bash
# scripts/test_behavior_trees.sh

echo "=== eCar Behavior Trees Testing ==="

# Test 1: Validate BT XML files
echo "1. Validating BT XML files..."

BT_FILES=(
    "behavior_trees/ecar_main_behavior.xml"
    "behavior_trees/ecar_navigation_behavior.xml"
)

for bt_file in "${BT_FILES[@]}"; do
    if [ -f "$bt_file" ]; then
        echo "✓ Found: $bt_file"
        # Validate XML syntax
        xmllint --noout "$bt_file" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "✓ Valid XML: $bt_file"
        else
            echo "✗ Invalid XML: $bt_file"
        fi
    else
        echo "✗ Missing: $bt_file"
    fi
done

# Test 2: Test BT node registration
echo -e "\n2. Testing BT node registration..."
ros2 run tadeo_ecar_behavior test_bt_nodes &
TEST_PID=$!
sleep 3

if ps -p $TEST_PID > /dev/null; then
    echo "✓ BT nodes test running"
    kill $TEST_PID
else
    echo "✗ BT nodes test failed"
fi

# Test 3: Test BT execution
echo -e "\n3. Testing BT execution..."
timeout 30s ros2 run tadeo_ecar_behavior ecar_bt_navigator &
BT_PID=$!
sleep 5

if ps -p $BT_PID > /dev/null; then
    echo "✓ BT Navigator running"
    
    # Test BT status
    BT_STATUS=$(timeout 5s ros2 topic echo /bt_status --once 2>/dev/null | grep "data:" | cut -d'"' -f2)
    if [ -n "$BT_STATUS" ]; then
        echo "✓ BT Status: $BT_STATUS"
    else
        echo "✗ No BT status received"
    fi
    
    kill $BT_PID
else
    echo "✗ BT Navigator failed to start"
fi

# Test 4: Test Groot connectivity
echo -e "\n4. Testing Groot connectivity..."
if command -v groot >/dev/null 2>&1; then
    echo "✓ Groot is installed"
    
    # Check if ZMQ port is available
    if netstat -tuln | grep -q ":1666"; then
        echo "✓ ZMQ publisher is active"
    else
        echo "✗ ZMQ publisher not found"
    fi
else
    echo "✗ Groot not installed"
fi

echo -e "\n=== BT Testing Complete ==="
```

## Ejercicios Prácticos

### Ejercicio 1: Crear Nodo BT Personalizado

```cpp
// TODO: Implementar nodo BT personalizado
class CheckSensorStatus : public BT::ConditionNode
{
public:
    CheckSensorStatus(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("sensor_name", "Sensor to check"),
            BT::InputPort<double>("timeout", 5.0, "Timeout in seconds")
        };
    }
    
private:
    // TODO: Implementar lógica de verificación de sensores
};
```

### Ejercicio 2: Diseñar BT para Misión Compleja

```xml
<!-- TODO: behavior_trees/delivery_mission.xml -->
<!-- Diseñar BT para misión de entrega que incluya:
- Navegación a punto de recogida
- Verificación de carga
- Navegación a punto de entrega  
- Confirmación de entrega
- Regreso a base
- Manejo de errores y recovery
-->
```

### Ejercicio 3: Integrar BT con Sistema Completo

```bash
# TODO: Integrar BT con el sistema completo del eCar
# 1. Compilar todos los nodos BT
colcon build --packages-select tadeo_ecar_behavior

# 2. Lanzar sistema completo con BT
ros2 launch tadeo_ecar_bringup ecar_full_system_bt.launch.py

# 3. Enviar misión de prueba
ros2 service call /start_mission std_srvs/srv/Trigger

# 4. Monitorear con Groot
ros2 run groot groot
```

## Conclusiones del Capítulo

En este capítulo hemos aprendido:

1. **Fundamentos BT**: Estructura jerárquica vs máquinas de estado
2. **BehaviorTree.CPP**: Framework para implementación en ROS2
3. **Nodos Básicos**: Control, decoradores y leaf nodes
4. **Nodos Personalizados**: Implementación específica para eCar
5. **BT para eCar**: Comportamientos complejos con capacidades 4WD4WS
6. **Integración Nav2**: Navigator personalizado para eCar
7. **Debugging**: Groot y herramientas de análisis
8. **Testing**: Validación y verificación de comportamientos

### Checklist de Conocimientos

Antes de continuar al siguiente capítulo:

- [ ] Entiendes la diferencia entre BT y FSM
- [ ] Puedes crear nodos BT personalizados
- [ ] Sabes diseñar comportamientos complejos
- [ ] Comprendes el uso del Blackboard
- [ ] Puedes integrar BT con navegación
- [ ] Sabes usar Groot para debugging
- [ ] Has implementado BTs para eCar específicos

### Próximo Capítulo

En el Capítulo 13 estudiaremos:
- Control avanzado para robots 4WD4WS
- Cinemática y dinámica del eCar
- Controladores PID y MPC
- Control omnidireccional
- Integración con actuadores

## Referencias

- [BehaviorTree.CPP](https://www.behaviortree.dev/)
- [Groot](https://github.com/BehaviorTree/Groot)
- [Nav2 Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)
- [BT Design Patterns](https://arxiv.org/abs/1709.00084)
- [Robotics Behavior Trees](https://robohub.org/introduction-to-behavior-trees/)