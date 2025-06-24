# Tadeo eCar Safety Package

This package provides comprehensive safety and emergency systems for the Tadeo eCar autonomous robot. It implements multiple layers of safety protection including emergency stops, collision avoidance, system monitoring, and watchdog functionality.

## Package Overview

The `tadeo_ecar_safety` package provides:
- Emergency stop system with multiple trigger sources
- Real-time collision avoidance and obstacle detection
- Comprehensive system health and safety monitoring
- Component watchdog for system reliability
- Multi-layered safety architecture with redundancy

## Directory Structure

```
tadeo_ecar_safety/
├── include/tadeo_ecar_safety/
│   └── safety_types.hpp                # Safety data structures and enums
├── src/
│   ├── emergency_stop_node.cpp         # Emergency stop system
│   ├── collision_avoidance_node.cpp    # Collision avoidance and safety zones
│   ├── safety_monitor_node.cpp         # System health and limits monitoring
│   └── watchdog_node.cpp               # Component failure detection
├── launch/
│   └── safety_system.launch.py         # Complete safety system launch
├── config/
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Nodes

### 1. Emergency Stop Node (`emergency_stop_node`)

Central emergency stop system with multiple trigger sources and safety command filtering.

**Subscribed Topics:**
- `joy` (sensor_msgs/Joy): Joystick input for emergency button
- `cmd_vel_input` (geometry_msgs/Twist): Raw velocity commands to filter
- `safety_status` (tadeo_ecar_msgs/SafetyStatus): Safety system status
- `system_health` (tadeo_ecar_msgs/SystemHealth): Component health status

**Published Topics:**
- `cmd_vel_safe` (geometry_msgs/Twist): Safety-filtered velocity commands
- `emergency_stop_active` (std_msgs/Bool): Emergency stop status
- `emergency_reason` (std_msgs/String): Reason for emergency stop
- `safety_override` (std_msgs/Bool): Safety override status

**Services:**
- `emergency_stop` (tadeo_ecar_interfaces/EmergencyStop): Manual emergency stop control

**Key Features:**
- Multiple emergency trigger sources (joystick, service calls, system failures)
- Command timeout safety (stops robot if no commands received)
- Heartbeat monitoring for communication loss detection
- Manual and automatic reset capabilities
- Safety override for maintenance operations

### 2. Collision Avoidance Node (`collision_avoidance_node`)

Real-time collision avoidance using safety zones and obstacle prediction.

**Subscribed Topics:**
- `cmd_vel_raw` (geometry_msgs/Twist): Input velocity commands
- `scan` (sensor_msgs/LaserScan): LiDAR scan data
- `perception/fused_obstacles` (visualization_msgs/MarkerArray): Detected obstacles
- `robot_status` (tadeo_ecar_msgs/RobotStatus): Current robot state

**Published Topics:**
- `cmd_vel_safe` (geometry_msgs/Twist): Collision-safe velocity commands
- `safety_status` (tadeo_ecar_msgs/SafetyStatus): Collision safety status
- `safety_zones` (visualization_msgs/MarkerArray): Safety zone visualization
- `collision_prediction` (geometry_msgs/PointStamped): Predicted collision points

**Key Features:**
- Multi-zone safety system (emergency, warning, slow zones)
- Real-time collision prediction and avoidance
- Lateral avoidance for 4WS capability
- Adaptive deceleration based on obstacle distance
- Safety zone visualization for debugging

### 3. Safety Monitor Node (`safety_monitor_node`)

Comprehensive system health monitoring with safety limit enforcement.

**Subscribed Topics:**
- `imu/data` (sensor_msgs/Imu): IMU data for stability monitoring
- `battery_state` (sensor_msgs/BatteryState): Battery status
- `cmd_vel` (geometry_msgs/Twist): Velocity commands for limit checking
- `robot_status` (tadeo_ecar_msgs/RobotStatus): Robot operational status
- `system_health` (tadeo_ecar_msgs/SystemHealth): Component health updates

**Published Topics:**
- `safety_monitor_status` (tadeo_ecar_msgs/SafetyStatus): Monitor status
- `safety/tilt_angle` (std_msgs/Float64): Current robot tilt angle
- `safety/lateral_acceleration` (std_msgs/Float64): Lateral acceleration
- `safety/stability_factor` (std_msgs/Float64): Vehicle stability metric
- `safety/battery_health` (std_msgs/Float64): Battery health factor

**Key Features:**
- Real-time stability monitoring (tilt angle, lateral acceleration)
- Battery health and critical level monitoring
- Velocity and acceleration limit enforcement
- Temperature monitoring and thermal protection
- System health aggregation and analysis

### 4. Watchdog Node (`watchdog_node`)

System-wide component monitoring with automatic failure detection.

**Subscribed Topics:**
- `{component_name}/health` (tadeo_ecar_msgs/SystemHealth): Individual component health
- `system_health` (tadeo_ecar_msgs/SystemHealth): Generic health updates

**Published Topics:**
- `watchdog_status` (tadeo_ecar_msgs/SystemHealth): Watchdog system status
- `system_alive` (std_msgs/Bool): Overall system health indicator
- `failed_components` (std_msgs/String): List of failed components

**Key Features:**
- Monitors all critical system components
- Configurable timeout periods per component type
- Automatic emergency stop on critical failures
- Component recovery detection
- System-wide health reporting

## Safety Architecture

### Multi-Layer Safety Design

1. **Layer 1: Emergency Stop**
   - Immediate vehicle stop capability
   - Multiple trigger sources (joystick, service, automatic)
   - Highest priority safety system

2. **Layer 2: Collision Avoidance**
   - Real-time obstacle detection and avoidance
   - Safety zone monitoring
   - Predictive collision detection

3. **Layer 3: System Monitoring**
   - Continuous health monitoring
   - Safety limit enforcement
   - Environmental condition monitoring

4. **Layer 4: Watchdog Protection**
   - Component failure detection
   - System-wide health assessment
   - Automatic recovery mechanisms

### Safety Zones

The collision avoidance system uses three concentric safety zones:

- **Emergency Zone (0-0.3m)**: Immediate stop
- **Warning Zone (0.3-1.0m)**: Hard deceleration
- **Slow Zone (1.0-2.0m)**: Moderate deceleration

## Usage Examples

### Basic Launch
```bash
# Launch complete safety system
ros2 launch tadeo_ecar_safety safety_system.launch.py

# Launch with specific components
ros2 launch tadeo_ecar_safety safety_system.launch.py enable_collision_avoidance:=true enable_watchdog:=true
```

### Emergency Operations
```bash
# Trigger emergency stop
ros2 service call /emergency_stop tadeo_ecar_interfaces/srv/EmergencyStop "{activate: true}"

# Reset emergency stop
ros2 service call /emergency_stop tadeo_ecar_interfaces/srv/EmergencyStop "{activate: false}"

# Check emergency status
ros2 topic echo /emergency_stop_active
```

### Safety Monitoring
```bash
# Monitor safety status
ros2 topic echo /safety_status
ros2 topic echo /safety_monitor_status

# Monitor system health
ros2 topic echo /system_alive
ros2 topic echo /failed_components

# Monitor stability metrics
ros2 topic echo /safety/tilt_angle
ros2 topic echo /safety/stability_factor
```

### Collision Avoidance
```bash
# Monitor safety zones
ros2 topic echo /safety_zones

# View collision predictions
ros2 topic echo /collision_prediction

# Monitor obstacle distances
ros2 topic echo /safety_status --filter "closest_obstacle_distance"
```

## Configuration

### Emergency Stop Parameters
```yaml
emergency:
  enable_joystick: true
  joystick_button: 0
  auto_reset_timeout: 5.0
  require_manual_reset: true

safety:
  max_cmd_age: 1.0
  enable_heartbeat: true
  heartbeat_timeout: 2.0
```

### Collision Avoidance Parameters
```yaml
zones:
  emergency_distance: 0.3
  warning_distance: 1.0
  slow_distance: 2.0
  angular_range: 1.57  # 90 degrees

avoidance:
  enable_lateral: true
  lateral_gain: 0.5
  deceleration_gain: 0.8
```

### Safety Monitor Parameters
```yaml
limits:
  max_tilt_angle: 0.524  # 30 degrees
  max_lateral_acceleration: 8.0
  max_linear_velocity: 2.0

battery:
  critical_voltage: 20.0
  warning_voltage: 22.0
  critical_percentage: 10.0
```

### Watchdog Parameters
```yaml
timeouts:
  control_system: 2.0
  perception_system: 3.0
  safety_system: 1.0

critical_components:
  control: true
  perception: false
  safety: true
```

## Safety Features

### Emergency Stop Triggers
- **Manual**: Joystick button press
- **Service**: ROS service call
- **Automatic**: System failure detection
- **Timeout**: Communication loss
- **Health**: Component failures

### Collision Avoidance
- **Zone-based**: Multiple safety zones with different responses
- **Predictive**: Time-to-collision calculation
- **Adaptive**: Speed-dependent safety distances
- **Lateral**: 4WS-enabled lateral avoidance

### System Protection
- **Stability**: Tilt angle and lateral acceleration monitoring
- **Thermal**: Temperature-based protection
- **Power**: Battery health monitoring
- **Communication**: Heartbeat and timeout detection

## Integration with Other Packages

### Dependencies
- `tadeo_ecar_msgs`: Safety status and system health messages
- `tadeo_ecar_interfaces`: Emergency stop and safety services
- `tadeo_ecar_config`: Safety parameters and limits

### Data Flow
```
Input Commands → Emergency Stop → Collision Avoidance → Control System
                      ↑                    ↑
                Safety Monitor ←→ Watchdog System
```

## Advanced Features

### Redundant Safety Systems
- Multiple independent safety checks
- Cross-validation between safety nodes
- Graceful degradation on component failure

### Adaptive Safety
- Dynamic safety zone sizing based on speed
- Environmental condition adaptation
- Predictive safety based on robot behavior

### Health Monitoring
- Real-time component health tracking
- Predictive failure detection
- Automatic recovery mechanisms

## Troubleshooting

### Common Issues

1. **Emergency stop stuck active**:
   ```bash
   # Check emergency reason
   ros2 topic echo /emergency_reason
   
   # Manual reset
   ros2 service call /emergency_stop tadeo_ecar_interfaces/srv/EmergencyStop "{activate: false}"
   ```

2. **Collision avoidance too aggressive**:
   ```bash
   # Adjust safety zone distances
   ros2 param set /collision_avoidance zones.warning_distance 1.5
   ```

3. **Safety monitor false alarms**:
   ```bash
   # Check stability readings
   ros2 topic echo /safety/stability_factor
   
   # Adjust thresholds if needed
   ros2 param set /safety_monitor stability.warning_factor 0.3
   ```

4. **Watchdog triggering emergency**:
   ```bash
   # Check failed components
   ros2 topic echo /failed_components
   
   # Check component health
   ros2 topic echo /system_health
   ```

### Performance Optimization
- Adjust monitoring frequencies based on computational resources
- Tune safety zone parameters for specific environments
- Configure component timeouts based on system capabilities

## Related Packages

- `tadeo_ecar_control`: Receives safety-filtered commands
- `tadeo_ecar_perception`: Provides obstacle detection for collision avoidance
- `tadeo_ecar_navigation`: Integrates safety constraints in path planning
- `tadeo_ecar_bringup`: Includes safety system in overall robot launch