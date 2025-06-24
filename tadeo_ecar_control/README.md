# Tadeo eCar Control Package

This package provides comprehensive vehicle control for the Tadeo eCar 4WD4WS autonomous robot. It implements advanced control algorithms for four-wheel drive and four-wheel steering systems.

## Package Overview

The `tadeo_ecar_control` package provides:
- Individual wheel velocity and steering control
- Four-wheel steering (4WS) coordination
- Vehicle dynamics monitoring and stability control
- Real-time odometry calculation
- Safety and emergency control features

## Directory Structure

```
tadeo_ecar_control/
├── include/tadeo_ecar_control/
│   └── wheel_controller.hpp     # Wheel controller header
├── src/
│   ├── wheel_controller_node.cpp           # Individual wheel control
│   ├── four_wheel_steering_controller_node.cpp  # 4WS coordination
│   └── vehicle_dynamics_node.cpp           # Vehicle dynamics and stability
├── launch/
│   └── control_system.launch.py           # Complete control system launch
├── config/
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Nodes

### 1. Wheel Controller Node (`wheel_controller_node`)

Controls individual wheel velocities and steering angles using PID controllers.

**Subscribed Topics:**
- `cmd_vel` (geometry_msgs/Twist): Target vehicle velocities
- `joint_states` (sensor_msgs/JointState): Current wheel states from simulation/hardware

**Published Topics:**
- `fl_wheel_velocity_controller/command` (std_msgs/Float64): Front-left wheel velocity command
- `fr_wheel_velocity_controller/command` (std_msgs/Float64): Front-right wheel velocity command
- `rl_wheel_velocity_controller/command` (std_msgs/Float64): Rear-left wheel velocity command
- `rr_wheel_velocity_controller/command` (std_msgs/Float64): Rear-right wheel velocity command
- `fl_steering_controller/command` (std_msgs/Float64): Front-left steering command
- `fr_steering_controller/command` (std_msgs/Float64): Front-right steering command
- `rl_steering_controller/command` (std_msgs/Float64): Rear-left steering command
- `rr_steering_controller/command` (std_msgs/Float64): Rear-right steering command
- `wheel_states` (tadeo_ecar_msgs/WheelStates): Current wheel states and diagnostics

**Services:**
- `calibrate_wheels` (tadeo_ecar_interfaces/CalibrateWheels): Calibrate wheel positions

**Key Features:**
- Individual PID control for each wheel
- Ackermann steering geometry calculation
- Command timeout safety
- Wheel slip detection
- Velocity and steering angle limiting

### 2. Four Wheel Steering Controller Node (`four_wheel_steering_controller_node`)

Coordinates 4WS behavior and publishes vehicle odometry.

**Subscribed Topics:**
- `cmd_vel` (geometry_msgs/Twist): High-level vehicle commands
- `wheel_states` (tadeo_ecar_msgs/WheelStates): Current wheel states

**Published Topics:**
- `cmd_vel_out` (geometry_msgs/TwistStamped): Processed vehicle commands
- `odom` (nav_msgs/Odometry): Vehicle odometry
- `robot_status` (tadeo_ecar_msgs/RobotStatus): Vehicle status information

**Key Features:**
- Automatic switching between 4WS and front-wheel steering modes
- Enhanced low-speed maneuverability with 4WS
- Real-time odometry calculation
- TF frame broadcasting
- Crab walk capability

### 3. Vehicle Dynamics Node (`vehicle_dynamics_node`)

Monitors vehicle dynamics and applies stability control.

**Subscribed Topics:**
- `cmd_vel` (geometry_msgs/Twist): Target velocities
- `imu` (sensor_msgs/Imu): IMU data for dynamics feedback
- `wheel_states` (tadeo_ecar_msgs/WheelStates): Wheel state feedback

**Published Topics:**
- `dynamics/cmd_vel` (geometry_msgs/Twist): Stability-controlled commands
- `dynamics/lateral_acceleration` (std_msgs/Float64): Current lateral acceleration
- `dynamics/slip_angle` (std_msgs/Float64): Vehicle slip angle
- `dynamics/stability_factor` (std_msgs/Float64): Vehicle stability metric

**Key Features:**
- Real-time stability monitoring
- Lateral acceleration limiting
- Slip angle calculation
- Automatic velocity reduction during instability
- Wheel slip compensation

## Usage Examples

### Basic Launch
```bash
# Launch complete control system
ros2 launch tadeo_ecar_control control_system.launch.py

# Launch with specific parameters
ros2 launch tadeo_ecar_control control_system.launch.py enable_4ws:=true use_sim_time:=true
```

### Individual Node Launch
```bash
# Launch only wheel controller
ros2 run tadeo_ecar_control wheel_controller_node --ros-args --params-file config/control_params.yaml

# Launch 4WS controller
ros2 run tadeo_ecar_control four_wheel_steering_controller_node

# Launch vehicle dynamics
ros2 run tadeo_ecar_control vehicle_dynamics_node
```

### Sending Velocity Commands
```bash
# Forward motion
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Turning motion
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"

# Lateral motion (4WS mode)
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### Wheel Calibration
```bash
# Calibrate all wheels to zero position
ros2 service call /calibrate_wheels tadeo_ecar_interfaces/srv/CalibrateWheels "{}"
```

### Monitoring Vehicle State
```bash
# Monitor wheel states
ros2 topic echo /wheel_states

# Monitor vehicle odometry
ros2 topic echo /odom

# Monitor robot status
ros2 topic echo /robot_status

# Monitor vehicle dynamics
ros2 topic echo /dynamics/stability_factor
```

## Configuration

### Control Parameters
Key parameters in `control_params.yaml`:

```yaml
control:
  frequency: 50.0
  cmd_timeout: 0.5
  enable_4ws: true
  4ws_speed_threshold: 0.5
  max_steering_angle: 0.785

wheels:
  fl:
    max_velocity: 10.0
    max_steering_angle: 0.785
    velocity_pid: {kp: 1.0, ki: 0.1, kd: 0.05}
    steering_pid: {kp: 2.0, ki: 0.2, kd: 0.1}
```

### Vehicle Dynamics Parameters
```yaml
vehicle:
  mass: 150.0
  inertia: 50.0
  center_of_gravity_height: 0.3

dynamics:
  max_lateral_acceleration: 8.0
  max_slip_angle: 0.349
  stability_threshold: 0.8

tires:
  cornering_stiffness_front: 100000.0
  cornering_stiffness_rear: 120000.0
  friction_coefficient: 0.8
```

## Safety Features

### Emergency Stop
```bash
# Emergency stop all motion
ros2 service call /emergency_stop tadeo_ecar_interfaces/srv/EmergencyStop "{}"
```

### Stability Control
- Automatic velocity reduction when stability factor drops below threshold
- Lateral acceleration limiting to prevent rollover
- Wheel slip detection and compensation
- IMU-based feedback for enhanced stability

### Command Timeout
- Automatic stop if no command received within timeout period
- Configurable timeout duration
- Safe state maintenance during communication loss

## Integration with Other Packages

### Dependencies
- `tadeo_ecar_msgs`: Custom message definitions
- `tadeo_ecar_interfaces`: Service and action definitions
- `tadeo_ecar_config`: Configuration parameters

### Interfaces
- **Input**: High-level velocity commands from navigation/teleoperation
- **Output**: Low-level wheel commands to hardware/simulation
- **Feedback**: Odometry and status information to navigation stack

## Advanced Features

### Four-Wheel Steering Modes

1. **Front-Wheel Steering (FWS)**: Traditional steering for higher speeds
2. **Four-Wheel Steering (4WS)**: Enhanced maneuverability for low speeds
3. **Crab Walk**: Lateral movement with all wheels steered same direction

### Adaptive Control
- Speed-dependent steering mode switching
- Dynamic stability monitoring
- Real-time parameter adaptation
- Predictive slip prevention

## Troubleshooting

### Common Issues

1. **Wheels not responding**:
   - Check wheel calibration: `ros2 service call /calibrate_wheels`
   - Verify joint_states topic is publishing
   - Check PID parameter values

2. **Poor tracking performance**:
   - Tune PID controllers in `control_params.yaml`
   - Verify wheel encoder feedback
   - Check for mechanical issues

3. **Stability warnings**:
   - Reduce maximum velocities
   - Check vehicle load distribution
   - Verify IMU calibration

4. **4WS not working**:
   - Ensure `enable_4ws` parameter is true
   - Check steering angle limits
   - Verify rear wheel steering hardware

### Debugging Commands
```bash
# Check node status
ros2 node list | grep control

# Monitor control loop frequency
ros2 topic hz /wheel_states

# View active parameters
ros2 param list /wheel_controller

# Check for errors
ros2 log show wheel_controller
```

## Performance Optimization

### Real-time Considerations
- Use appropriate control frequencies (50-100 Hz)
- Minimize computational overhead in control loops
- Optimize PID calculations for real-time performance

### Hardware Integration
- Ensure adequate motor controller bandwidth
- Use high-resolution encoders for accurate feedback
- Implement proper electrical noise filtering

## Related Packages

- `tadeo_ecar_description`: Robot model and simulation setup
- `tadeo_ecar_safety`: Safety monitoring and emergency systems
- `tadeo_ecar_navigation`: High-level navigation and path planning
- `tadeo_ecar_perception`: Sensor data processing for control feedback