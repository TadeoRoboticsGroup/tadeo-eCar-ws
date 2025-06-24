# Tadeo eCar Config Package

This package contains all configuration files for the Tadeo eCar autonomous robot system. It provides centralized parameter management for all components of the robot.

## Package Overview

The `tadeo_ecar_config` package provides:
- Robot physical parameters and specifications
- Controller configuration for 4WD4WS system
- Navigation stack (Nav2) parameters
- Sensor calibration and configuration
- Safety system parameters

## Directory Structure

```
tadeo_ecar_config/
├── config/
│   ├── robot_params.yaml      # Robot physical parameters
│   ├── control_params.yaml    # Controller PID parameters
│   └── nav2_params.yaml       # Navigation stack configuration
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Configuration Files

### robot_params.yaml
Contains robot physical specifications and sensor parameters:
- Robot dimensions and wheel configuration
- Sensor mounting positions and specifications
- Safety limits and emergency parameters
- Battery and power system settings

### control_params.yaml
4WD4WS controller configuration:
- Individual wheel PID controllers
- Steering angle limits and rates
- Velocity and acceleration limits
- Control loop frequencies

### nav2_params.yaml
Complete Nav2 navigation stack configuration:
- Controller server parameters
- Planner server configuration
- Recovery behavior settings
- Costmap parameters for local and global planning

## Usage Examples

### Loading Robot Parameters
```bash
# Load robot parameters in a launch file
robot_config = os.path.join(
    get_package_share_directory('tadeo_ecar_config'),
    'config', 'robot_params.yaml'
)

Node(
    package='your_package',
    executable='your_node',
    parameters=[robot_config]
)
```

### Loading Controller Parameters
```bash
# Load controller parameters
control_config = os.path.join(
    get_package_share_directory('tadeo_ecar_config'),
    'config', 'control_params.yaml'
)

Node(
    package='tadeo_ecar_control',
    executable='wheel_controller_node',
    parameters=[control_config]
)
```

### Loading Nav2 Parameters
```bash
# Load navigation parameters
nav2_config = os.path.join(
    get_package_share_directory('tadeo_ecar_config'),
    'config', 'nav2_params.yaml'
)

Node(
    package='nav2_controller',
    executable='controller_server',
    parameters=[nav2_config]
)
```

### Using Parameters in C++ Nodes
```cpp
#include <rclcpp/rclcpp.hpp>

class YourNode : public rclcpp::Node
{
public:
    YourNode() : Node("your_node")
    {
        // Declare and get parameters
        this->declare_parameter("robot.wheelbase", 1.2);
        this->declare_parameter("robot.track_width", 1.0);
        
        double wheelbase = this->get_parameter("robot.wheelbase").as_double();
        double track_width = this->get_parameter("robot.track_width").as_double();
        
        RCLCPP_INFO(this->get_logger(), "Wheelbase: %.2f, Track width: %.2f", 
                    wheelbase, track_width);
    }
};
```

### Using Parameters in Python Nodes
```python
import rclpy
from rclpy.node import Node

class YourNode(Node):
    def __init__(self):
        super().__init__('your_node')
        
        # Declare and get parameters
        self.declare_parameter('robot.max_velocity', 2.0)
        self.declare_parameter('control.pid_kp', 1.0)
        
        max_vel = self.get_parameter('robot.max_velocity').value
        pid_kp = self.get_parameter('control.pid_kp').value
        
        self.get_logger().info(f'Max velocity: {max_vel}, PID Kp: {pid_kp}')
```

## Integration with Other Packages

This config package is used by:
- `tadeo_ecar_control`: Controller parameters and limits
- `tadeo_ecar_navigation`: Nav2 stack configuration
- `tadeo_ecar_safety`: Safety limits and emergency parameters
- `tadeo_ecar_perception`: Sensor specifications and mounting
- `tadeo_ecar_localization`: Sensor fusion parameters

## Parameter Validation

All configuration files include parameter validation to ensure:
- Physical limits are respected (wheel angles, velocities)
- Safety parameters are within acceptable ranges
- Controller gains are stable
- Navigation parameters are consistent

## Customization

To customize parameters for your specific robot:

1. **Robot Dimensions**: Modify `robot_params.yaml` with your robot's specifications
2. **Controller Tuning**: Adjust PID gains in `control_params.yaml`
3. **Navigation Behavior**: Tune Nav2 parameters in `nav2_params.yaml`

## Dependencies

- `rclcpp` (C++ nodes)
- `rclpy` (Python nodes)
- Standard ROS2 parameter system

## Maintenance

When modifying configuration files:
1. Validate parameter ranges and units
2. Test with simulation before deploying to hardware
3. Document any changes in this README
4. Update related packages if interfaces change

## Troubleshooting

### Common Issues

1. **Parameter not found**: Ensure parameter names match exactly in YAML files
2. **Invalid values**: Check parameter ranges and data types
3. **Node crashes**: Verify all required parameters are declared

### Debugging Parameters
```bash
# List all parameters for a node
ros2 param list /your_node_name

# Get specific parameter value
ros2 param get /your_node_name parameter_name

# Set parameter at runtime
ros2 param set /your_node_name parameter_name value
```

## Related Packages

- `tadeo_ecar_description`: Robot URDF and visualization
- `tadeo_ecar_control`: Motion controllers
- `tadeo_ecar_navigation`: Autonomous navigation
- `tadeo_ecar_safety`: Safety monitoring systems