# Tadeo eCar Localization Package

This package provides advanced localization and sensor fusion capabilities for the Tadeo eCar autonomous robot. It implements multiple filtering algorithms and sensor fusion techniques for robust and accurate position estimation.

## Package Overview

The `tadeo_ecar_localization` package provides:
- Extended Kalman Filter (EKF) localization
- Unscented Kalman Filter (UKF) localization
- Multi-sensor fusion for optimal pose estimation
- Map-based pose correction using scan matching
- Real-time health monitoring of localization systems

## Directory Structure

```
tadeo_ecar_localization/
├── include/tadeo_ecar_localization/
│   └── kalman_filter_base.hpp          # Base classes for filtering
├── src/
│   ├── ekf_localization_node.cpp       # Extended Kalman Filter
│   ├── ukf_localization_node.cpp       # Unscented Kalman Filter
│   ├── sensor_fusion_localization_node.cpp  # Multi-filter fusion
│   └── map_matcher_node.cpp            # Map-based correction
├── launch/
│   └── localization_system.launch.py   # Complete localization system
├── config/
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Nodes

### 1. EKF Localization Node (`ekf_localization_node`)

Implements Extended Kalman Filter for state estimation with linearized motion models.

**Subscribed Topics:**
- `odom` (nav_msgs/Odometry): Wheel odometry measurements
- `imu/data` (sensor_msgs/Imu): IMU acceleration data
- `initialpose` (geometry_msgs/PoseWithCovarianceStamped): Initial pose estimate

**Published Topics:**
- `odometry/filtered` (nav_msgs/Odometry): EKF-filtered odometry
- `pose` (geometry_msgs/PoseWithCovarianceStamped): Current pose estimate
- `localization/ekf_health` (tadeo_ecar_msgs/SystemHealth): EKF health status

**Key Features:**
- 8-state vehicle model (position, velocity, acceleration)
- Adaptive process noise based on time step
- Automatic state covariance monitoring
- Transform broadcasting capability

### 2. UKF Localization Node (`ukf_localization_node`)

Implements Unscented Kalman Filter for better handling of non-linear systems.

**Subscribed Topics:**
- `odom` (nav_msgs/Odometry): Wheel odometry measurements
- `imu/data` (sensor_msgs/Imu): IMU acceleration data
- `initialpose` (geometry_msgs/PoseWithCovarianceStamped): Initial pose estimate

**Published Topics:**
- `odometry/ukf_filtered` (nav_msgs/Odometry): UKF-filtered odometry
- `ukf_pose` (geometry_msgs/PoseWithCovarianceStamped): Current pose estimate
- `localization/ukf_health` (tadeo_ecar_msgs/SystemHealth): UKF health status

**Key Features:**
- Sigma point-based non-linear filtering
- Better handling of non-Gaussian noise
- Configurable UKF parameters (alpha, beta, kappa)
- Non-linear process model for improved accuracy

### 3. Sensor Fusion Localization Node (`sensor_fusion_localization_node`)

Combines multiple localization sources for optimal pose estimation.

**Subscribed Topics:**
- `odometry/filtered` (nav_msgs/Odometry): EKF localization output
- `odometry/ukf_filtered` (nav_msgs/Odometry): UKF localization output
- `gps/fix` (sensor_msgs/NavSatFix): GPS position (optional)

**Published Topics:**
- `odometry/fused` (nav_msgs/Odometry): Fused odometry estimate
- `pose/fused` (geometry_msgs/PoseWithCovarianceStamped): Fused pose estimate
- `localization/fusion_health` (tadeo_ecar_msgs/SystemHealth): Fusion health status

**Key Features:**
- Weighted fusion of multiple estimates
- Data freshness monitoring
- Automatic fallback on sensor failure
- Conservative covariance estimation

### 4. Map Matcher Node (`map_matcher_node`)

Provides map-based pose correction using scan matching techniques.

**Subscribed Topics:**
- `pose/fused` (geometry_msgs/PoseWithCovarianceStamped): Input pose estimate
- `map` (nav_msgs/OccupancyGrid): Occupancy grid map
- `scan` (sensor_msgs/LaserScan): LiDAR scan data

**Published Topics:**
- `pose/corrected` (geometry_msgs/PoseWithCovarianceStamped): Map-corrected pose
- `localization/map_matcher_health` (tadeo_ecar_msgs/SystemHealth): Matcher health

**Key Features:**
- Grid-based scan matching
- Configurable search parameters
- Correction distance limiting
- Score-based matching validation

## Usage Examples

### Basic Launch
```bash
# Launch complete localization system
ros2 launch tadeo_ecar_localization localization_system.launch.py

# Launch with specific filters
ros2 launch tadeo_ecar_localization localization_system.launch.py enable_ekf:=true enable_ukf:=true enable_fusion:=true
```

### Individual Filter Launch
```bash
# Launch only EKF
ros2 run tadeo_ecar_localization ekf_localization_node

# Launch only UKF
ros2 run tadeo_ecar_localization ukf_localization_node

# Launch sensor fusion
ros2 run tadeo_ecar_localization sensor_fusion_localization_node
```

### Setting Initial Pose
```bash
# Set initial pose estimate
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'map'
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]"
```

### Monitoring Localization
```bash
# Monitor EKF output
ros2 topic echo /odometry/filtered

# Monitor UKF output
ros2 topic echo /odometry/ukf_filtered

# Monitor fused output
ros2 topic echo /odometry/fused

# Monitor corrected pose
ros2 topic echo /pose/corrected

# Check localization health
ros2 topic echo /localization/fusion_health
```

## Configuration

### EKF Parameters
```yaml
ekf_localization:
  filter_frequency: 50.0
  publish_tf: true
  map_frame: "map"
  odom_frame: "odom"
  base_link_frame: "base_link"
  use_imu: true
  use_odom: true
  initial_x: 0.0
  initial_y: 0.0
  initial_yaw: 0.0
```

### UKF Parameters
```yaml
ukf_localization:
  filter_frequency: 50.0
  alpha: 0.001    # Sigma point spread
  beta: 2.0       # Prior knowledge parameter
  kappa: 0.0      # Secondary scaling parameter
```

### Sensor Fusion Parameters
```yaml
sensor_fusion:
  fusion_frequency: 20.0
  ekf_weight: 0.6
  ukf_weight: 0.4
  gps_weight: 0.3
  use_gps: false
```

### Map Matching Parameters
```yaml
map_matcher:
  matching_frequency: 10.0
  max_correction_distance: 0.5
  min_scan_match_score: 0.7
  enable_correction: true
```

## Algorithm Details

### Extended Kalman Filter (EKF)
- **State Vector**: [x, y, yaw, vx, vy, vyaw, ax, ay]
- **Process Model**: Constant acceleration with time-varying Jacobians
- **Measurement Models**: Direct observation of pose and velocity states
- **Advantages**: Computationally efficient, well-established
- **Limitations**: Assumes local linearity, may struggle with highly non-linear dynamics

### Unscented Kalman Filter (UKF)
- **State Vector**: [x, y, yaw, vx, vy, vyaw, ax, ay]
- **Process Model**: Non-linear constant acceleration model
- **Sigma Points**: Deterministically sampled points representing state distribution
- **Advantages**: Better handling of non-linearities, no Jacobian computation required
- **Limitations**: Higher computational cost, more parameters to tune

### Sensor Fusion
- **Fusion Method**: Weighted average of multiple estimates
- **Weight Calculation**: Based on covariance and data freshness
- **Fallback Strategy**: Automatic switching when sensors fail
- **Output**: Combined estimate with conservative covariance

### Map Matching
- **Matching Method**: Grid-based scan correlation
- **Search Strategy**: Local optimization around current pose
- **Validation**: Score-based acceptance with distance limits
- **Correction**: Small adjustments to improve map consistency

## Performance Characteristics

### Computational Requirements
- **EKF**: ~15% CPU at 50Hz
- **UKF**: ~25% CPU at 50Hz
- **Fusion**: ~10% CPU at 20Hz
- **Map Matching**: ~30% CPU at 10Hz

### Accuracy Specifications
- **Position Accuracy**: 5-10cm (with good sensor data)
- **Orientation Accuracy**: 2-5 degrees
- **Velocity Accuracy**: 10-20cm/s
- **Update Rate**: 20-50Hz (configurable)

## Troubleshooting

### Common Issues

1. **Filter divergence**:
   ```bash
   # Check covariance trace
   ros2 topic echo /localization/ekf_health --filter "error_message"
   
   # Reset with known pose
   ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped [...]
   ```

2. **Poor fusion results**:
   ```bash
   # Check individual filter health
   ros2 topic echo /odometry/filtered
   ros2 topic echo /odometry/ukf_filtered
   
   # Adjust fusion weights
   ros2 param set /sensor_fusion_localization ekf_weight 0.8
   ```

3. **Map matching failures**:
   ```bash
   # Check scan match scores
   ros2 topic echo /localization/map_matcher_health
   
   # Verify map and scan data
   ros2 topic echo /map --field info
   ros2 topic echo /scan --field ranges[0:10]
   ```

4. **Transform tree issues**:
   ```bash
   # Check TF tree
   ros2 run tf2_tools view_frames
   
   # Monitor transforms
   ros2 run tf2_ros tf2_echo map odom
   ```

### Performance Optimization
- Reduce filter frequencies for lower CPU usage
- Tune process noise matrices for specific environments
- Adjust fusion weights based on sensor reliability
- Use map matching only when high accuracy is needed

## Integration with Other Packages

### Dependencies
- `tadeo_ecar_msgs`: System health and status messages
- `tadeo_ecar_config`: Localization parameters
- `Eigen3`: Matrix operations for filtering algorithms

### Data Flow
```
Sensors → EKF/UKF → Fusion → Map Matching → Navigation
         ↓         ↓         ↓
       Health    Health    Health → Safety Monitor
```

## Related Packages

- `tadeo_ecar_perception`: Provides sensor data for localization
- `tadeo_ecar_navigation`: Uses localization output for path planning
- `tadeo_ecar_slam`: Builds maps used by map matching
- `tadeo_ecar_safety`: Monitors localization health for safety