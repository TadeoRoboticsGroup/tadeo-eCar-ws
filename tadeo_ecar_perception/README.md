# Tadeo eCar Perception Package

This package provides comprehensive sensor processing and perception capabilities for the Tadeo eCar autonomous robot. It integrates data from multiple sensors to create a unified understanding of the robot's environment.

## Package Overview

The `tadeo_ecar_perception` package provides:
- Camera image processing and object detection
- LiDAR point cloud processing and obstacle detection
- IMU data filtering and orientation estimation
- Multi-sensor data fusion for robust perception
- Real-time health monitoring of all sensors

## Directory Structure

```
tadeo_ecar_perception/
├── include/tadeo_ecar_perception/
├── src/
│   ├── camera_processor_node.cpp       # Camera processing and computer vision
│   ├── lidar_processor_node.cpp        # LiDAR data processing and clustering
│   ├── imu_processor_node.cpp          # IMU filtering and orientation
│   └── sensor_fusion_node.cpp          # Multi-sensor data fusion
├── launch/
│   └── perception_system.launch.py     # Complete perception system launch
├── config/
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Nodes

### 1. Camera Processor Node (`camera_processor_node`)

Processes camera images for object detection and lane detection.

**Subscribed Topics:**
- `camera/image_raw` (sensor_msgs/Image): Raw camera images

**Published Topics:**
- `camera/processed_image` (sensor_msgs/Image): Processed images with annotations
- `camera/compressed_processed` (sensor_msgs/CompressedImage): Compressed processed images
- `perception/detected_objects` (visualization_msgs/MarkerArray): Detected objects as 3D markers
- `perception/lane_center` (geometry_msgs/PointStamped): Detected lane center point
- `perception/camera_health` (tadeo_ecar_msgs/SystemHealth): Camera system health

**Key Features:**
- Pedestrian detection using HOG descriptors
- Color-based object detection (traffic cones, signs)
- Face detection using Haar cascades
- Lane detection and center calculation
- Distance estimation for detected objects
- Real-time image processing with OpenCV

### 2. LiDAR Processor Node (`lidar_processor_node`)

Processes LiDAR data for obstacle detection and environment mapping.

**Subscribed Topics:**
- `scan` (sensor_msgs/LaserScan): 2D laser scan data
- `points` (sensor_msgs/PointCloud2): 3D point cloud data

**Published Topics:**
- `scan_filtered` (sensor_msgs/LaserScan): Filtered laser scan data
- `points_filtered` (sensor_msgs/PointCloud2): Filtered point cloud data
- `perception/obstacles` (visualization_msgs/MarkerArray): Detected obstacles
- `perception/free_space` (geometry_msgs/PolygonStamped): Navigable free space
- `perception/closest_obstacle` (geometry_msgs/PointStamped): Nearest obstacle location
- `perception/lidar_health` (tadeo_ecar_msgs/SystemHealth): LiDAR system health

**Key Features:**
- Point cloud filtering (voxel grid, statistical outlier removal)
- Ground plane removal using RANSAC
- Euclidean clustering for obstacle detection
- Free space calculation
- Obstacle classification based on geometry
- Range and angle filtering

### 3. IMU Processor Node (`imu_processor_node`)

Processes IMU data for orientation estimation and motion analysis.

**Subscribed Topics:**
- `imu/data_raw` (sensor_msgs/Imu): Raw IMU measurements

**Published Topics:**
- `imu/data` (sensor_msgs/Imu): Filtered and calibrated IMU data
- `imu/euler_angles` (geometry_msgs/Vector3Stamped): Roll, pitch, yaw angles
- `imu/linear_acceleration` (geometry_msgs/Vector3Stamped): Linear acceleration vector
- `imu/angular_velocity` (geometry_msgs/Vector3Stamped): Angular velocity vector
- `imu/roll` (std_msgs/Float64): Roll angle
- `imu/pitch` (std_msgs/Float64): Pitch angle
- `imu/yaw` (std_msgs/Float64): Yaw angle
- `imu/lateral_acceleration` (std_msgs/Float64): Lateral acceleration
- `imu/longitudinal_acceleration` (std_msgs/Float64): Longitudinal acceleration
- `imu/tilt_angle` (std_msgs/Float64): Overall tilt magnitude
- `perception/imu_health` (tadeo_ecar_msgs/SystemHealth): IMU system health

**Key Features:**
- Automatic bias calibration on startup
- Low-pass and high-pass filtering options
- Orientation estimation through integration
- Stability monitoring and vibration detection
- Gravity compensation
- Real-time data validation

### 4. Sensor Fusion Node (`sensor_fusion_node`)

Fuses data from multiple sensors for robust environment perception.

**Subscribed Topics:**
- `imu/data` (sensor_msgs/Imu): Processed IMU data
- `scan` (sensor_msgs/LaserScan): LiDAR scan data
- `camera/image_raw` (sensor_msgs/Image): Camera images
- `perception/obstacles` (visualization_msgs/MarkerArray): LiDAR obstacles
- `perception/detected_objects` (visualization_msgs/MarkerArray): Camera objects

**Published Topics:**
- `perception/fused_obstacles` (visualization_msgs/MarkerArray): Fused obstacle tracks
- `perception/nearest_obstacle` (geometry_msgs/PointStamped): Nearest fused obstacle
- `perception/fusion_health` (tadeo_ecar_msgs/SystemHealth): Fusion system health
- `perception/confidence_map` (visualization_msgs/MarkerArray): Sensor confidence visualization

**Key Features:**
- Multi-sensor data association and tracking
- Confidence-based obstacle fusion
- Track management with unique IDs
- Sensor health monitoring
- Weighted fusion based on sensor reliability
- Expired track removal

## Usage Examples

### Basic Launch
```bash
# Launch complete perception system
ros2 launch tadeo_ecar_perception perception_system.launch.py

# Launch with specific sensors enabled
ros2 launch tadeo_ecar_perception perception_system.launch.py enable_camera:=true enable_lidar:=true enable_imu:=true
```

### Individual Node Launch
```bash
# Launch camera processor only
ros2 run tadeo_ecar_perception camera_processor_node

# Launch LiDAR processor with parameters
ros2 run tadeo_ecar_perception lidar_processor_node --ros-args -p filtering.min_range:=0.2

# Launch IMU processor
ros2 run tadeo_ecar_perception imu_processor_node

# Launch sensor fusion
ros2 run tadeo_ecar_perception sensor_fusion_node
```

### Monitoring Perception Data
```bash
# View detected objects
ros2 topic echo /perception/detected_objects

# View LiDAR obstacles
ros2 topic echo /perception/obstacles

# View fused obstacles
ros2 topic echo /perception/fused_obstacles

# Monitor IMU orientation
ros2 topic echo /imu/euler_angles

# View processed camera feed
ros2 run rqt_image_view rqt_image_view /camera/processed_image
```

### Sensor Health Monitoring
```bash
# Monitor all sensor health
ros2 topic echo /perception/camera_health
ros2 topic echo /perception/lidar_health
ros2 topic echo /perception/imu_health
ros2 topic echo /perception/fusion_health

# Check sensor data rates
ros2 topic hz /camera/image_raw
ros2 topic hz /scan
ros2 topic hz /imu/data_raw
```

## Configuration

### Camera Processing Parameters
```yaml
processing:
  frequency: 10.0
  enable_object_detection: true
  enable_lane_detection: true
  min_object_confidence: 0.5

camera:
  image_width: 640
  image_height: 480
  focal_length: 500.0
  mount_height: 0.5
```

### LiDAR Processing Parameters
```yaml
filtering:
  min_range: 0.1
  max_range: 30.0
  voxel_size: 0.05

clustering:
  min_cluster_size: 10
  max_cluster_size: 500
  cluster_tolerance: 0.2

obstacle:
  min_height: 0.1
  max_height: 3.0
  min_radius: 0.05
  max_radius: 2.0
```

### IMU Processing Parameters
```yaml
filtering:
  enable_calibration: true
  calibration_samples: 1000
  enable_lowpass: true
  lowpass_alpha: 0.8
  enable_highpass: true
  highpass_alpha: 0.95

thresholds:
  max_acceleration: 20.0
  max_angular_velocity: 10.0
  stability_threshold: 0.2
```

### Sensor Fusion Parameters
```yaml
fusion:
  frequency: 20.0
  max_association_distance: 1.0
  obstacle_timeout: 2.0
  min_confidence: 0.3
  track_threshold: 0.7

weights:
  imu: 1.0
  lidar: 0.8
  camera: 0.6
```

## Advanced Features

### Computer Vision Capabilities
- **Object Detection**: HOG-based pedestrian detection, color-based object detection
- **Lane Detection**: Edge detection and Hough line transforms
- **Face Detection**: Haar cascade classifiers
- **Distance Estimation**: Monocular depth estimation using object size

### LiDAR Processing
- **Point Cloud Filtering**: Voxel grid downsampling, statistical outlier removal
- **Segmentation**: Ground plane removal, Euclidean clustering
- **Obstacle Classification**: Geometry-based type determination
- **Free Space Mapping**: Navigable area calculation

### IMU Processing
- **Sensor Fusion**: Accelerometer and gyroscope integration
- **Calibration**: Automatic bias correction and gravity compensation
- **Filtering**: Configurable low-pass and high-pass filters
- **Stability Analysis**: Vibration detection and stability metrics

### Multi-Sensor Fusion
- **Data Association**: Spatial and temporal obstacle matching
- **Track Management**: Persistent obstacle tracking with unique IDs
- **Confidence Weighting**: Sensor reliability-based fusion
- **Health Monitoring**: Real-time sensor status assessment

## Safety Features

### Obstacle Detection
```bash
# Monitor closest obstacles
ros2 topic echo /perception/closest_obstacle
ros2 topic echo /perception/nearest_obstacle

# Emergency obstacle alerts
ros2 topic echo /perception/fused_obstacles --filter "markers[0].color.r > 0.8"
```

### Sensor Validation
- Automatic data validation and outlier rejection
- Sensor timeout detection and fallback modes
- Cross-sensor consistency checking
- Health status reporting for all components

## Integration with Other Packages

### Dependencies
- `tadeo_ecar_msgs`: Custom message types for robot status
- `tadeo_ecar_config`: Sensor parameters and configurations
- `OpenCV`: Computer vision processing
- `PCL`: Point cloud processing

### Interfaces
- **Input**: Raw sensor data from hardware or simulation
- **Output**: Processed perception data for navigation and control
- **Health**: System status information for monitoring

## Troubleshooting

### Common Issues

1. **Camera not working**:
   ```bash
   # Check camera device
   ls /dev/video*
   
   # Test camera feed
   ros2 topic echo /camera/image_raw
   ```

2. **LiDAR data missing**:
   ```bash
   # Check LiDAR connection
   ros2 topic list | grep scan
   
   # Monitor data rate
   ros2 topic hz /scan
   ```

3. **IMU calibration issues**:
   ```bash
   # Monitor calibration status
   ros2 topic echo /perception/imu_health
   
   # Restart calibration
   ros2 service call /imu_calibrate std_srvs/srv/Empty
   ```

4. **Poor fusion results**:
   ```bash
   # Check sensor health
   ros2 topic echo /perception/fusion_health
   
   # Monitor confidence levels
   ros2 topic echo /perception/confidence_map
   ```

### Performance Optimization
- Adjust processing frequencies based on computational resources
- Tune filter parameters for specific environments
- Configure sensor weights based on reliability
- Use compressed image topics when bandwidth is limited

## Related Packages

- `tadeo_ecar_control`: Uses perception data for obstacle avoidance
- `tadeo_ecar_navigation`: Integrates perception for path planning
- `tadeo_ecar_safety`: Monitors perception health for safety systems
- `tadeo_ecar_slam`: Uses perception data for mapping and localization