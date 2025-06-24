# Safety system status message
std_msgs/Header header

# Safety state
uint8 SAFE = 0
uint8 WARNING = 1
uint8 DANGER = 2
uint8 EMERGENCY = 3
uint8 safety_state

# Emergency stops
bool hardware_estop_active
bool software_estop_active
bool remote_estop_active

# Collision detection
bool collision_detected
bool obstacle_in_path
float32 min_obstacle_distance

# Speed limits
float32 current_max_speed
float32 safety_max_speed
bool speed_limit_active

# Sensor monitoring
bool lidar_operational
bool camera_operational
bool imu_operational

# Safety zones
bool in_safe_zone
bool approaching_boundary
string current_zone

# Watchdog timers
bool communication_timeout
bool control_timeout
bool sensor_timeout

# Safety actions
bool automatic_braking_active
bool speed_reduction_active
bool path_replanning_required
