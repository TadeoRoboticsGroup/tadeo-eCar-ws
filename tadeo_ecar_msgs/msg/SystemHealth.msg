# System health monitoring message
std_msgs/Header header

# Component health status
uint8 HEALTHY = 0
uint8 WARNING = 1
uint8 ERROR = 2
uint8 CRITICAL = 3

# Core systems
uint8 cpu_status
uint8 memory_status
uint8 storage_status
uint8 network_status

# Hardware components
uint8 lidar_status
uint8 camera_status
uint8 imu_status
uint8 gps_status

# Motor controllers
uint8 front_left_motor_status
uint8 front_right_motor_status
uint8 rear_left_motor_status
uint8 rear_right_motor_status

# Temperatures (Celsius)
float32 cpu_temperature
float32 gpu_temperature
float32 motor_temperature

# Error codes and messages
uint32[] error_codes
string[] error_messages

# Diagnostic information
string diagnostic_info
uint64 uptime_seconds
