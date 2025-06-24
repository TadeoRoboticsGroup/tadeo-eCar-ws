# Robot operational status message
std_msgs/Header header

# Robot identification
string robot_id
string robot_name

# Operational state
uint8 IDLE = 0
uint8 MANUAL = 1
uint8 AUTONOMOUS = 2
uint8 EMERGENCY = 3
uint8 MAINTENANCE = 4
uint8 operational_state

# System status
bool is_ready
bool is_moving
bool is_charging
bool emergency_stop_active

# Battery information
float32 battery_voltage
float32 battery_percentage
float32 battery_temperature

# Performance metrics
float32 cpu_usage
float32 memory_usage
float32 network_quality
