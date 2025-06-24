# Navigation system state message
std_msgs/Header header

# Navigation mode
uint8 IDLE = 0
uint8 PLANNING = 1
uint8 EXECUTING = 2
uint8 PAUSED = 3
uint8 COMPLETED = 4
uint8 FAILED = 5
uint8 navigation_mode

# Current goal
geometry_msgs/PoseStamped current_goal
float32 distance_to_goal
float32 estimated_time_to_goal

# Path information
geometry_msgs/PoseStamped[] current_path
uint32 current_waypoint_index
float32 path_completion_percentage

# Localization quality
float32 localization_confidence
float32 map_match_quality
bool localization_stable

# Planning status
bool path_available
bool path_valid
string planner_status
uint32 replanning_count

# Navigation metrics
float32 average_speed
float32 path_length
float32 deviation_from_path
uint64 navigation_start_time

# Obstacles and constraints
uint32 detected_obstacles_count
bool dynamic_obstacles_present
bool static_obstacles_present
