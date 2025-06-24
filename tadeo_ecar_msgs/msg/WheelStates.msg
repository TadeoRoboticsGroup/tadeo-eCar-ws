# 4WD4WS wheel states message
std_msgs/Header header

# Individual wheel velocities (rad/s)
float32 front_left_velocity
float32 front_right_velocity
float32 rear_left_velocity
float32 rear_right_velocity

# Individual wheel steering angles (rad)
float32 front_left_steering
float32 front_right_steering
float32 rear_left_steering
float32 rear_right_steering

# Individual wheel torques (Nm)
float32 front_left_torque
float32 front_right_torque
float32 rear_left_torque
float32 rear_right_torque

# Wheel slip indicators
bool front_left_slip
bool front_right_slip
bool rear_left_slip
bool rear_right_slip

# Encoder positions (rad)
float32 front_left_position
float32 front_right_position
float32 rear_left_position
float32 rear_right_position
