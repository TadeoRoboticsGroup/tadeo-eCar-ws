#ifndef TADEO_ECAR_SAFETY_TYPES_HPP_
#define TADEO_ECAR_SAFETY_TYPES_HPP_

#include <string>
#include <vector>
#include <chrono>
#include <geometry_msgs/msg/point.hpp>

namespace tadeo_ecar_safety
{

enum class SafetyLevel
{
    SAFE = 0,
    WARNING = 1,
    CRITICAL = 2,
    EMERGENCY = 3
};

enum class EmergencyType
{
    NONE = 0,
    COLLISION_IMMINENT = 1,
    SENSOR_FAILURE = 2,
    COMMUNICATION_LOSS = 3,
    SYSTEM_OVERLOAD = 4,
    USER_EMERGENCY = 5,
    HARDWARE_FAULT = 6,
    STABILITY_LOSS = 7,
    BATTERY_CRITICAL = 8
};

struct SafetyZone
{
    std::string name;
    double min_distance;
    double max_distance;
    double angular_range;
    SafetyLevel level;
    bool is_active;
};

struct EmergencyCondition
{
    EmergencyType type;
    std::string description;
    SafetyLevel severity;
    std::chrono::steady_clock::time_point timestamp;
    bool is_active;
    std::string source_component;
};

struct SafetyLimits
{
    double max_linear_velocity;
    double max_angular_velocity;
    double max_acceleration;
    double max_deceleration;
    double min_obstacle_distance;
    double max_tilt_angle;
    double max_lateral_acceleration;
};

struct CollisionPrediction
{
    bool collision_predicted;
    double time_to_collision;
    double collision_distance;
    std::string obstacle_type;
    geometry_msgs::msg::Point collision_point;
    double confidence;
};

} // namespace tadeo_ecar_safety

#endif // TADEO_ECAR_SAFETY_TYPES_HPP_