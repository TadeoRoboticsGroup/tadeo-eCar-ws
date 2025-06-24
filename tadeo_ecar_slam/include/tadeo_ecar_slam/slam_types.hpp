#ifndef TADEO_ECAR_SLAM_TYPES_HPP_
#define TADEO_ECAR_SLAM_TYPES_HPP_

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace tadeo_ecar_slam
{

struct ScanPoint
{
    double x, y;
    double range, angle;
    double intensity;
    rclcpp::Time timestamp;
};

struct Pose2D
{
    double x, y, theta;
    rclcpp::Time timestamp;
    Eigen::Matrix3d covariance;
    
    Pose2D() : x(0), y(0), theta(0), covariance(Eigen::Matrix3d::Identity()) {}
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_), covariance(Eigen::Matrix3d::Identity()) {}
};

struct GridCell
{
    int8_t occupancy;  // -1: unknown, 0: free, 100: occupied
    double log_odds;
    int hits;
    int misses;
    
    GridCell() : occupancy(-1), log_odds(0.0), hits(0), misses(0) {}
};

struct KeyFrame
{
    int id;
    Pose2D pose;
    std::vector<ScanPoint> scan_points;
    rclcpp::Time timestamp;
    Eigen::MatrixXd descriptor;
    
    KeyFrame() : id(-1) {}
    KeyFrame(int id_, const Pose2D& pose_, const std::vector<ScanPoint>& scan_)
        : id(id_), pose(pose_), scan_points(scan_) {}
};

struct LoopClosure
{
    int keyframe1_id;
    int keyframe2_id;
    Pose2D relative_pose;
    double confidence;
    Eigen::Matrix3d covariance;
    
    LoopClosure() : keyframe1_id(-1), keyframe2_id(-1), confidence(0.0) {}
};

struct GraphEdge
{
    int from_node;
    int to_node;
    Pose2D transform;
    Eigen::Matrix3d information;
    
    GraphEdge() : from_node(-1), to_node(-1) {}
    GraphEdge(int from, int to, const Pose2D& trans) 
        : from_node(from), to_node(to), transform(trans) {}
};

struct GraphNode
{
    int id;
    Pose2D pose;
    std::vector<int> edges;
    bool is_fixed;
    
    GraphNode() : id(-1), is_fixed(false) {}
    GraphNode(int id_, const Pose2D& pose_) : id(id_), pose(pose_), is_fixed(false) {}
};

class OccupancyGrid
{
public:
    OccupancyGrid(double resolution, int width, int height, double origin_x, double origin_y)
        : resolution_(resolution), width_(width), height_(height), 
          origin_x_(origin_x), origin_y_(origin_y)
    {
        grid_.resize(width_ * height_);
    }
    
    GridCell& getCell(int x, int y) {
        return grid_[y * width_ + x];
    }
    
    const GridCell& getCell(int x, int y) const {
        return grid_[y * width_ + x];
    }
    
    bool isValid(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }
    
    void worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const {
        grid_x = static_cast<int>((world_x - origin_x_) / resolution_);
        grid_y = static_cast<int>((world_y - origin_y_) / resolution_);
    }
    
    void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const {
        world_x = origin_x_ + grid_x * resolution_;
        world_y = origin_y_ + grid_y * resolution_;
    }
    
    double getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }
    
private:
    std::vector<GridCell> grid_;
    double resolution_;
    int width_, height_;
    double origin_x_, origin_y_;
};

// Utility functions
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline Pose2D composePoses(const Pose2D& p1, const Pose2D& p2) {
    Pose2D result;
    result.x = p1.x + p2.x * cos(p1.theta) - p2.y * sin(p1.theta);
    result.y = p1.y + p2.x * sin(p1.theta) + p2.y * cos(p1.theta);
    result.theta = normalizeAngle(p1.theta + p2.theta);
    return result;
}

inline Pose2D inversePose(const Pose2D& pose) {
    Pose2D result;
    double cos_theta = cos(pose.theta);
    double sin_theta = sin(pose.theta);
    result.x = -(pose.x * cos_theta + pose.y * sin_theta);
    result.y = -(-pose.x * sin_theta + pose.y * cos_theta);
    result.theta = normalizeAngle(-pose.theta);
    return result;
}

inline double poseDistance(const Pose2D& p1, const Pose2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dtheta = normalizeAngle(p1.theta - p2.theta);
    return sqrt(dx*dx + dy*dy) + 0.5 * abs(dtheta);
}

} // namespace tadeo_ecar_slam

#endif // TADEO_ECAR_SLAM_TYPES_HPP_