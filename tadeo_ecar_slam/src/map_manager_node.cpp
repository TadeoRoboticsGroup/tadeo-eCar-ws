#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/get_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <tadeo_ecar_interfaces/srv/save_map.hpp>
#include "tadeo_ecar_slam/slam_types.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <memory>

namespace tadeo_ecar_slam
{

class MapManagerNode : public rclcpp::Node
{
public:
    MapManagerNode() : Node("map_manager_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        loadParameters();
        initializeMapStorage();
        
        // Subscribers
        grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10,
            std::bind(&MapManagerNode::gridMapCallback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "slam_pose", 10,
            std::bind(&MapManagerNode::poseCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&MapManagerNode::scanCallback, this, std::placeholders::_1));
        
        // Publishers
        merged_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("merged_map", 10);
        map_updates_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_updates", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>("slam/map_manager_health", 10);
        
        // Services
        save_map_service_ = this->create_service<tadeo_ecar_interfaces::srv::SaveMap>(
            "save_merged_map",
            std::bind(&MapManagerNode::saveMapService, this, std::placeholders::_1, std::placeholders::_2));
        
        load_map_service_ = this->create_service<tadeo_ecar_interfaces::srv::SaveMap>(
            "load_map",
            std::bind(&MapManagerNode::loadMapService, this, std::placeholders::_1, std::placeholders::_2));
        
        get_map_service_ = this->create_service<nav_msgs::srv::GetMap>(
            "get_merged_map",
            std::bind(&MapManagerNode::getMapService, this, std::placeholders::_1, std::placeholders::_2));
        
        // Timers
        map_processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / processing_frequency_)),
            std::bind(&MapManagerNode::processingLoop, this));
        
        map_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency_)),
            std::bind(&MapManagerNode::publishLoop, this));
        
        auto_save_timer_ = this->create_wall_timer(
            std::chrono::seconds(auto_save_interval_),
            std::bind(&MapManagerNode::autoSaveLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Map Manager Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("processing_frequency", 2.0);
        this->declare_parameter("publish_frequency", 1.0);
        this->declare_parameter("auto_save_interval", 300); // 5 minutes
        this->declare_parameter("map_storage_path", "/tmp/tadeo_maps/");
        this->declare_parameter("max_stored_maps", 10);
        this->declare_parameter("map_merge_enabled", true);
        this->declare_parameter("quality_threshold", 0.7);
        this->declare_parameter("occupancy_threshold", 50);
        this->declare_parameter("erosion_radius", 1);
        this->declare_parameter("dilation_radius", 1);
        this->declare_parameter("enable_map_filtering", true);
        this->declare_parameter("min_obstacle_size", 5);
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("base_frame", "base_link");
        
        processing_frequency_ = this->get_parameter("processing_frequency").as_double();
        publish_frequency_ = this->get_parameter("publish_frequency").as_double();
        auto_save_interval_ = this->get_parameter("auto_save_interval").as_int();
        map_storage_path_ = this->get_parameter("map_storage_path").as_string();
        max_stored_maps_ = this->get_parameter("max_stored_maps").as_int();
        map_merge_enabled_ = this->get_parameter("map_merge_enabled").as_bool();
        quality_threshold_ = this->get_parameter("quality_threshold").as_double();
        occupancy_threshold_ = this->get_parameter("occupancy_threshold").as_int();
        erosion_radius_ = this->get_parameter("erosion_radius").as_int();
        dilation_radius_ = this->get_parameter("dilation_radius").as_int();
        enable_map_filtering_ = this->get_parameter("enable_map_filtering").as_bool();
        min_obstacle_size_ = this->get_parameter("min_obstacle_size").as_int();
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
    }
    
    void initializeMapStorage()
    {
        // Create storage directory
        try {
            std::filesystem::create_directories(map_storage_path_);
            RCLCPP_INFO(this->get_logger(), "Map storage directory: %s", map_storage_path_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create map storage directory: %s", e.what());
        }
        
        // Initialize map counters
        map_update_count_ = 0;
        last_save_time_ = this->now();
        current_map_received_ = false;
        
        // Load existing maps if available
        loadStoredMaps();
    }
    
    void gridMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_grid_map_ = *msg;
        current_map_received_ = true;
        last_map_time_ = msg->header.stamp;
        map_update_count_++;
        
        RCLCPP_DEBUG(this->get_logger(), "Received grid map update %d", map_update_count_);
    }
    
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        last_pose_time_ = msg->header.stamp;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        current_scan_ = *msg;
        last_scan_time_ = msg->header.stamp;
    }
    
    void processingLoop()
    {
        if (!current_map_received_) return;
        
        // Process current map
        nav_msgs::msg::OccupancyGrid processed_map = current_grid_map_;
        
        if (enable_map_filtering_) {
            processed_map = filterMap(processed_map);
        }
        
        // Merge with stored maps if enabled
        if (map_merge_enabled_ && !stored_maps_.empty()) {
            merged_map_ = mergeWithStoredMaps(processed_map);
        } else {
            merged_map_ = processed_map;
        }
        
        // Update map quality metrics
        updateMapQuality();
        
        RCLCPP_DEBUG(this->get_logger(), "Map processing completed");
    }
    
    nav_msgs::msg::OccupancyGrid filterMap(const nav_msgs::msg::OccupancyGrid& input_map)
    {
        nav_msgs::msg::OccupancyGrid filtered_map = input_map;
        
        int width = input_map.info.width;
        int height = input_map.info.height;
        
        // Apply morphological operations
        if (erosion_radius_ > 0) {
            filtered_map = erodeMap(filtered_map, erosion_radius_);
        }
        
        if (dilation_radius_ > 0) {
            filtered_map = dilateMap(filtered_map, dilation_radius_);
        }
        
        // Remove small obstacles
        if (min_obstacle_size_ > 0) {
            filtered_map = removeSmallObstacles(filtered_map, min_obstacle_size_);
        }
        
        return filtered_map;
    }
    
    nav_msgs::msg::OccupancyGrid erodeMap(const nav_msgs::msg::OccupancyGrid& input_map, int radius)
    {
        nav_msgs::msg::OccupancyGrid eroded_map = input_map;
        int width = input_map.info.width;
        int height = input_map.info.height;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                
                if (input_map.data[index] == 100) { // Occupied cell
                    bool should_erode = false;
                    
                    // Check neighborhood
                    for (int dy = -radius; dy <= radius; ++dy) {
                        for (int dx = -radius; dx <= radius; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                int neighbor_index = ny * width + nx;
                                if (input_map.data[neighbor_index] != 100) {
                                    should_erode = true;
                                    break;
                                }
                            } else {
                                should_erode = true;
                                break;
                            }
                        }
                        if (should_erode) break;
                    }
                    
                    if (should_erode) {
                        eroded_map.data[index] = 0; // Make free
                    }
                }
            }
        }
        
        return eroded_map;
    }
    
    nav_msgs::msg::OccupancyGrid dilateMap(const nav_msgs::msg::OccupancyGrid& input_map, int radius)
    {
        nav_msgs::msg::OccupancyGrid dilated_map = input_map;
        int width = input_map.info.width;
        int height = input_map.info.height;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                
                if (input_map.data[index] == 0) { // Free cell
                    bool should_dilate = false;
                    
                    // Check neighborhood
                    for (int dy = -radius; dy <= radius; ++dy) {
                        for (int dx = -radius; dx <= radius; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                                int neighbor_index = ny * width + nx;
                                if (input_map.data[neighbor_index] == 100) {
                                    should_dilate = true;
                                    break;
                                }
                            }
                        }
                        if (should_dilate) break;
                    }
                    
                    if (should_dilate) {
                        dilated_map.data[index] = 100; // Make occupied
                    }
                }
            }
        }
        
        return dilated_map;
    }
    
    nav_msgs::msg::OccupancyGrid removeSmallObstacles(const nav_msgs::msg::OccupancyGrid& input_map, int min_size)
    {
        nav_msgs::msg::OccupancyGrid cleaned_map = input_map;
        int width = input_map.info.width;
        int height = input_map.info.height;
        
        std::vector<bool> visited(width * height, false);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                
                if (input_map.data[index] == 100 && !visited[index]) {
                    // Found unvisited obstacle, perform flood fill
                    std::vector<std::pair<int, int>> component;
                    std::queue<std::pair<int, int>> queue;
                    
                    queue.push({x, y});
                    visited[index] = true;
                    
                    while (!queue.empty()) {
                        auto [cx, cy] = queue.front();
                        queue.pop();
                        component.push_back({cx, cy});
                        
                        // Check 4-connected neighbors
                        int dx[] = {-1, 1, 0, 0};
                        int dy[] = {0, 0, -1, 1};
                        
                        for (int i = 0; i < 4; ++i) {
                            int nx = cx + dx[i];
                            int ny = cy + dy[i];
                            int neighbor_index = ny * width + nx;
                            
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height &&
                                !visited[neighbor_index] && input_map.data[neighbor_index] == 100) {
                                visited[neighbor_index] = true;
                                queue.push({nx, ny});
                            }
                        }
                    }
                    
                    // Remove component if too small
                    if (component.size() < static_cast<size_t>(min_size)) {
                        for (const auto& [ox, oy] : component) {
                            int obstacle_index = oy * width + ox;
                            cleaned_map.data[obstacle_index] = 0; // Make free
                        }
                    }
                }
            }
        }
        
        return cleaned_map;
    }
    
    nav_msgs::msg::OccupancyGrid mergeWithStoredMaps(const nav_msgs::msg::OccupancyGrid& new_map)
    {
        if (stored_maps_.empty()) {
            return new_map;
        }
        
        // Simple merging: take the most confident value for each cell
        nav_msgs::msg::OccupancyGrid merged = new_map;
        
        for (const auto& stored_map : stored_maps_) {
            if (mapsAreCompatible(merged, stored_map)) {
                mergeTwoMaps(merged, stored_map);
            }
        }
        
        return merged;
    }
    
    bool mapsAreCompatible(const nav_msgs::msg::OccupancyGrid& map1, const nav_msgs::msg::OccupancyGrid& map2)
    {
        const double tolerance = 1e-6;
        
        return (abs(map1.info.resolution - map2.info.resolution) < tolerance) &&
               (map1.info.width == map2.info.width) &&
               (map1.info.height == map2.info.height) &&
               (abs(map1.info.origin.position.x - map2.info.origin.position.x) < tolerance) &&
               (abs(map1.info.origin.position.y - map2.info.origin.position.y) < tolerance);
    }
    
    void mergeTwoMaps(nav_msgs::msg::OccupancyGrid& target, const nav_msgs::msg::OccupancyGrid& source)
    {
        int size = target.info.width * target.info.height;
        
        for (int i = 0; i < size; ++i) {
            int8_t target_val = target.data[i];
            int8_t source_val = source.data[i];
            
            // Merge logic: prefer occupied over free, known over unknown
            if (target_val == -1) { // Unknown in target
                target.data[i] = source_val;
            } else if (source_val == 100 && target_val != 100) { // Obstacle in source
                target.data[i] = 100;
            } else if (source_val == 0 && target_val == -1) { // Free in source, unknown in target
                target.data[i] = 0;
            }
            // Otherwise keep target value
        }
    }
    
    void updateMapQuality()
    {
        if (merged_map_.data.empty()) return;
        
        int total_cells = merged_map_.info.width * merged_map_.info.height;
        int known_cells = 0;
        int occupied_cells = 0;
        
        for (const auto& cell : merged_map_.data) {
            if (cell != -1) {
                known_cells++;
                if (cell > occupancy_threshold_) {
                    occupied_cells++;
                }
            }
        }
        
        map_quality_.coverage = static_cast<double>(known_cells) / total_cells;
        map_quality_.obstacle_density = static_cast<double>(occupied_cells) / total_cells;
        map_quality_.last_update = this->now();
        
        RCLCPP_DEBUG(this->get_logger(), "Map quality - Coverage: %.2f, Obstacles: %.2f",
                     map_quality_.coverage, map_quality_.obstacle_density);
    }
    
    void publishLoop()
    {
        publishMergedMap();
        publishHealthStatus();
    }
    
    void publishMergedMap()
    {
        if (merged_map_.data.empty()) return;
        
        merged_map_.header.stamp = this->now();
        merged_map_.header.frame_id = map_frame_;
        
        merged_map_pub_->publish(merged_map_);
        
        // Publish incremental updates if needed
        if (map_update_count_ > 0) {
            publishMapUpdates();
        }
    }
    
    void publishMapUpdates()
    {
        // For now, just publish the full map as update
        auto update_msg = merged_map_;
        update_msg.header.stamp = this->now();
        map_updates_pub_->publish(update_msg);
    }
    
    void autoSaveLoop()
    {
        if (merged_map_.data.empty()) return;
        
        auto current_time = this->now();
        if (map_quality_.coverage > quality_threshold_) {
            std::string filename = generateMapFilename(current_time);
            saveMapToFile(merged_map_, filename);
            last_save_time_ = current_time;
            
            RCLCPP_INFO(this->get_logger(), "Auto-saved map: %s", filename.c_str());
        }
    }
    
    std::string generateMapFilename(const rclcpp::Time& timestamp)
    {
        auto time_t = timestamp.seconds();
        auto tm = *std::localtime(&time_t);
        
        std::ostringstream oss;
        oss << map_storage_path_ << "map_" 
            << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".yaml";
        
        return oss.str();
    }
    
    void saveMapToFile(const nav_msgs::msg::OccupancyGrid& map, const std::string& filename)
    {
        try {
            // Save map data as PGM file
            std::string pgm_filename = filename.substr(0, filename.find_last_of('.')) + ".pgm";
            std::ofstream pgm_file(pgm_filename, std::ios::binary);
            
            if (!pgm_file.is_open()) {
                throw std::runtime_error("Cannot open PGM file for writing");
            }
            
            // Write PGM header
            pgm_file << "P5\n";
            pgm_file << map.info.width << " " << map.info.height << "\n";
            pgm_file << "255\n";
            
            // Write map data
            for (int y = map.info.height - 1; y >= 0; y--) { // Flip Y axis
                for (int x = 0; x < static_cast<int>(map.info.width); x++) {
                    int index = y * map.info.width + x;
                    int8_t cell = map.data[index];
                    
                    uint8_t pixel;
                    if (cell == -1) {
                        pixel = 205; // Unknown (gray)
                    } else if (cell == 0) {
                        pixel = 255; // Free (white)
                    } else {
                        pixel = 0;   // Occupied (black)
                    }
                    
                    pgm_file.write(reinterpret_cast<const char*>(&pixel), 1);
                }
            }
            
            pgm_file.close();
            
            // Save YAML metadata
            YAML::Node yaml_node;
            yaml_node["image"] = pgm_filename.substr(pgm_filename.find_last_of('/') + 1);
            yaml_node["resolution"] = map.info.resolution;
            yaml_node["origin"][0] = map.info.origin.position.x;
            yaml_node["origin"][1] = map.info.origin.position.y;
            yaml_node["origin"][2] = 0.0;
            yaml_node["negate"] = 0;
            yaml_node["occupied_thresh"] = 0.65;
            yaml_node["free_thresh"] = 0.196;
            
            std::ofstream yaml_file(filename);
            yaml_file << yaml_node;
            yaml_file.close();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", e.what());
            throw;
        }
    }
    
    void loadStoredMaps()
    {
        try {
            if (!std::filesystem::exists(map_storage_path_)) {
                return;
            }
            
            for (const auto& entry : std::filesystem::directory_iterator(map_storage_path_)) {
                if (entry.path().extension() == ".yaml") {
                    nav_msgs::msg::OccupancyGrid loaded_map;
                    if (loadMapFromFile(entry.path().string(), loaded_map)) {
                        stored_maps_.push_back(loaded_map);
                        RCLCPP_INFO(this->get_logger(), "Loaded stored map: %s", 
                                    entry.path().filename().c_str());
                    }
                }
            }
            
            // Keep only the most recent maps
            if (stored_maps_.size() > static_cast<size_t>(max_stored_maps_)) {
                stored_maps_.erase(stored_maps_.begin(), 
                                   stored_maps_.end() - max_stored_maps_);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load stored maps: %s", e.what());
        }
    }
    
    bool loadMapFromFile(const std::string& filename, nav_msgs::msg::OccupancyGrid& map)
    {
        try {
            YAML::Node yaml_node = YAML::LoadFile(filename);
            
            std::string image_file = yaml_node["image"].as<std::string>();
            std::string pgm_path = std::filesystem::path(filename).parent_path() / image_file;
            
            // Load PGM file
            std::ifstream pgm_file(pgm_path, std::ios::binary);
            if (!pgm_file.is_open()) {
                return false;
            }
            
            std::string format;
            int width, height, max_val;
            pgm_file >> format >> width >> height >> max_val;
            pgm_file.ignore(); // Skip newline
            
            if (format != "P5") {
                return false;
            }
            
            // Set map info
            map.info.resolution = yaml_node["resolution"].as<double>();
            map.info.width = width;
            map.info.height = height;
            map.info.origin.position.x = yaml_node["origin"][0].as<double>();
            map.info.origin.position.y = yaml_node["origin"][1].as<double>();
            map.info.origin.position.z = 0.0;
            map.info.origin.orientation.w = 1.0;
            
            // Load map data
            map.data.resize(width * height);
            for (int y = height - 1; y >= 0; y--) { // Flip Y axis
                for (int x = 0; x < width; x++) {
                    uint8_t pixel;
                    pgm_file.read(reinterpret_cast<char*>(&pixel), 1);
                    
                    int index = y * width + x;
                    if (pixel >= 250) {
                        map.data[index] = 0;   // Free
                    } else if (pixel <= 10) {
                        map.data[index] = 100; // Occupied
                    } else {
                        map.data[index] = -1;  // Unknown
                    }
                }
            }
            
            pgm_file.close();
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map from %s: %s", filename.c_str(), e.what());
            return false;
        }
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = base_frame_;
        
        // Remove component_name field - not part of SystemHealth message
        
        // Check system health
        auto current_time = this->now();
        bool map_timeout = current_map_received_ && (current_time - last_map_time_).seconds() > 5.0;
        
        if (map_timeout) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(9001);
            health_msg.error_messages.push_back("Map data timeout");
        } else if (!current_map_received_) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(9002);
            health_msg.error_messages.push_back("Waiting for map data");
        } else if (map_quality_.coverage < quality_threshold_) {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(9003);
            health_msg.error_messages.push_back("Low map quality");
        } else {
            // Use appropriate status enum instead of string status
            health_msg.error_codes.push_back(0);
            health_msg.error_messages.push_back("");
        }
        
        // Replace with cpu_temperature = 20.0; // See SystemHealth.msg // Placeholder
        // Replace with appropriate field - memory_usage not in SystemHealth.msg // Placeholder
        // Replace with specific temperature fields: cpu_temperature, gpu_temperature, motor_temperature // Placeholder
        
        health_pub_->publish(health_msg);
    }
    
    void saveMapService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::SaveMap::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::SaveMap::Response> response)
    {
        try {
            if (merged_map_.data.empty()) {
                response->success = false;
                response->message = "No map available to save";
                return;
            }
            
            std::string full_path = map_storage_path_ + request->filename;
            if (!full_path.ends_with(".yaml")) {
                full_path += ".yaml";
            }
            
            saveMapToFile(merged_map_, full_path);
            
            response->success = true;
            response->message = "Map saved to " + full_path;
            
            RCLCPP_INFO(this->get_logger(), "Map saved: %s", full_path.c_str());
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to save map: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Map save failed: %s", e.what());
        }
    }
    
    void loadMapService(
        const std::shared_ptr<tadeo_ecar_interfaces::srv::SaveMap::Request> request,
        std::shared_ptr<tadeo_ecar_interfaces::srv::SaveMap::Response> response)
    {
        try {
            std::string full_path = map_storage_path_ + request->filename;
            if (!full_path.ends_with(".yaml")) {
                full_path += ".yaml";
            }
            
            nav_msgs::msg::OccupancyGrid loaded_map;
            if (loadMapFromFile(full_path, loaded_map)) {
                stored_maps_.push_back(loaded_map);
                
                response->success = true;
                response->message = "Map loaded from " + full_path;
                
                RCLCPP_INFO(this->get_logger(), "Map loaded: %s", full_path.c_str());
            } else {
                response->success = false;
                response->message = "Failed to load map from " + full_path;
            }
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to load map: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Map load failed: %s", e.what());
        }
    }
    
    void getMapService(
        const std::shared_ptr<nav_msgs::srv::GetMap::Request> /* request */,
        std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
    {
        if (!merged_map_.data.empty()) {
            response->map = merged_map_;
            response->map.header.stamp = this->now();
        } else {
            RCLCPP_WARN(this->get_logger(), "No map available for GetMap service");
        }
    }
    
    // Map quality metrics
    struct MapQuality {
        double coverage = 0.0;
        double obstacle_density = 0.0;
        rclcpp::Time last_update;
    };
    
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_updates_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    
    rclcpp::Service<tadeo_ecar_interfaces::srv::SaveMap>::SharedPtr save_map_service_;
    rclcpp::Service<tadeo_ecar_interfaces::srv::SaveMap>::SharedPtr load_map_service_;
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr get_map_service_;
    
    rclcpp::TimerBase::SharedPtr map_processing_timer_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;
    rclcpp::TimerBase::SharedPtr auto_save_timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    double processing_frequency_;
    double publish_frequency_;
    int auto_save_interval_;
    std::string map_storage_path_;
    int max_stored_maps_;
    bool map_merge_enabled_;
    double quality_threshold_;
    int occupancy_threshold_;
    int erosion_radius_;
    int dilation_radius_;
    bool enable_map_filtering_;
    int min_obstacle_size_;
    std::string map_frame_;
    std::string base_frame_;
    
    // Map storage
    nav_msgs::msg::OccupancyGrid current_grid_map_;
    nav_msgs::msg::OccupancyGrid merged_map_;
    std::vector<nav_msgs::msg::OccupancyGrid> stored_maps_;
    MapQuality map_quality_;
    
    // State
    bool current_map_received_;
    int map_update_count_;
    rclcpp::Time last_save_time_;
    rclcpp::Time last_map_time_;
    rclcpp::Time last_pose_time_;
    rclcpp::Time last_scan_time_;
    
    // Current sensor data
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
    sensor_msgs::msg::LaserScan current_scan_;
};

} // namespace tadeo_ecar_slam

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_slam::MapManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}