#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <tadeo_ecar_msgs/msg/system_health.hpp>
#include <vector>
#include <memory>

namespace tadeo_ecar_perception
{

struct DetectedObject
{
    cv::Rect bbox;
    std::string class_name;
    float confidence;
    cv::Point2f center;
    double distance;
};

class CameraProcessorNode : public rclcpp::Node
{
public:
    CameraProcessorNode() : Node("camera_processor_node"), it_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}))
    {
        loadParameters();
        initializeDetectors();
        
        // Image subscriber
        image_sub_ = it_.subscribe("camera/image_raw", 1, 
            std::bind(&CameraProcessorNode::imageCallback, this, std::placeholders::_1));
        
        // Publishers
        processed_image_pub_ = it_.advertise("camera/processed_image", 1);
        compressed_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "camera/compressed_processed", 10);
        objects_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "perception/detected_objects", 10);
        lane_points_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "perception/lane_center", 10);
        health_pub_ = this->create_publisher<tadeo_ecar_msgs::msg::SystemHealth>(
            "perception/camera_health", 10);
        
        // Processing timer
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / processing_frequency_)),
            std::bind(&CameraProcessorNode::processingLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Camera Processor Node initialized");
    }

private:
    void loadParameters()
    {
        this->declare_parameter("processing.frequency", 10.0);
        this->declare_parameter("processing.enable_object_detection", true);
        this->declare_parameter("processing.enable_lane_detection", true);
        this->declare_parameter("processing.min_object_confidence", 0.5);
        this->declare_parameter("camera.image_width", 640);
        this->declare_parameter("camera.image_height", 480);
        this->declare_parameter("camera.focal_length", 500.0);
        this->declare_parameter("camera.mount_height", 0.5);
        
        processing_frequency_ = this->get_parameter("processing.frequency").as_double();
        enable_object_detection_ = this->get_parameter("processing.enable_object_detection").as_bool();
        enable_lane_detection_ = this->get_parameter("processing.enable_lane_detection").as_bool();
        min_confidence_ = this->get_parameter("processing.min_object_confidence").as_double();
        image_width_ = this->get_parameter("camera.image_width").as_int();
        image_height_ = this->get_parameter("camera.image_height").as_int();
        focal_length_ = this->get_parameter("camera.focal_length").as_double();
        camera_height_ = this->get_parameter("camera.mount_height").as_double();
    }
    
    void initializeDetectors()
    {
        // Initialize HOG descriptor for pedestrian detection
        hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        
        // Initialize cascade classifiers
        try {
            // Load Haar cascades (these would need to be provided)
            face_cascade_.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml");
            RCLCPP_INFO(this->get_logger(), "Face cascade loaded successfully");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Could not load face cascade: %s", e.what());
        }
        
        // Initialize color ranges for object detection
        initializeColorRanges();
    }
    
    void initializeColorRanges()
    {
        // Define HSV color ranges for different objects
        // Red objects (stop signs, traffic cones)
        red_lower1_ = cv::Scalar(0, 50, 50);
        red_upper1_ = cv::Scalar(10, 255, 255);
        red_lower2_ = cv::Scalar(170, 50, 50);
        red_upper2_ = cv::Scalar(180, 255, 255);
        
        // Yellow objects (traffic signs, construction)
        yellow_lower_ = cv::Scalar(20, 100, 100);
        yellow_upper_ = cv::Scalar(30, 255, 255);
        
        // Green objects (go signals)
        green_lower_ = cv::Scalar(40, 50, 50);
        green_upper_ = cv::Scalar(80, 255, 255);
        
        // White/Gray for lane detection
        white_lower_ = cv::Scalar(0, 0, 200);
        white_upper_ = cv::Scalar(180, 30, 255);
    }
    
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            // Convert ROS image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            current_image_ = cv_ptr->image.clone();
            image_header_ = msg->header;
            
            // Process immediately for real-time performance
            processImage();
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    void processImage()
    {
        if (current_image_.empty()) return;
        
        cv::Mat processed_image = current_image_.clone();
        std::vector<DetectedObject> detected_objects;
        
        // Object detection
        if (enable_object_detection_) {
            detectPedestrians(processed_image, detected_objects);
            detectColoredObjects(processed_image, detected_objects);
            detectFaces(processed_image, detected_objects);
        }
        
        // Lane detection
        cv::Point2f lane_center(0, 0);
        if (enable_lane_detection_) {
            lane_center = detectLanes(processed_image);
        }
        
        // Draw results
        drawDetections(processed_image, detected_objects);
        
        // Publish results
        publishProcessedImage(processed_image);
        publishDetectedObjects(detected_objects);
        publishLaneCenter(lane_center);
    }
    
    void detectPedestrians(cv::Mat& image, std::vector<DetectedObject>& objects)
    {
        std::vector<cv::Rect> detections;
        std::vector<double> weights;
        
        // Convert to grayscale for HOG
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        // Detect pedestrians
        hog_.detectMultiScale(gray, detections, weights, 0.0, cv::Size(8,8), 
                              cv::Size(32,32), 1.05, 2);
        
        for (size_t i = 0; i < detections.size(); ++i) {
            if (weights[i] > min_confidence_) {
                DetectedObject obj;
                obj.bbox = detections[i];
                obj.class_name = "pedestrian";
                obj.confidence = weights[i];
                obj.center = cv::Point2f(obj.bbox.x + obj.bbox.width/2.0f, 
                                         obj.bbox.y + obj.bbox.height/2.0f);
                obj.distance = estimateDistance(obj.bbox.height);
                
                objects.push_back(obj);
            }
        }
    }
    
    void detectColoredObjects(cv::Mat& image, std::vector<DetectedObject>& objects)
    {
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        // Detect red objects
        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv, red_lower1_, red_upper1_, red_mask1);
        cv::inRange(hsv, red_lower2_, red_upper2_, red_mask2);
        cv::bitwise_or(red_mask1, red_mask2, red_mask);
        
        detectObjectsInMask(red_mask, "traffic_cone", objects);
        
        // Detect yellow objects
        cv::Mat yellow_mask;
        cv::inRange(hsv, yellow_lower_, yellow_upper_, yellow_mask);
        detectObjectsInMask(yellow_mask, "warning_sign", objects);
    }
    
    void detectObjectsInMask(const cv::Mat& mask, const std::string& class_name, 
                             std::vector<DetectedObject>& objects)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 500) { // Minimum area threshold
                cv::Rect bbox = cv::boundingRect(contour);
                
                DetectedObject obj;
                obj.bbox = bbox;
                obj.class_name = class_name;
                obj.confidence = std::min(1.0, area / 5000.0); // Simple confidence based on area
                obj.center = cv::Point2f(bbox.x + bbox.width/2.0f, bbox.y + bbox.height/2.0f);
                obj.distance = estimateDistance(bbox.height);
                
                objects.push_back(obj);
            }
        }
    }
    
    void detectFaces(cv::Mat& image, std::vector<DetectedObject>& objects)
    {
        if (face_cascade_.empty()) return;
        
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        std::vector<cv::Rect> faces;
        face_cascade_.detectMultiScale(gray, faces, 1.1, 3, 0, cv::Size(30, 30));
        
        for (const auto& face : faces) {
            DetectedObject obj;
            obj.bbox = face;
            obj.class_name = "person";
            obj.confidence = 0.8; // Fixed confidence for cascade detection
            obj.center = cv::Point2f(face.x + face.width/2.0f, face.y + face.height/2.0f);
            obj.distance = estimateDistance(face.height);
            
            objects.push_back(obj);
        }
    }
    
    cv::Point2f detectLanes(cv::Mat& image)
    {
        cv::Mat hsv, mask, edges;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        // Create mask for white/yellow lane markings
        cv::Mat white_mask, yellow_mask;
        cv::inRange(hsv, white_lower_, white_upper_, white_mask);
        cv::inRange(hsv, yellow_lower_, yellow_upper_, yellow_mask);
        cv::bitwise_or(white_mask, yellow_mask, mask);
        
        // Apply region of interest (lower half of image)
        cv::Mat roi_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        cv::rectangle(roi_mask, cv::Point(0, image.rows/2), 
                      cv::Point(image.cols, image.rows), cv::Scalar(255), -1);
        cv::bitwise_and(mask, roi_mask, mask);
        
        // Edge detection
        cv::Canny(mask, edges, 50, 150);
        
        // Hough line detection
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edges, lines, 1, CV_PI/180, 50, 50, 10);
        
        // Find lane center
        cv::Point2f lane_center(image.cols / 2.0f, image.rows * 0.8f);
        
        if (!lines.empty()) {
            // Separate left and right lanes
            std::vector<cv::Vec4i> left_lines, right_lines;
            for (const auto& line : lines) {
                cv::Point2f center((line[0] + line[2]) / 2.0f, (line[1] + line[3]) / 2.0f);
                if (center.x < image.cols / 2) {
                    left_lines.push_back(line);
                } else {
                    right_lines.push_back(line);
                }
            }
            
            // Calculate average lane center
            if (!left_lines.empty() || !right_lines.empty()) {
                float left_x = 0, right_x = image.cols;
                
                if (!left_lines.empty()) {
                    float sum_x = 0;
                    for (const auto& line : left_lines) {
                        sum_x += (line[0] + line[2]) / 2.0f;
                    }
                    left_x = sum_x / left_lines.size();
                }
                
                if (!right_lines.empty()) {
                    float sum_x = 0;
                    for (const auto& line : right_lines) {
                        sum_x += (line[0] + line[2]) / 2.0f;
                    }
                    right_x = sum_x / right_lines.size();
                }
                
                lane_center.x = (left_x + right_x) / 2.0f;
            }
        }
        
        return lane_center;
    }
    
    double estimateDistance(int object_height_pixels)
    {
        // Simple distance estimation based on object height
        // Assumes known object height (e.g., 1.7m for person)
        double real_height = 1.7; // meters
        if (object_height_pixels > 0) {
            return (real_height * focal_length_) / object_height_pixels;
        }
        return 0.0;
    }
    
    void drawDetections(cv::Mat& image, const std::vector<DetectedObject>& objects)
    {
        for (const auto& obj : objects) {
            // Choose color based on object type
            cv::Scalar color;
            if (obj.class_name == "pedestrian" || obj.class_name == "person") {
                color = cv::Scalar(0, 255, 0); // Green
            } else if (obj.class_name == "traffic_cone") {
                color = cv::Scalar(0, 165, 255); // Orange
            } else {
                color = cv::Scalar(255, 0, 0); // Blue
            }
            
            // Draw bounding box
            cv::rectangle(image, obj.bbox, color, 2);
            
            // Draw label
            std::string label = obj.class_name + " (" + 
                                std::to_string(static_cast<int>(obj.confidence * 100)) + "%)";
            if (obj.distance > 0) {
                label += " " + std::to_string(static_cast<int>(obj.distance * 100) / 100.0) + "m";
            }
            
            cv::putText(image, label, cv::Point(obj.bbox.x, obj.bbox.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }
    }
    
    void publishProcessedImage(const cv::Mat& image)
    {
        // Publish standard image
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            image_header_, "bgr8", image).toImageMsg();
        processed_image_pub_.publish(msg);
        
        // Publish compressed image
        auto compressed_msg = sensor_msgs::msg::CompressedImage();
        compressed_msg.header = image_header_;
        compressed_msg.format = "jpg";
        
        std::vector<uchar> buffer;
        cv::imencode(".jpg", image, buffer);
        compressed_msg.data = buffer;
        
        compressed_image_pub_->publish(compressed_msg);
    }
    
    void publishDetectedObjects(const std::vector<DetectedObject>& objects)
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        for (size_t i = 0; i < objects.size(); ++i) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header = image_header_;
            marker.header.frame_id = "camera_link";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Convert image coordinates to 3D position
            marker.pose.position.x = objects[i].distance;
            marker.pose.position.y = -(objects[i].center.x - image_width_/2.0) * 
                                     objects[i].distance / focal_length_;
            marker.pose.position.z = camera_height_;
            
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 1.0;
            
            marker.color.a = 0.8;
            if (objects[i].class_name == "pedestrian" || objects[i].class_name == "person") {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            }
            
            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(marker);
        }
        
        objects_pub_->publish(marker_array);
    }
    
    void publishLaneCenter(const cv::Point2f& lane_center)
    {
        auto point_msg = geometry_msgs::msg::PointStamped();
        point_msg.header = image_header_;
        point_msg.header.frame_id = "camera_link";
        
        // Convert image coordinates to camera frame
        point_msg.point.x = 2.0; // Fixed distance ahead
        point_msg.point.y = -(lane_center.x - image_width_/2.0) * 2.0 / focal_length_;
        point_msg.point.z = 0.0;
        
        lane_points_pub_->publish(point_msg);
    }
    
    void processingLoop()
    {
        publishHealthStatus();
    }
    
    void publishHealthStatus()
    {
        auto health_msg = tadeo_ecar_msgs::msg::SystemHealth();
        health_msg.header.stamp = this->now();
        health_msg.header.frame_id = "camera_link";
        
        // Set camera status based on image availability
        health_msg.camera_status = current_image_.empty() ? 
            tadeo_ecar_msgs::msg::SystemHealth::ERROR : 
            tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set system component statuses
        health_msg.cpu_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.memory_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.storage_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        health_msg.network_status = tadeo_ecar_msgs::msg::SystemHealth::HEALTHY;
        
        // Set temperatures
        health_msg.cpu_temperature = 45.0; // Placeholder
        health_msg.gpu_temperature = 42.0; // Placeholder
        health_msg.motor_temperature = 38.0; // Placeholder
        
        // Set error information if needed
        if (current_image_.empty()) {
            health_msg.error_codes.push_back(1001);
            health_msg.error_messages.push_back("No camera image received");
        }
        
        // Set diagnostic info
        health_msg.diagnostic_info = "Camera processor running";
        health_msg.uptime_seconds = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        
        health_pub_->publish(health_msg);
    }
    
    // ROS2 interfaces
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher processed_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lane_points_pub_;
    rclcpp::Publisher<tadeo_ecar_msgs::msg::SystemHealth>::SharedPtr health_pub_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    
    // Parameters
    double processing_frequency_;
    bool enable_object_detection_;
    bool enable_lane_detection_;
    double min_confidence_;
    int image_width_;
    int image_height_;
    double focal_length_;
    double camera_height_;
    
    // Computer vision components
    cv::HOGDescriptor hog_;
    cv::CascadeClassifier face_cascade_;
    
    // Color ranges
    cv::Scalar red_lower1_, red_upper1_, red_lower2_, red_upper2_;
    cv::Scalar yellow_lower_, yellow_upper_;
    cv::Scalar green_lower_, green_upper_;
    cv::Scalar white_lower_, white_upper_;
    
    // Current state
    cv::Mat current_image_;
    std_msgs::msg::Header image_header_;
};

} // namespace tadeo_ecar_perception

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tadeo_ecar_perception::CameraProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}