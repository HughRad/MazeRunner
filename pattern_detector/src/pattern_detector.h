#ifndef PATTERN_DETECTOR_H
#define PATTERN_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// Structure to represent a detected shape
struct Shape {
    enum Type { SQUARE, CIRCLE } type;
    cv::Point2f center;
    float size;  // Radius for circle, side length for square
    cv::Scalar color;
};

// Structure to represent the pattern arrangement
struct PatternArrangement {
    std::vector<Shape> shapes;
    cv::Point2f center;
    float width;   // Width of pattern in pixels
    float height;  // Height of pattern in pixels
    float rotation; // Rotation of pattern in radians
};

class PatternDetector {
public:
    PatternDetector(ros::NodeHandle& nh);
    ~PatternDetector();

    // Process incoming camera image
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    // Process camera info 
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    
    // Detect red shapes in the image
    std::vector<Shape> detectRedShapes(const cv::Mat& image);
    
    // Identify if shapes form our target pattern
    bool identifyPattern(const std::vector<Shape>& shapes, PatternArrangement& pattern);
    
    // Calculate camera movement to align with pattern
    geometry_msgs::Twist calculateAlignment(const PatternArrangement& detected, 
                                           const PatternArrangement& target);
    
    // Draw UI overlay with pattern and alignment information
    cv::Mat drawUIOverlay(const cv::Mat& image, const PatternArrangement& detected, 
                         const PatternArrangement& target);

private:
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Image transport
    image_transport::ImageTransport it_;
    
    // Subscribers
    image_transport::Subscriber image_sub_;
    ros::Subscriber camera_info_sub_;
    
    // Publishers
    image_transport::Publisher overlay_pub_;
    ros::Publisher cmd_vel_pub_;
    
    // Camera parameters
    sensor_msgs::CameraInfo camera_info_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    // Target pattern dimensions in meters
    const float PATTERN_WIDTH_M = 0.3;  // 30 cm
    const float PATTERN_HEIGHT_M = 0.2; // 20 cm
    
    // Target pattern arrangement
    PatternArrangement target_pattern_;
    
    // Color detection parameters
    cv::Scalar red_lower1_;
    cv::Scalar red_upper1_;
    cv::Scalar red_lower2_;
    cv::Scalar red_upper2_;
    
    // Shape detection parameters
    float circle_detection_dp_;
    float circle_min_dist_;
    int circle_param1_;
    int circle_param2_;
    int circle_min_radius_;
    int circle_max_radius_;
    float square_approx_epsilon_factor_;
    
    // Helper functions
    std::vector<cv::Point> detectRedContours(const cv::Mat& image);
    std::vector<Shape> identifyCircles(const cv::Mat& image, const cv::Mat& mask);
    std::vector<Shape> identifySquares(const std::vector<cv::Point>& contours);
    bool isSquare(const std::vector<cv::Point>& contour, cv::Point2f& center, float& size);
    float calculatePatternRotation(const PatternArrangement& pattern);
};

#endif // PATTERN_DETECTOR_H
