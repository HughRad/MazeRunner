#include "pattern_detector.h"

PatternDetector::PatternDetector(ros::NodeHandle& nh) : nh_(nh), it_(nh) {
    // Initialize subscribers
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &PatternDetector::imageCallback, this);
    camera_info_sub_ = nh_.subscribe("/camera/camera_info", 1, &PatternDetector::cameraInfoCallback, this);
    
    // Initialize publishers
    overlay_pub_ = it_.advertise("/pattern_detector/overlay", 1);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    // Initialize red color detection thresholds (in HSV color space)
    // Red in HSV wraps around 0/180, so we need two ranges
    red_lower1_ = cv::Scalar(0, 100, 100);
    red_upper1_ = cv::Scalar(10, 255, 255);
    red_lower2_ = cv::Scalar(160, 100, 100);
    red_upper2_ = cv::Scalar(180, 255, 255);
    
    // Initialize circle detection parameters
    circle_detection_dp_ = 1.2;
    circle_min_dist_ = 20.0;
    circle_param1_ = 50;
    circle_param2_ = 30;
    circle_min_radius_ = 10;
    circle_max_radius_ = 100;
    
    // Initialize square detection parameters
    square_approx_epsilon_factor_ = 0.02;
    
    // Initialize target pattern
    // Arrange 2 circles and 2 squares in rectangle formation
    // Using normalized coordinates, where (0,0) is center and width/height are 1.0
    Shape circle1, circle2, square1, square2;
    
    circle1.type = Shape::CIRCLE;
    circle1.center = cv::Point2f(-0.5, -0.5);  // Top-left
    circle1.size = 0.1;  // Relative to pattern size
    circle1.color = cv::Scalar(0, 0, 255);  // Red
    
    square1.type = Shape::SQUARE;
    square1.center = cv::Point2f(0.5, -0.5);  // Top-right
    square1.size = 0.15;  // Relative to pattern size
    square1.color = cv::Scalar(0, 0, 255);  // Red
    
    circle2.type = Shape::CIRCLE;
    circle2.center = cv::Point2f(-0.5, 0.5);  // Bottom-left
    circle2.size = 0.1;  // Relative to pattern size
    circle2.color = cv::Scalar(0, 0, 255);  // Red
    
    square2.type = Shape::SQUARE;
    square2.center = cv::Point2f(0.5, 0.5);  // Bottom-right
    square2.size = 0.15;  // Relative to pattern size
    square2.color = cv::Scalar(0, 0, 255);  // Red
    
    target_pattern_.shapes.push_back(circle1);
    target_pattern_.shapes.push_back(circle2);
    target_pattern_.shapes.push_back(square1);
    target_pattern_.shapes.push_back(square2);
    target_pattern_.center = cv::Point2f(0, 0);
    target_pattern_.width = 1.0;
    target_pattern_.height = 1.0;
    target_pattern_.rotation = 0.0;
    
    // Log initialization
    ROS_INFO("Pattern Detector initialized");
}

PatternDetector::~PatternDetector() {
    // Cleanup resources if needed
}

void PatternDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Detect red shapes
    std::vector<Shape> shapes = detectRedShapes(cv_ptr->image);
    
    // Identify pattern arrangement
    PatternArrangement detected_pattern;
    bool pattern_found = identifyPattern(shapes, detected_pattern);
    
    // Draw UI overlay
    cv::Mat overlay = drawUIOverlay(cv_ptr->image, detected_pattern, target_pattern_);
    
    // Publish overlay image
    sensor_msgs::ImagePtr overlay_msg = cv_bridge::CvImage(msg->header, "bgr8", overlay).toImageMsg();
    overlay_pub_.publish(overlay_msg);
    
    // If pattern found, calculate and publish alignment commands
    if (pattern_found) {
        geometry_msgs::Twist cmd_vel = calculateAlignment(detected_pattern, target_pattern_);
        cmd_vel_pub_.publish(cmd_vel);
    }
}

void PatternDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    camera_info_ = *msg;
    
    // Extract camera matrix and distortion coefficients
    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_info_.K.data())).clone();
    dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double*>(camera_info_.D.data())).clone();
    
    ROS_INFO("Camera info received");
}

std::vector<Shape> PatternDetector::detectRedShapes(const cv::Mat& image) {
    // Convert image to HSV color space
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    
    // Create mask for red color (using two ranges)
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, red_lower1_, red_upper1_, mask1);
    cv::inRange(hsv, red_lower2_, red_upper2_, mask2);
    cv::bitwise_or(mask1, mask2, mask);
    
    // Apply morphological operations to clean up the mask
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // Find contours in the mask
    std::vector<cv::Point> contours = detectRedContours(mask);
    
    // Identify circles using Hough transform
    std::vector<Shape> circles = identifyCircles(image, mask);
    
    // Identify squares using contour analysis
    std::vector<Shape> squares = identifySquares(contours);
    
    // Combine the detected shapes
    std::vector<Shape> shapes;
    shapes.insert(shapes.end(), circles.begin(), circles.end());
    shapes.insert(shapes.end(), squares.begin(), squares.end());
    
    return shapes;
}

std::vector<cv::Point> PatternDetector::detectRedContours(const cv::Mat& mask) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Flatten the contours for convenience
    std::vector<cv::Point> allContours;
    for (const auto& contour : contours) {
        allContours.insert(allContours.end(), contour.begin(), contour.end());
    }
    
    return allContours;
}

std::vector<Shape> PatternDetector::identifyCircles(const cv::Mat& image, const cv::Mat& mask) {
    std::vector<Shape> circles;
    std::vector<cv::Vec3f> detected_circles;
    
    // Apply Hough Circle Transform
    cv::HoughCircles(mask, detected_circles, cv::HOUGH_GRADIENT, 
                    circle_detection_dp_, circle_min_dist_, 
                    circle_param1_, circle_param2_, 
                    circle_min_radius_, circle_max_radius_);
    
    // Create Shape objects for each detected circle
    for (const auto& c : detected_circles) {
        Shape circle;
        circle.type = Shape::CIRCLE;
        circle.center = cv::Point2f(c[0], c[1]);
        circle.size = c[2];  // Radius
        circle.color = cv::Scalar(0, 0, 255);  // Red
        circles.push_back(circle);
    }
    
    return circles;
}

std::vector<Shape> PatternDetector::identifySquares(const std::vector<cv::Point>& contours) {
    std::vector<Shape> squares;
    
    // Find contours in the mask
    std::vector<std::vector<cv::Point>> contoursList;
    if (!contours.empty()) {
        // If we received flattened contours, we need to split them back
        // This is a simplified approach - in practice, you'd want better contour grouping
        std::vector<cv::Point> current;
        for (size_t i = 0; i < contours.size(); i++) {
            if (i > 0 && cv::norm(contours[i] - contours[i-1]) > 5) {
                if (current.size() > 5) {  // Minimum points for a meaningful contour
                    contoursList.push_back(current);
                }
                current.clear();
            }
            current.push_back(contours[i]);
        }
        if (current.size() > 5) {
            contoursList.push_back(current);
        }
    }
    
    // Check each contour for square-like shape
    for (const auto& contour : contoursList) {
        if (contour.size() < 4) continue;
        
        Shape square;
        if (isSquare(contour, square.center, square.size)) {
            square.type = Shape::SQUARE;
            square.color = cv::Scalar(0, 0, 255);  // Red
            squares.push_back(square);
        }
    }
    
    return squares;
}

bool PatternDetector::isSquare(const std::vector<cv::Point>& contour, cv::Point2f& center, float& size) {
    // Approximate contour with polygon
    std::vector<cv::Point> approx;
    double epsilon = square_approx_epsilon_factor_ * cv::arcLength(contour, true);
    cv::approxPolyDP(contour, approx, epsilon, true);
    
    // Check if the polygon has 4 vertices
    if (approx.size() != 4) return false;
    
    // Check if the shape is convex
    if (!cv::isContourConvex(approx)) return false;
    
    // Find center and side length
    cv::Moments m = cv::moments(approx);
    center = cv::Point2f(m.m10/m.m00, m.m01/m.m00);
    
    // Check if sides are approximately equal
    std::vector<double> sides;
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        double side = cv::norm(approx[i] - approx[j]);
        sides.push_back(side);
    }
    
    double mean_side = (sides[0] + sides[1] + sides[2] + sides[3]) / 4.0;
    size = mean_side / 2.0;  // Using half side as size for consistency with circles
    
    // Check if sides are similar in length (within 20%)
    for (int i = 0; i < 4; i++) {
        if (std::abs(sides[i] - mean_side) > 0.2 * mean_side) return false;
    }
    
    // Check if angles are approximately 90 degrees
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        int k = (i + 2) % 4;
        cv::Point2f v1 = approx[j] - approx[i];
        cv::Point2f v2 = approx[k] - approx[j];
        double dot = v1.x * v2.x + v1.y * v2.y;
        double mag1 = cv::norm(v1);
        double mag2 = cv::norm(v2);
        double cosine = dot / (mag1 * mag2);
        
        // Check if angle is close to 90 degrees (cos(90) = 0)
        if (std::abs(cosine) > 0.3) return false;
    }
    
    return true;
}

bool PatternDetector::identifyPattern(const std::vector<Shape>& shapes, PatternArrangement& pattern) {
    // Check if we have at least 4 shapes (2 circles, 2 squares)
    if (shapes.size() < 4) return false;
    
    // Count circles and squares
    std::vector<Shape> circles, squares;
    for (const auto& shape : shapes) {
        if (shape.type == Shape::CIRCLE) {
            circles.push_back(shape);
        } else if (shape.type == Shape::SQUARE) {
            squares.push_back(shape);
        }
    }
    
    // Check if we have exactly 2 circles and 2 squares
    if (circles.size() < 2 || squares.size() < 2) return false;
    
    // Sort shapes by position (left to right, top to bottom)
    std::sort(circles.begin(), circles.end(), 
        [](const Shape& a, const Shape& b) {
            return (a.center.y < b.center.y) || (a.center.y == b.center.y && a.center.x < b.center.x);
        });
    
    std::sort(squares.begin(), squares.end(), 
        [](const Shape& a, const Shape& b) {
            return (a.center.y < b.center.y) || (a.center.y == b.center.y && a.center.x < b.center.x);
        });
    
    // Take the first 2 circles and squares
    std::vector<Shape> pattern_shapes;
    pattern_shapes.push_back(circles[0]);
    pattern_shapes.push_back(circles[1]);
    pattern_shapes.push_back(squares[0]);
    pattern_shapes.push_back(squares[1]);
    
    // Calculate pattern properties
    cv::Point2f top_left = pattern_shapes[0].center;
    cv::Point2f bottom_right = pattern_shapes[3].center;
    
    pattern.shapes = pattern_shapes;
    pattern.center = cv::Point2f((top_left.x + bottom_right.x) / 2.0, 
                                (top_left.y + bottom_right.y) / 2.0);
    pattern.width = std::abs(bottom_right.x - top_left.x);
    pattern.height = std::abs(bottom_right.y - top_left.y);
    pattern.rotation = calculatePatternRotation(pattern);
    
    // Check if the pattern dimensions match the expected ratio (30x20 cm)
    float aspect_ratio = pattern.width / pattern.height;
    float expected_ratio = PATTERN_WIDTH_M / PATTERN_HEIGHT_M;
    
    // Allow 20% tolerance in aspect ratio
    if (std::abs(aspect_ratio - expected_ratio) > 0.2 * expected_ratio) {
        return false;
    }
    
    return true;
}

float PatternDetector::calculatePatternRotation(const PatternArrangement& pattern) {
    // Calculate rotation based on the angle between the pattern's axes and the image axes
    if (pattern.shapes.size() < 4) return 0.0;
    
    // Use the line from top-left to top-right to calculate rotation
    cv::Point2f top_left = pattern.shapes[0].center;
    cv::Point2f top_right = pattern.shapes[2].center;
    
    // Calculate the angle
    float dx = top_right.x - top_left.x;
    float dy = top_right.y - top_left.y;
    float angle = std::atan2(dy, dx);
    
    return angle;
}

geometry_msgs::Twist PatternDetector::calculateAlignment(const PatternArrangement& detected, 
                                                        const PatternArrangement& target) {
    geometry_msgs::Twist cmd_vel;
    
    // Size difference indicates forward/backward movement
    float size_ratio = detected.width / (target.width * PATTERN_WIDTH_M);
    
    // Position difference indicates left/right and up/down movement
    float image_center_x = camera_info_.width / 2.0;
    float image_center_y = camera_info_.height / 2.0;
    
    float x_error = (detected.center.x - image_center_x) / image_center_x;
    float y_error = (detected.center.y - image_center_y) / image_center_y;
    
    // Rotation difference indicates yaw adjustment
    float rotation_error = detected.rotation - target.rotation;
    
    // Apply proportional control
    const float K_LINEAR_X = 0.2;  // Forward/backward gain
    const float K_LINEAR_Y = 0.2;  // Left/right gain
    const float K_LINEAR_Z = 0.2;  // Up/down gain
    const float K_ANGULAR_Z = 0.5; // Yaw gain
    
    // Calculate velocities (with deadband to prevent oscillation)
    const float DEADBAND = 0.05;
    
    // Size control (z-axis)
    if (std::abs(1.0 - size_ratio) > DEADBAND) {
        cmd_vel.linear.z = K_LINEAR_Z * (1.0 - size_ratio);
    }
    
    // Position control (x, y axes)
    if (std::abs(x_error) > DEADBAND) {
        cmd_vel.linear.x = -K_LINEAR_X * x_error;
    }
    
    if (std::abs(y_error) > DEADBAND) {
        cmd_vel.linear.y = -K_LINEAR_Y * y_error;
    }
    
    // Rotation control (yaw)
    if (std::abs(rotation_error) > DEADBAND) {
        cmd_vel.angular.z = -K_ANGULAR_Z * rotation_error;
    }
    
    return cmd_vel;
}

cv::Mat PatternDetector::drawUIOverlay(const cv::Mat& image, const PatternArrangement& detected, 
                                      const PatternArrangement& target) {
    cv::Mat overlay = image.clone();
    
    // Draw target pattern overlay
    const int UI_MARGIN = 10;
    const int UI_WIDTH = 150;
    const int UI_HEIGHT = 100;
    
    // Create UI box in the top-right corner
    cv::Rect ui_rect(image.cols - UI_WIDTH - UI_MARGIN, UI_MARGIN, UI_WIDTH, UI_HEIGHT);
    cv::rectangle(overlay, ui_rect, cv::Scalar(0, 0, 0), -1);
    cv::rectangle(overlay, ui_rect, cv::Scalar(255, 255, 255), 1);
    
    // Draw target pattern in UI
    cv::Point ui_center(ui_rect.x + ui_rect.width / 2, ui_rect.y + ui_rect.height / 2);
    int ui_pattern_width = ui_rect.width * 0.8;
    int ui_pattern_height = ui_rect.height * 0.8;
    
    // Draw target pattern shapes
    for (const auto& shape : target.shapes) {
        cv::Point2f pos(
            ui_center.x + shape.center.x * ui_pattern_width / 2,
            ui_center.y + shape.center.y * ui_pattern_height / 2
        );
        
        if (shape.type == Shape::CIRCLE) {
            int radius = shape.size * ui_pattern_width / 2;
            cv::circle(overlay, pos, radius, shape.color, 2);
        } else if (shape.type == Shape::SQUARE) {
            int half_side = shape.size * ui_pattern_width / 2;
            cv::Rect rect(pos.x - half_side, pos.y - half_side, half_side * 2, half_side * 2);
            cv::rectangle(overlay, rect, shape.color, 2);
        }
    }
    
    // Add alignment status text
    std::string status = "Searching...";
    if (detected.shapes.size() == 4) {
        status = "Pattern Found";
    }
    cv::putText(overlay, status, cv::Point(ui_rect.x + 5, ui_rect.y + 15), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
    
    // If pattern is detected, highlight it in the main view
    if (detected.shapes.size() == 4) {
        // Draw bounding box around the pattern
        cv::Point2f top_left(detected.center.x - detected.width/2, detected.center.y - detected.height/2);
        cv::Point2f bottom_right(detected.center.x + detected.width/2, detected.center.y + detected.height/2);
        cv::rectangle(overlay, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);
        
        // Draw each shape
        for (const auto& shape : detected.shapes) {
            if (shape.type == Shape::CIRCLE) {
                cv::circle(overlay, shape.center, shape.size, cv::Scalar(0, 255, 0), 2);
            } else if (shape.type == Shape::SQUARE) {
                cv::Rect rect(shape.center.x - shape.size, shape.center.y - shape.size, 
                             shape.size * 2, shape.size * 2);
                cv::rectangle(overlay, rect, cv::Scalar(0, 255, 0), 2);
            }
        }
        
        // Draw center crosshair
        cv::line(overlay, cv::Point(detected.center.x - 10, detected.center.y), 
                cv::Point(detected.center.x + 10, detected.center.y), cv::Scalar(0, 255, 0), 1);
        cv::line(overlay, cv::Point(detected.center.x, detected.center.y - 10), 
                cv::Point(detected.center.x, detected.center.y + 10), cv::Scalar(0, 255, 0), 1);
        
        // Draw alignment guide
        cv::Point2f image_center(image.cols / 2, image.rows / 2);
        cv::circle(overlay, image_center, 5, cv::Scalar(0, 255, 255), -1);
        cv::line(overlay, image_center, detected.center, cv::Scalar(0, 255, 255), 1);
    }
    
    return overlay;
}
