#include "pattern_detector.h"
#include <opencv2/highgui.hpp>
#include <ros/package.h>
#include <string>
#include <iostream>

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "pattern_detector_test");
    ros::NodeHandle nh;
    
    // Create the pattern detector
    PatternDetector detector(nh);
    
    // Get the package path for finding test images
    std::string pkg_path = ros::package::getPath("pattern_detector");
    std::string test_images_path = pkg_path + "/test_images";
    
    // List of test images
    std::vector<std::string> image_files = {
        "test_pattern_1.jpg",
        "test_pattern_2.jpg",
        "test_pattern_3.jpg"
    };
    
    for (const auto& image_file : image_files) {
        std::string image_path = test_images_path + "/" + image_file;
        std::cout << "Processing: " << image_path << std::endl;
        
        // Load the test image
        cv::Mat image = cv::imread(image_path);
        if (image.empty()) {
            std::cerr << "Error: Could not load image " << image_path << std::endl;
            continue;
        }
        
        // Process the image
        std::vector<Shape> shapes = detector.detectRedShapes(image);
        
        // Check if pattern is found
        PatternArrangement detected_pattern;
        bool pattern_found = detector.identifyPattern(shapes, detected_pattern);
        
        // Get the overlay with detection results
        cv::Mat overlay = detector.drawUIOverlay(image, detected_pattern, detector.target_pattern_);
        
        // Display results
        std::cout << "Image: " << image_file << std::endl;
        std::cout << "Pattern found: " << (pattern_found ? "YES" : "NO") << std::endl;
        if (pattern_found) {
            std::cout << "Pattern center: (" << detected_pattern.center.x << ", " 
                     << detected_pattern.center.y << ")" << std::endl;
            std::cout << "Pattern size: " << detected_pattern.width << " x " 
                     << detected_pattern.height << " pixels" << std::endl;
            std::cout << "Pattern rotation: " << detected_pattern.rotation << " radians" << std::endl;
            
            // Display the movement that would be needed
            geometry_msgs::Twist movement = detector.calculateAlignment(detected_pattern, detector.target_pattern_);
            std::cout << "Suggested movement:" << std::endl;
            std::cout << "  Linear: [" << movement.linear.x << ", " << movement.linear.y 
                     << ", " << movement.linear.z << "]" << std::endl;
            std::cout << "  Angular: [" << movement.angular.x << ", " << movement.angular.y 
                     << ", " << movement.angular.z << "]" << std::endl;
        }
        
        // Save the overlay image with detection results
        std::string result_path = test_images_path + "/result_" + image_file;
        cv::imwrite(result_path, overlay);
        std::cout << "Result saved to: " << result_path << std::endl;
        
        // Show the result (if display is available)
        cv::imshow("Detection Result", overlay);
        cv::waitKey(0);
        
        std::cout << "------------------------------------------------" << std::endl;
    }
    
    cv::destroyAllWindows();
    return 0;
}