#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

// Function to create a test image with the pattern in different positions/orientations
void createTestImage(const std::string& filename, 
                    cv::Point center = cv::Point(320, 240),
                    double scale = 1.0,
                    double rotation = 0.0,
                    bool addNoise = false) {
    
    // Create a blank image (white background)
    cv::Mat image(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Define the pattern dimensions (30cm x 20cm) - scale to pixels
    // Assuming 100 pixels = 10cm for simplicity
    int pattern_width = 300 * scale;  // 30cm
    int pattern_height = 200 * scale; // 20cm
    
    // Define the corners of the pattern
    cv::Point2f top_left(center.x - pattern_width/2, center.y - pattern_height/2);
    cv::Point2f top_right(center.x + pattern_width/2, center.y - pattern_height/2);
    cv::Point2f bottom_left(center.x - pattern_width/2, center.y + pattern_height/2);
    cv::Point2f bottom_right(center.x + pattern_width/2, center.y + pattern_height/2);
    
    // Calculate rotation matrix
    cv::Mat rot_mat = cv::getRotationMatrix2D(center, rotation * 180 / M_PI, 1.0);
    
    // Apply rotation to corner points
    std::vector<cv::Point2f> corners = {top_left, top_right, bottom_left, bottom_right};
    std::vector<cv::Point2f> rotated_corners;
    cv::transform(corners, rotated_corners, rot_mat);
    
    top_left = rotated_corners[0];
    top_right = rotated_corners[1];
    bottom_left = rotated_corners[2];
    bottom_right = rotated_corners[3];
    
    // Draw pattern outline (for reference)
    std::vector<cv::Point> contour = {
        cv::Point(top_left), 
        cv::Point(top_right), 
        cv::Point(bottom_right), 
        cv::Point(bottom_left)
    };
    cv::polylines(image, contour, true, cv::Scalar(200, 200, 200), 1);
    
    // Define circle radius and square side
    int circle_radius = 25 * scale;
    int square_side = 40 * scale;
    
    // Draw the circles (red)
    cv::circle(image, top_left, circle_radius, cv::Scalar(0, 0, 255), -1);
    cv::circle(image, bottom_left, circle_radius, cv::Scalar(0, 0, 255), -1);
    
    // Draw the squares (red)
    // We need to calculate the corners of each square
    double half_side = square_side / 2.0;
    
    // Top-right square corners
    std::vector<cv::Point2f> tr_square = {
        cv::Point2f(top_right.x - half_side, top_right.y - half_side),
        cv::Point2f(top_right.x + half_side, top_right.y - half_side),
        cv::Point2f(top_right.x + half_side, top_right.y + half_side),
        cv::Point2f(top_right.x - half_side, top_right.y + half_side)
    };
    
    // Bottom-right square corners
    std::vector<cv::Point2f> br_square = {
        cv::Point2f(bottom_right.x - half_side, bottom_right.y - half_side),
        cv::Point2f(bottom_right.x + half_side, bottom_right.y - half_side),
        cv::Point2f(bottom_right.x + half_side, bottom_right.y + half_side),
        cv::Point2f(bottom_right.x - half_side, bottom_right.y + half_side)
    };
    
    // Rotate the square corners
    std::vector<cv::Point2f> tr_square_rot, br_square_rot;
    cv::transform(tr_square, tr_square_rot, rot_mat);
    cv::transform(br_square, br_square_rot, rot_mat);
    
    // Convert to integer points for drawing
    std::vector<cv::Point> tr_square_pts, br_square_pts;
    for (const auto& p : tr_square_rot) tr_square_pts.push_back(cv::Point(p));
    for (const auto& p : br_square_rot) br_square_pts.push_back(cv::Point(p));
    
    // Draw the squares
    cv::fillPoly(image, tr_square_pts, cv::Scalar(0, 0, 255));
    cv::fillPoly(image, br_square_pts, cv::Scalar(0, 0, 255));
    
    // Add some noise/distortion if requested
    if (addNoise) {
        // Add Gaussian noise
        cv::Mat noise(image.size(), image.type());
        cv::randn(noise, cv::Scalar(0, 0, 0), cv::Scalar(10, 10, 10));
        image += noise;
        
        // Add some blur
        cv::GaussianBlur(image, image, cv::Size(5, 5), 2.0);
    }
    
    // Save the test image
    cv::imwrite(filename, image);
    std::cout << "Created test image: " << filename << std::endl;
}

int main() {
    // Create a directory for test images if it doesn't exist
    system("mkdir -p test_images");
    
    // Create various test images with different parameters
    
    // 1. Basic centered pattern
    createTestImage("test_images/test_pattern_1.jpg");
    
    // 2. Shifted pattern
    createTestImage("test_images/test_pattern_2.jpg", cv::Point(400, 200), 1.0, 0.0, false);
    
    // 3. Rotated pattern
    createTestImage("test_images/test_pattern_3.jpg", cv::Point(320, 240), 1.0, M_PI/8, false);
    
    // 4. Scaled pattern (smaller)
    createTestImage("test_images/test_pattern_4.jpg", cv::Point(320, 240), 0.7, 0.0, false);
    
    // 5. Scaled pattern (larger)
    createTestImage("test_images/test_pattern_5.jpg", cv::Point(320, 240), 1.3, 0.0, false);
    
    // 6. Rotated and shifted pattern
    createTestImage("test_images/test_pattern_6.jpg", cv::Point(250, 300), 1.0, -M_PI/12, false);
    
    // 7. Pattern with noise
    createTestImage("test_images/test_pattern_7.jpg", cv::Point(320, 240), 1.0, 0.0, true);
    
    // 8. Difficult case - rotated, shifted, scaled with noise
    createTestImage("test_images/test_pattern_8.jpg", cv::Point(400, 350), 0.8, M_PI/6, true);
    
    std::cout << "Generated 8 test images in the test_images directory" << std::endl;
    return 0;
}