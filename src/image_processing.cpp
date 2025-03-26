#include "image_processing.h"
#include <iostream>

ImageProcessor::ImageProcessor() {}

std::vector<std::string> ImageProcessor::processMaze(const std::string& imagePath) {
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return {};
    }

    // Preprocess and display the binary image
    cv::Mat binaryImage = preprocessImage(image);
    cv::imshow("Binary Image", binaryImage);
    cv::waitKey(0);

    // Detect maze walls
    std::vector<cv::Vec4i> walls = detectMazeWalls(binaryImage);

    // Debugging: Draw walls
    cv::Mat debugImage;
    cv::cvtColor(binaryImage, debugImage, cv::COLOR_GRAY2BGR);
    for (const auto& wall : walls) {
        cv::line(debugImage, cv::Point(wall[0], wall[1]), 
                 cv::Point(wall[2], wall[3]), cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("Maze Walls", debugImage);
    cv::waitKey(0);

    // Generate the maze structure
    std::vector<std::string> maze = generateMazeArray(walls, binaryImage);

    // Debugging: Print the maze array
    for (const auto& row : maze) {
        std::cout << row << std::endl;
    }

    return maze;
}

cv::Mat ImageProcessor::preprocessImage(const cv::Mat& image) {
    cv::Mat blurred, binary;
    cv::GaussianBlur(image, blurred, cv::Size(5, 5), 0);
    cv::adaptiveThreshold(blurred, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 11, 2);
    return binary;
}

std::vector<cv::Vec4i> ImageProcessor::detectMazeWalls(const cv::Mat& binaryImage) {
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(binaryImage, lines, 1, CV_PI / 180, 50, 50, 10);

    // Debugging: Print detected wall segments
    std::cout << "Detected Maze Walls:" << std::endl;
    for (const auto& line : lines) {
        std::cout << "Wall: (" << line[0] << "," << line[1] << ") to ("
                 << line[2] << "," << line[3] << ")" << std::endl;
    }

    return lines;
}

std::vector<std::string> ImageProcessor::generateMazeArray(const std::vector<cv::Vec4i>& walls, const cv::Mat& binaryImage) {
    // Determine grid size based on image dimensions
    const int GRID_SIZE = 20; // pixels per cell
    int rows = binaryImage.rows / GRID_SIZE;
    int cols = binaryImage.cols / GRID_SIZE;
    
    std::vector<std::string> maze(rows, std::string(cols, '.'));

    // Create a matrix to store wall density
    cv::Mat wallDensity = cv::Mat::zeros(rows, cols, CV_32F);

    // Process each wall segment
    for (const auto& wall : walls) {
        // Use Bresenham's line algorithm to get all points along the wall
        cv::LineIterator it(binaryImage, cv::Point(wall[0], wall[1]), 
                          cv::Point(wall[2], wall[3]), 8);
        for(int i = 0; i < it.count; i++, ++it) {
            cv::Point pt = it.pos();
            int row = pt.y / GRID_SIZE;
            int col = pt.x / GRID_SIZE;
            
            if (row >= 0 && row < rows && col >= 0 && col < cols) {
                wallDensity.at<float>(row, col) += 1.0f;
            }
        }
    }

    // Convert density to walls using a threshold
    float threshold = 5.0f; // Adjust this value based on your needs
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (wallDensity.at<float>(i, j) > threshold) {
                maze[i][j] = '#';
            }
        }
    }

    // Debug output
    std::cout << "Maze dimensions: " << rows << "x" << cols << std::endl;
    
    return maze;
}
