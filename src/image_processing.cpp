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
    cv::waitKey(0); // Debug: Pause to inspect

    // Detect grid intersections
    std::vector<cv::Point> gridPoints = detectGridPoints(binaryImage);

    // Debugging: Draw grid points
    cv::Mat debugImage;
    cv::cvtColor(binaryImage, debugImage, cv::COLOR_GRAY2BGR);
    for (const auto& pt : gridPoints) {
        cv::circle(debugImage, pt, 5, cv::Scalar(0, 0, 255), -1);
    }
    cv::imshow("Grid Points", debugImage);
    cv::waitKey(0); // Debug: Pause to inspect

    // Generate the maze structure
    std::vector<std::string> maze = generateMazeArray(gridPoints, binaryImage);

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

std::vector<cv::Point> ImageProcessor::detectGridPoints(const cv::Mat& binaryImage) {
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(binaryImage, lines, 1, CV_PI / 180, 50, 50, 10);

    std::vector<cv::Point> gridPoints;
    for (const auto& line : lines) {
        gridPoints.emplace_back(line[0], line[1]);
        gridPoints.emplace_back(line[2], line[3]);
    }

    // Debugging: Print detected points
    std::cout << "Detected Grid Points:" << std::endl;
    for (const auto& pt : gridPoints) {
        std::cout << "(" << pt.x << ", " << pt.y << ")" << std::endl;
    }

    return gridPoints;
}

std::vector<std::string> ImageProcessor::generateMazeArray(const std::vector<cv::Point>& gridPoints, const cv::Mat& binaryImage) {
    std::vector<std::string> maze(9, std::string(9, '.'));

    for (size_t i = 0; i < gridPoints.size(); i++) {
        for (size_t j = i + 1; j < gridPoints.size(); j++) {
            if (cv::norm(gridPoints[i] - gridPoints[j]) < 20) {
                int row = std::min(gridPoints[i].y, gridPoints[j].y) / (binaryImage.rows / 9);
                int col = std::min(gridPoints[i].x, gridPoints[j].x) / (binaryImage.cols / 9);
                maze[row][col] = '#';
            }
        }
    }

    return maze;
}
