/**
 * @file image_processing.cpp
 * @brief Implementation of image processing operations for maze detection
 * 
 * This file contains the implementation of the ImageProcessor class that
 * processes images of mazes captured by a camera. It detects maze walls,
 * pathways, start and end points using computer vision techniques with OpenCV.
 * 
 * @author Original author
 * @date May 2025
 */

#include "image_processing.h"
#include <iostream>

/**
 * @brief Default constructor for the ImageProcessor class
 */
ImageProcessor::ImageProcessor() {}

/**
 * @brief Processes an image to extract maze structure
 * 
 * Takes an image of a maze and processes it to extract the maze structure
 * as an array of characters representing walls, paths, start and end points.
 * 
 * @param image The input image containing the maze
 * @param debugInfo Optional pointer to store debug information and images
 * @return std::vector<std::string> 2D array representation of the maze
 */
std::vector<std::string> ImageProcessor::processMaze(const cv::Mat& image, DebugInfo* debugInfo) {

    if (image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return {};
    }

    // Store original image if debug info requested
    if (debugInfo) {
        debugInfo->image = image.clone();
    }

    // Crop the image to the maze boundaries
    cv::Mat croppedImage = cropToMazeBoundaries(image);

    // Preprocess image
    cv::Mat binaryImage = preprocessImage(croppedImage);
    
    // Detect maze walls
    std::vector<cv::Vec4i> walls = detectMazeWalls(binaryImage);

    // If debug info requested, prepare debug images
    if (debugInfo) {
        debugInfo->binaryImage = binaryImage.clone();
        cv::cvtColor(binaryImage, debugInfo->wallsImage, cv::COLOR_GRAY2BGR);
        for (const auto& wall : walls) {
            cv::line(debugInfo->wallsImage, cv::Point(wall[0], wall[1]), 
                     cv::Point(wall[2], wall[3]), cv::Scalar(0, 0, 255), 2);
        }
        debugInfo->walls = walls;

        cv::imshow("Maze Walls", debugInfo->wallsImage);
    }

    // Generate the maze structure
    return generateMazeArray(walls, binaryImage);
}

/**
 * @brief Crops the image to contain only the maze using ArUco markers
 * 
 * Detects ArUco markers in the image and uses them to define the bounds
 * of the maze, then crops the image to that region.
 * 
 * @param image The input image to crop
 * @return cv::Mat The cropped image containing only the maze
 */
cv::Mat ImageProcessor::cropToMazeBoundaries(const cv::Mat& image) {
    // Detect ArUco markers
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);

    if (markerIds.size() != 2) {
        std::cerr << "Error: Could not detect both ArUco markers." << std::endl;
        return image;
    }

    // Get marker positions
    cv::Point2f marker1Center = (markerCorners[0][0] + markerCorners[0][2]) * 0.5f;
    cv::Point2f marker2Center = (markerCorners[1][0] + markerCorners[1][2]) * 0.5f;

    // Calculate maze region
    float markerDistance = cv::norm(marker2Center - marker1Center);
    cv::Point2f mazeDiagonal = marker2Center - marker1Center;
    cv::Point2f mazeCenter = marker1Center + mazeDiagonal * 0.5f;

    // Calculate maze dimensions based on marker distance
    float mazeSize = markerDistance * 0.55f;
    float halfSize = mazeSize * 0.5f;

    // Create crop region centered between markers
    cv::Rect mazeRegion(
        mazeCenter.x - halfSize,
        mazeCenter.y - halfSize,
        mazeSize,
        mazeSize
    );

    // Ensure crop region is within image bounds
    mazeRegion.x = std::max(0, mazeRegion.x);
    mazeRegion.y = std::max(0, mazeRegion.y);
    mazeRegion.width = std::min(image.cols - mazeRegion.x, mazeRegion.width);
    mazeRegion.height = std::min(image.rows - mazeRegion.y, mazeRegion.height);

    // Debug visualization
    cv::Mat debugImage = image.clone();
    // Draw marker centers
    cv::circle(debugImage, marker1Center, 5, cv::Scalar(255, 0, 0), -1);
    cv::circle(debugImage, marker2Center, 5, cv::Scalar(255, 0, 0), -1);
    // Draw maze region
    cv::rectangle(debugImage, mazeRegion, cv::Scalar(0, 255, 0), 2);
    // Draw line between markers
    cv::line(debugImage, marker1Center, marker2Center, cv::Scalar(0, 0, 255), 2);
    cv::imshow("Maze Detection", debugImage);

    return image(mazeRegion);
}

/**
 * @brief Preprocesses the cropped image for wall detection
 * 
 * Converts the image to grayscale, applies blur and then uses adaptive
 * thresholding to create a binary image where walls are white and paths are black.
 * 
 * @param croppedImage The cropped image containing only the maze
 * @return cv::Mat The binary image ready for wall detection
 */
cv::Mat ImageProcessor::preprocessImage(const cv::Mat& croppedImage) {
    cv::Mat gray, blurred, binary;

    // Convert to grayscale
    cv::cvtColor(croppedImage, gray, cv::COLOR_BGR2GRAY);

    // Reduce noise (blur) using a 5x5 Gaussian filter
    cv::GaussianBlur(gray, blurred, cv::Size(21, 21), 0);

    // Convert grayscale image to binary (white for walls/black for paths)
    cv::adaptiveThreshold(blurred, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 11, 2);
    return binary;
}

/**
 * @brief Detects walls in the binary image using Hough transform
 * 
 * Uses the Hough transform to detect line segments in the binary image,
 * which represent the walls of the maze.
 * 
 * @param binaryImage The preprocessed binary image
 * @return std::vector<cv::Vec4i> Vector of line segments representing walls
 */
std::vector<cv::Vec4i> ImageProcessor::detectMazeWalls(const cv::Mat& binaryImage) {
    std::vector<cv::Vec4i> lines;

    // Use Hough Transform to detect lines in the binary image
    cv::HoughLinesP(binaryImage, lines, 1, CV_PI / 180, 20, 20, 10);

    return lines;
}

/**
 * @brief Generates a 2D maze array from the detected walls
 * 
 * Converts the detected wall segments into a grid representation of the maze,
 * using a fixed size grid. Also detects start and end points of the maze.
 * 
 * @param walls Vector of line segments representing maze walls
 * @param binaryImage The binary image used for reference dimensions
 * @return std::vector<std::string> 2D array representation of the maze
 */
std::vector<std::string> ImageProcessor::generateMazeArray(const std::vector<cv::Vec4i>& walls, const cv::Mat& binaryImage) {
    const int MAZE_SIZE = 17; // Fixed grid
    std::vector<std::string> maze(MAZE_SIZE, std::string(MAZE_SIZE, '.'));

    // Calculate cell size based on image dimensions
    float cellWidth = static_cast<float>(binaryImage.cols) / MAZE_SIZE;
    float cellHeight = static_cast<float>(binaryImage.rows) / MAZE_SIZE;
    
    // Create a matrix to store wall density
    cv::Mat wallDensity = cv::Mat::zeros(MAZE_SIZE, MAZE_SIZE, CV_32F);

    // Process each wall segment
    for (const auto& wall : walls) {
        // Use Bresenham's line algorithm to get all points along the wall
        cv::LineIterator it(binaryImage, cv::Point(wall[0], wall[1]), 
                          cv::Point(wall[2], wall[3]), 8);
        for(int i = 0; i < it.count; i++, ++it) {
            cv::Point pt = it.pos();
            // Convert pixel coordinates to grid coordinates
            int row = static_cast<int>(pt.y / cellHeight);
            int col = static_cast<int>(pt.x / cellWidth);
            
            if (row >= 0 && row < MAZE_SIZE && col >= 0 && col < MAZE_SIZE) {
                wallDensity.at<float>(row, col) += 1.0f;
            }
        }
    }

    // Convert density to walls using a threshold
    float threshold = 20.0f; // Adjust this value based on conversion accuracy (lower = more walls)
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            if (wallDensity.at<float>(i, j) > threshold) {
                maze[i][j] = '#';
            }
        }
    }

    // Detect start and end points
    auto [startFound, endFound] = detectStartEndPoints(maze, MAZE_SIZE);

    // Debug output
    std::cout << "Generated " << MAZE_SIZE << "x" << MAZE_SIZE << " maze:" << std::endl;
    if (!startFound) std::cout << "Warning: No start point found!" << std::endl;
    if (!endFound) std::cout << "Warning: No end point found!" << std::endl;
    
    return maze;
}

/**
 * @brief Detects start and end points of the maze
 * 
 * Looks for openings in the maze boundary walls to set start (S) and end (E) points.
 * Checks all four walls in priority order: left, top, right, bottom.
 * 
 * @param maze Reference to the maze array to modify
 * @param mazeSize Size of the maze grid
 * @return std::pair<bool, bool> Flags indicating if start and end points were found
 */
std::pair<bool, bool> ImageProcessor::detectStartEndPoints(std::vector<std::string>& maze, const int mazeSize) {
    bool startFound = false;
    bool endFound = false;

    // Priority order for start point: Left -> Top -> Right -> Bottom
    // Check left wall for start
    for (int i = 0; i < mazeSize && !startFound; i++) {
        if (maze[i][0] == '.' && maze[i][1] == '.') {
            maze[i][0] = 'S';
            startFound = true;
        }
    }

    // Check top wall for start if not found
    for (int j = 0; j < mazeSize && !startFound; j++) {
        if (maze[0][j] == '.' && maze[1][j] == '.') {
            maze[0][j] = 'S';
            startFound = true;
        }
    }

    // Check right wall for start if not found
    for (int i = 0; i < mazeSize && !startFound; i++) {
        if (maze[i][mazeSize-1] == '.' && maze[i][mazeSize-2] == '.') {
            maze[i][mazeSize-1] = 'S';
            startFound = true;
        }
    }

    // Check bottom wall for start if not found
    for (int j = 0; j < mazeSize && !startFound; j++) {
        if (maze[mazeSize-1][j] == '.' && maze[mazeSize-2][j] == '.') {
            maze[mazeSize-1][j] = 'S';
            startFound = true;
        }
    }

    // After finding start, check all walls again for any remaining gaps to mark as end
    // Check left wall for end
    for (int i = 0; i < mazeSize && !endFound; i++) {
        if (maze[i][0] == '.' && maze[i][1] == '.') {
            maze[i][0] = 'E';
            endFound = true;
        }
    }

    // Check top wall for end if not found
    for (int j = 0; j < mazeSize && !endFound; j++) {
        if (maze[0][j] == '.' && maze[1][j] == '.') {
            maze[0][j] = 'E';
            endFound = true;
        }
    }

    // Check right wall for end if not found
    for (int i = 0; i < mazeSize && !endFound; i++) {
        if (maze[i][mazeSize-1] == '.' && maze[i][mazeSize-2] == '.') {
            maze[i][mazeSize-1] = 'E';
            endFound = true;
        }
    }

    // Check bottom wall for end if not found
    for (int j = 0; j < mazeSize && !endFound; j++) {
        if (maze[mazeSize-1][j] == '.' && maze[mazeSize-2][j] == '.') {
            maze[mazeSize-1][j] = 'E';
            endFound = true;
        }
    }

    return {startFound, endFound};
}
