#include "image_processing.h"
#include <iostream>

ImageProcessor::ImageProcessor() {}

std::vector<std::string> ImageProcessor::processMaze(const std::string& imagePath, DebugInfo* debugInfo) {
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
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
    }

    // Generate the maze structure
    return generateMazeArray(walls, binaryImage);
}

cv::Mat ImageProcessor::cropToMazeBoundaries(const cv::Mat& image) {
    cv::Mat croppedImage;

    // Load ArUco dictionary and parameters
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    
    // Detect ArUco markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);

    // Debug output
    if (markerIds.size() != 2) {
        std::cerr << "Error: Could not detect ArUco markers." << std::endl;
        return image; // Return original image if markers aren't found
    }

    // Collect all marker corners
    std::vector<cv::Point2f> allPoints;
    for (const auto& corners : markerCorners) {
        allPoints.insert(allPoints.end(), corners.begin(), corners.end());
    }

    // Get bounding rectangle of all markers
    cv::Rect boundingBox = cv::boundingRect(allPoints);

    // Define a margin to remove the markers
    int margin = 100; // Adjust based on marker size
    cv::Rect mazeRegion(
        std::max(0, boundingBox.x + margin),
        std::max(0, boundingBox.y + margin),
        std::min(image.cols - boundingBox.x - margin, boundingBox.width - 2 * margin),
        std::min(image.rows - boundingBox.y - margin, boundingBox.height - 2 * margin)
    );

    // Ensure cropping region is valid
    if (mazeRegion.x + mazeRegion.width > image.cols || mazeRegion.y + mazeRegion.height > image.rows) {
        std::cerr << "Error: Cropping region exceeds image bounds." << std::endl;
        return image;
    }

    // Crop the image to the detected maze area
    croppedImage = image(mazeRegion);
    return croppedImage;
}

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

std::vector<cv::Vec4i> ImageProcessor::detectMazeWalls(const cv::Mat& binaryImage) {
    std::vector<cv::Vec4i> lines;

    // Use Hough Transform to detect lines in the binary image
    cv::HoughLinesP(binaryImage, lines, 1, CV_PI / 180, 100, 50, 10);

    return lines;
}

/*
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
*/
std::vector<std::string> ImageProcessor::generateMazeArray(const std::vector<cv::Vec4i>& walls, const cv::Mat& binaryImage) {
    const int MAZE_SIZE = 19; // Fixed grid
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
    float threshold = 5.0f; // Adjust this value based on your needs
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

std::pair<bool, bool> ImageProcessor::detectStartEndPoints(std::vector<std::string>& maze, const int mazeSize) {
    bool startFound = false;
    bool endFound = false;

    // Find start point (gap in top or left wall)
    // Check top wall
    for (int j = 0; j < mazeSize && !startFound; j++) {
        if (maze[0][j] == '.' && maze[1][j] == '.') {
            maze[0][j] = 'S';
            startFound = true;
        }
    }
    // Check left wall if start not found
    for (int i = 0; i < mazeSize && !startFound; i++) {
        if (maze[i][0] == '.' && maze[i][1] == '.') {
            maze[i][0] = 'S';
            startFound = true;
        }
    }

    // Find end point (gap in bottom or right wall)
    // Check bottom wall
    for (int j = 0; j < mazeSize && !endFound; j++) {
        if (maze[mazeSize-1][j] == '.' && maze[mazeSize-2][j] == '.') {
            maze[mazeSize-1][j] = 'E';
            endFound = true;
        }
    }
    // Check right wall if end not found
    for (int i = 0; i < mazeSize && !endFound; i++) {
        if (maze[i][mazeSize-1] == '.' && maze[i][mazeSize-2] == '.') {
            maze[i][mazeSize-1] = 'E';
            endFound = true;
        }
    }

    return {startFound, endFound};
}