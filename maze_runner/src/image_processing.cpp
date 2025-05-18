#include "image_processing.h"
#include <iostream>

ImageProcessor::ImageProcessor() {}

std::vector<std::string> ImageProcessor::processMaze(const cv::Mat& image) {
    // Debug output
    if (image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return {};
    }

    // Crop the image to the maze boundaries
    cv::Mat croppedImage = cropToMazeBoundaries(image);

    // Preprocess image
    cv::Mat binaryImage = preprocessImage(croppedImage);

    // Generate the maze structure
    return generateMazeArray(binaryImage);
}

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
    float mazeSize = markerDistance * 0.565f;
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

cv::Mat ImageProcessor::preprocessImage(const cv::Mat& croppedImage) {
    cv::Mat resizedImage, gray, blurred, binary;

    // Resize image to a fixed size (optional)
    cv::resize(croppedImage, resizedImage, cv::Size(croppedImage.cols * 2.0, croppedImage.rows * 2.0), 0, 0, cv::INTER_LINEAR);

    // Convert to grayscale
    cv::cvtColor(resizedImage, gray, cv::COLOR_BGR2GRAY);

    // Reduce noise (blur) using a Gaussian filter (larger kernel size = more blur)
    cv::GaussianBlur(gray, blurred, cv::Size(25, 25), 0);

    // Convert grayscale image to binary (white for walls/black for paths)
    cv::adaptiveThreshold(blurred, binary, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, 3);
    return binary;
}

std::vector<std::string> ImageProcessor::generateMazeArray(const cv::Mat& binaryImage) {
    const int MAZE_SIZE = 17; // Fixed grid
    std::vector<std::string> maze(MAZE_SIZE, std::string(MAZE_SIZE, '.'));

    // Create debug visualisation image
    cv::Mat debugOverlay;
    cv::cvtColor(binaryImage, debugOverlay, cv::COLOR_GRAY2BGR);

    // Calculate cell dimensions
    float cellWidth = static_cast<float>(binaryImage.cols) / MAZE_SIZE;
    float cellHeight = static_cast<float>(binaryImage.rows) / MAZE_SIZE;

    // For each cell in the grid
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            // Calculate cell boundaries
            int startX = static_cast<int>(j * cellWidth);
            int startY = static_cast<int>(i * cellHeight);
            int endX = static_cast<int>((j + 1) * cellWidth);
            int endY = static_cast<int>((i + 1) * cellHeight);

            // Ensure we don't exceed image boundaries
            endX = std::min(endX, binaryImage.cols);
            endY = std::min(endY, binaryImage.rows);

            // Extract the cell region
            cv::Mat cell = binaryImage(cv::Range(startY, endY), cv::Range(startX, endX));

            // Count white pixels
            int whitePixels = cv::countNonZero(cell);
            int totalPixels = cell.rows * cell.cols;
            float whiteRatio = static_cast<float>(whitePixels) / totalPixels;

            // Draw grid lines
            cv::rectangle(debugOverlay, cv::Point(startX, startY), 
                         cv::Point(endX, endY), cv::Scalar(0, 255, 0), 1);

            // Color code cells based on classification
            cv::Scalar cellColor;
            if (whiteRatio > 0.1) {
                maze[i][j] = '#';
                // Red tint for walls
                cellColor = cv::Scalar(0, 0, 200);
            } else {
                // Green tint for paths
                cellColor = cv::Scalar(0, 200, 0);
            }

            // Apply semi-transparent overlay
            cv::Mat roi = debugOverlay(cv::Range(startY, endY), cv::Range(startX, endX));
            cv::addWeighted(roi, 0.7, cv::Mat(roi.size(), roi.type(), cellColor), 0.3, 0, roi);

            // Add text showing white ratio percentage
            std::string ratioText = std::to_string(int(whiteRatio * 100)) + "%";
            cv::putText(debugOverlay, ratioText, 
                       cv::Point(startX + 5, startY + 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                       cv::Scalar(255, 255, 255), 1);
        }
    }

    // Show debug visualisation
    cv::imshow("Maze Grid Analysis", debugOverlay);

    // Detect start and end points
    auto [startFound, endFound] = detectStartEndPoints(maze, MAZE_SIZE);

    // Debug output
    if (!startFound) std::cout << "Warning: No start point found!" << std::endl;
    if (!endFound) std::cout << "Warning: No end point found!" << std::endl;

    // Print maze representation
    for (const auto &row : maze) {
        std::cout << row << std::endl;
    }

    return maze;
}

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
    for (int i = 0; i < mazeSize; i++) {
        if (maze[i][0] == '.' && maze[i][1] == '.') {
            maze[i][0] = 'E';
            endFound = true;
        }
    }

    // Check top wall for end
    for (int j = 0; j < mazeSize; j++) {
        if (maze[0][j] == '.' && maze[1][j] == '.') {
            maze[0][j] = 'E';
            endFound = true;
        }
    }

    // Check right wall for end
    for (int i = 0; i < mazeSize; i++) {
        if (maze[i][mazeSize-1] == '.' && maze[i][mazeSize-2] == '.') {
            maze[i][mazeSize-1] = 'E';
            endFound = true;
        }
    }

    // Check bottom wall for end
    for (int j = 0; j < mazeSize; j++) {
        if (maze[mazeSize-1][j] == '.' && maze[mazeSize-2][j] == '.') {
            maze[mazeSize-1][j] = 'E';
            endFound = true;
        }
    }

    return {startFound, endFound};
}
