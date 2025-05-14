#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>

/*!
 * @class ImageProcessor
 * @brief Class for processing the image of the maze
 */
class ImageProcessor {
public:
    /*!
     * @struct DebugInfo
     * @brief Structure to hold debugging information
     */
    struct DebugInfo {
        cv::Mat image;
        cv::Mat binaryImage;
        cv::Mat wallsImage;
        std::vector<cv::Vec4i> walls;
    };

    ImageProcessor();

    /**
     * @brief Function that identifies the maze structure from an image
     * @param imagePath path to the image file // changed by corso to take cv::Mat directly for integration
     * @return a vector of strings representing the maze structure
     */
    std::vector<std::string> processMaze(const cv::Mat& image, DebugInfo* debugInfo = nullptr);

    private:
    /**
     * @brief Function that crops the image to the maze boundaries
     * @param image the input image
     * @return the cropped image containing only the maze
     */
    cv::Mat cropToMazeBoundaries(const cv::Mat& image);

    /**
     * @brief Function that preprocesses the image to generate a clean binary image
     * @param croppedImage the cropped input image
     * @return the binary image of the maze
     */
    cv::Mat preprocessImage(const cv::Mat& croppedImage);

    /**
     * @brief Function that detects the walls of the maze
     * @param binaryImage the binary image of the maze
     * @return a vector of line segments representing maze walls
     */
    std::vector<cv::Vec4i> detectMazeWalls(const cv::Mat& binaryImage);

    /**
     * @brief Function that generates a grid array from the wall segments
     * @param walls the detected wall segments of the maze
     * @param binaryImage the binary image of the maze
     * @return a vector of strings representing the maze structure
     */
    std::vector<std::string> generateMazeArray(const std::vector<cv::Vec4i>& walls, const cv::Mat& binaryImage);

    /**
     * @brief Detects start and end points in the maze
     * @param maze Reference to the maze array
     * @param mazeSize Size of the maze grid
     * @return std::pair<bool, bool> indicating if start and end were found
     */
    std::pair<bool, bool> detectStartEndPoints(std::vector<std::string>& maze, const int mazeSize);
};

#endif // IMAGE_PROCESSING_H