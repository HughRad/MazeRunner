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

    ImageProcessor();

    /**
     * @brief Function that identifies the maze structure from an image
     * @param imagePath path to the image file
     * @return a vector of strings representing the maze structure
     */
    std::vector<std::string> processMaze(const std::string &imagePath);

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
     * @brief Function that generates a grid array from the preprocessed input image
     * @param binaryImage the binary image of the maze
     * @return a vector of strings representing the maze structure
     */
    std::vector<std::string> generateMazeArray(const cv::Mat& binaryImage);

    /**
     * @brief Detects start and end points in the maze
     * @param maze Reference to the maze array
     * @param mazeSize Size of the maze grid
     * @return std::pair<bool, bool> indicating if start and end were found
     */
    std::pair<bool, bool> detectStartEndPoints(std::vector<std::string>& maze, const int mazeSize);
};

#endif // IMAGE_PROCESSING_H