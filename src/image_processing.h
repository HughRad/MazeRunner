#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/opencv.hpp>
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
     * @brief Function that preprocesses the image to extract the maze structure
     * @param image the input image
     * @return the binary image
     */
    cv::Mat preprocessImage(const cv::Mat& image);

    /**
     * @brief Function that detects the walls of the maze
     * @param binaryImage the binary image of the maze
     * @return a vector of line segments representing maze walls
     */
    std::vector<cv::Vec4i> detectMazeWalls(const cv::Mat& binaryImage);

    /**
     * @brief Function that generates the maze structure from the wall segments
     * @param walls the detected wall segments of the maze
     * @param binaryImage the binary image of the maze
     * @return a vector of strings representing the maze structure
     */
    std::vector<std::string> generateMazeArray(const std::vector<cv::Vec4i>& walls, const cv::Mat& binaryImage);
};

#endif // IMAGE_PROCESSING_H