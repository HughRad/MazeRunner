/**
 * @file image_processing.h
 * @brief Header file for image processing operations for maze detection
 * 
 * This header defines the ImageProcessor class for processing images of mazes
 * captured by a camera. It includes functions for detecting maze walls, paths,
 * and start/end points.
 * 
 * @author Original author
 * @date May 2025
 */

#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>

/**
 * @class ImageProcessor
 * @brief Class for processing the image of the maze
 * 
 * Provides functionality to detect and process maze images, identify walls,
 * paths, and start/end points, and convert the visual information into
 * a 2D array representation of the maze.
 */
class ImageProcessor {
public:
    /**
     * @struct DebugInfo
     * @brief Structure to hold debugging information
     * 
     * Contains various images and data used during the maze processing
     * for visualization and debugging purposes.
     */
    struct DebugInfo {
        cv::Mat image;         ///< Original input image
        cv::Mat binaryImage;   ///< Binary processed image
        cv::Mat wallsImage;    ///< Image with detected walls highlighted
        std::vector<cv::Vec4i> walls;  ///< Detected wall segments
    };

    /**
     * @brief Default constructor for the ImageProcessor class
     */
    ImageProcessor();

    /**
     * @brief Function that identifies the maze structure from an image
     * 
     * @param image The input image containing the maze
     * @param debugInfo Optional pointer to store debug information and images
     * @return std::vector<std::string> 2D array representation of the maze
     */
    std::vector<std::string> processMaze(const cv::Mat& image, DebugInfo* debugInfo = nullptr);

private:
    /**
     * @brief Function that crops the image to the maze boundaries
     * 
     * Detects ArUco markers in the image and uses them to identify
     * the position and boundaries of the maze.
     * 
     * @param image The input image
     * @return cv::Mat The cropped image containing only the maze
     */
    cv::Mat cropToMazeBoundaries(const cv::Mat& image);

    /**
     * @brief Function that preprocesses the image to generate a clean binary image
     * 
     * Converts to grayscale, applies blur, and performs thresholding
     * to create a binary image suitable for wall detection.
     * 
     * @param croppedImage The cropped input image
     * @return cv::Mat The binary image of the maze
     */
    cv::Mat preprocessImage(const cv::Mat& croppedImage);

    /**
     * @brief Function that detects the walls of the maze
     * 
     * Uses Hough transform to detect line segments in the binary image
     * that represent the walls of the maze.
     * 
     * @param binaryImage The binary image of the maze
     * @return std::vector<cv::Vec4i> A vector of line segments representing maze walls
     */
    std::vector<cv::Vec4i> detectMazeWalls(const cv::Mat& binaryImage);

    /**
     * @brief Function that generates a grid array from the wall segments
     * 
     * Converts the detected wall segments into a grid representation of the maze,
     * using a fixed size grid. Also identifies potential path cells.
     * 
     * @param walls The detected wall segments of the maze
     * @param binaryImage The binary image of the maze
     * @return std::vector<std::string> A vector of strings representing the maze structure
     */
    std::vector<std::string> generateMazeArray(const std::vector<cv::Vec4i>& walls, const cv::Mat& binaryImage);

    /**
     * @brief Detects start and end points in the maze
     * 
     * Looks for openings in the maze boundary walls to set start (S) and end (E) points.
     * Checks all four walls in priority order.
     * 
     * @param maze Reference to the maze array to modify
     * @param mazeSize Size of the maze grid
     * @return std::pair<bool, bool> Flags indicating if start and end points were found
     */
    std::pair<bool, bool> detectStartEndPoints(std::vector<std::string>& maze, const int mazeSize);
};

#endif // IMAGE_PROCESSING_H
