/**
 * @file image_processing.h
 * @brief Header file for image processing operations for maze detection
 * 
 * This header defines the ImageProcessor class for processing images of mazes
 * captured by a camera. It includes functions for detecting maze walls, paths,
 * and start/end points.
 * 
 * @author Ryan Thomas
 * @date May 2025
 */

 #ifndef IMAGE_PROCESSING_H
 #define IMAGE_PROCESSING_H
 
 #include <opencv2/opencv.hpp>
 #include <opencv2/aruco.hpp>
 #include <vector>
 #include <string>

 #include <ros/ros.h>
 #include <sensor_msgs/Image.h>
 #include <cv_bridge/cv_bridge.h>
 
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
      * @brief Default constructor for the ImageProcessor class
      */
     ImageProcessor();
 
     /**
      * @brief Function that identifies the maze structure from an image
      * 
      * @param image The input image containing the maze
      * @return std::vector<std::string> 2D array representation of the maze
      */
     std::vector<std::string> processMaze(const cv::Mat& image);

        /**
        * @brief Function that gets the latest binary image for display
        * 
        * @param image The binary image to set
        */
     cv::Mat getLatestBinaryImage() {
        return latest_binary_image_;
      }

      /**
      * @brief Stores the latest binary image for debugging
      * 
      * This image is used for visualizing the maze structure after processing.
      */
     cv::Mat latest_binary_image_;
 
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
      * @brief Function that generates a grid array from the input image
      * 
      * Converts the binary image into a grid representation of the maze,
      * using a fixed size grid. Also identifies potential path cells.
      * 
      * @param binaryImage The binary image of the maze
      * @return std::vector<std::string> A vector of strings representing the maze structure
      */
     std::vector<std::string> generateMazeArray(const cv::Mat& binaryImage);
 
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


     ros::NodeHandle nh;
     ros::Publisher vis_pub;
 };
 
 #endif // IMAGE_PROCESSING_H
