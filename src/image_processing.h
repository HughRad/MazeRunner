#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class ImageProcessor {
public:
    ImageProcessor();
    std::vector<std::string> processMaze(const std::string &imagePath);

    private:
    cv::Mat preprocessImage(const cv::Mat& image);
    std::vector<cv::Point> detectGridPoints(const cv::Mat& binaryImage);
    std::vector<std::string> generateMazeArray(const std::vector<cv::Point>& gridPoints, const cv::Mat& binaryImage);
};

#endif // IMAGE_PROCESSING_H