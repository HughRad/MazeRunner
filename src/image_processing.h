#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class ImageProcessor {
    public:
        ImageProcessor(ros::NodeHandle& nh);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        cv::Mat preprocessImage(const cv::Mat& image);
        cv::Mat detectMaze(const cv::Mat& image);
    private:
        ros::NodeHandle nh_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher processed_image_pub_;
};

#endif // IMAGE_PROCESSING_H