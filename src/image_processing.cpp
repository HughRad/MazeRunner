#include "image_processing.h"

ImageProcessor::ImageProcessor(ros::NodeHandle& nh) : nh_(nh) {
    image_transport::ImageTransport it(nh_);
    image_sub_ = it.subscribe("/camera/color/image_raw", 1, &ImageProcessor::imageCallback, this);
    processed_image_pub_ = it.advertise("/maze_runner/processed_image", 1);
}

void ImageProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Mat processed = preprocessImage(image);
        cv::Mat maze = detectMaze(processed);
        
        // Convert back to ROS message and publish
        sensor_msgs::ImagePtr processed_msg = cv_bridge::CvImage(msg->header, "mono8", maze).toImageMsg();
        processed_image_pub_.publish(processed_msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

cv::Mat ImageProcessor::preprocessImage(const cv::Mat& image) {
    cv::Mat gray, blurred, edges;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::Canny(blurred, edges, 50, 150);
    return edges;
}

cv::Mat ImageProcessor::detectMaze(const cv::Mat& image) {
    cv::Mat thresholded;
    cv::threshold(image, thresholded, 128, 255, cv::THRESH_BINARY);
    return thresholded;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;
    ImageProcessor ip(nh);
    ros::spin();
    return 0;
}