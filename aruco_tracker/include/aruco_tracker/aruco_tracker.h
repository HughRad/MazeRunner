#ifndef ARUCO_TRACKER_H
#define ARUCO_TRACKER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ArucoTracker
{
public:
  ArucoTracker(ros::NodeHandle& nh);
  ~ArucoTracker();

private:
  // ROS node handle
  ros::NodeHandle nh_;
  
  // Image transport
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  // Camera info subscriber
  ros::Subscriber camera_info_sub_;
  
  // Velocity publisher
  ros::Publisher velocity_pub_;
  
  // ArUco dictionary and parameters
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  
  // Camera parameters
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  
  // Desired Z distance
  double desired_z_;
  
  // Image callback
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  
  // Camera info callback
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
  
  // Process frame and detect ArUco markers
  void processFrame(const cv::Mat& frame, cv_bridge::CvImagePtr& cv_ptr);
  
  // Calculate velocity to centre on marker
  geometry_msgs::Twist calculateVelocity(const std::vector<cv::Point2f>& corners, const cv::Point2f& centre);
  
  // Calculate rotation to align with marker
  double calculateRotation(const std::vector<cv::Point2f>& corners);
  
  // Draw visualisation
  void drawVisualisation(cv_bridge::CvImagePtr& cv_ptr, const std::vector<cv::Point2f>& corners, 
                         const cv::Point2f& centre, const geometry_msgs::Twist& velocity, double rotation);
};

#endif // ARUCO_TRACKER_H