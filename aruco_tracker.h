#ifndef ARUCO_TRACKER_H
#define ARUCO_TRACKER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>  // Changed from PoseStamped to Point
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
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
  image_transport::Publisher snapshot_pub_;  // Publisher for snapshot image
  
  // Camera info subscriber
  ros::Subscriber camera_info_sub_;
  
  // Depth image subscriber
  ros::Subscriber depth_sub_;
  
  // Robot position subscribers
  ros::Subscriber end_effector_sub_;   // Subscriber for end effector pose
  
  // Publishers
  ros::Publisher waypoint_pub_;        // Changed to publish Point instead of PoseStamped
  ros::Publisher rotation_pub_;        // Publisher for rotation
  
  // ArUco dictionary and parameters
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  
  // Camera parameters
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  
  // Desired Z distance
  double desired_z_;
  
  // Current rotation
  double current_rotation_;
  
  // Robot poses
  geometry_msgs::Pose end_effector_pose_;// End effector pose
  
  // Generated waypoint
  geometry_msgs::Point waypoint_;      // Changed from PoseStamped to Point
  
  // Flags
  bool snapshot_taken_;
  bool waypoint_generated_;  // Flag to track if waypoint has been generated
  
  // Current depth at target point
  float current_depth_;
  
  // Path for saving snapshots
  std::string snapshot_folder_;
  std::string snapshot_path_;
  
  // Image callback
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  
  // Depth image callback
  void depthCallback(const sensor_msgs::ImageConstPtr& msg);
  
  // Camera info callback
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
  
  // Robot pose callbacks
  void endEffectorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  
  // Process frame and detect ArUco markers
  void processFrame(const cv::Mat& frame, cv_bridge::CvImagePtr& cv_ptr, const cv::Mat& original_frame);
  
  // Calculate center between two ArUco markers
  cv::Point2f calculateCenterBetweenMarkers(
      const std::vector<cv::Point2f>& corners1, 
      const std::vector<cv::Point2f>& corners2);
  
  // Generate waypoint from target point - modified to return Point instead of PoseStamped
  geometry_msgs::Point generateWaypoint(
      const cv::Point2f& target, 
      float depth);
  
  // Calculate rotation based on both markers
  double calculateRotation(
      const std::vector<cv::Point2f>& corners1,
      const std::vector<cv::Point2f>& corners2);
  
  // Draw visualisation
  void drawVisualization(
      cv_bridge::CvImagePtr& cv_ptr, 
      const std::vector<cv::Point2f>& corners1,
      const std::vector<cv::Point2f>& corners2,
      const cv::Point2f& target,
      const cv::Point2f& center, 
      const geometry_msgs::Point& waypoint,  // Changed from PoseStamped to Point
      double rotation);
  
  // Save snapshot when aligned
  bool saveSnapshot(const cv::Mat& image);
  
  // Check if camera is aligned with target
  bool isAligned(const cv::Point2f& target, const cv::Point2f& center);
};

#endif // ARUCO_TRACKER_H
