/**
 * @file aruco_tracker.h
 * @brief Header file for ArUco marker tracker
 * 
 * This header defines the ArucoTracker class that tracks ArUco markers using
 * a camera and generates waypoints for robot navigation. It includes functions
 * for marker detection, pose estimation, and visualization.
 * 
 * @author Original author
 * @date May 2025
 */

#ifndef ARUCO_TRACKER_H
#define ARUCO_TRACKER_H
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * @class ArucoTracker
 * @brief Class for tracking ArUco markers and generating waypoints
 * 
 * Provides functionality to detect ArUco markers in camera images,
 * calculate waypoints for robot navigation, and visualize the tracking process.
 */
class ArucoTracker
{
public:
  /**
   * @brief Constructor for the ArucoTracker class
   * 
   * @param nh ROS node handle for communication
   */
  ArucoTracker(ros::NodeHandle& nh);
  
  /**
   * @brief Destructor for the ArucoTracker class
   */
  ~ArucoTracker();

private:
  /**
   * @brief ROS node handle
   */
  ros::NodeHandle nh_;
  
  /**
   * @brief Image transport instance for handling image topics
   */
  image_transport::ImageTransport it_;
  
  /**
   * @brief Subscriber for camera image feed
   */
  image_transport::Subscriber image_sub_;
  
  /**
   * @brief Publisher for visualization image
   */
  image_transport::Publisher image_pub_;
  
  /**
   * @brief Publisher for snapshot image
   */
  image_transport::Publisher snapshot_pub_;
  
  /**
   * @brief Subscriber for camera calibration information
   */
  ros::Subscriber camera_info_sub_;
  
  /**
   * @brief Subscriber for depth image
   */
  ros::Subscriber depth_sub_;
  
  /**
   * @brief Subscriber for robot end effector pose
   */
  ros::Subscriber end_effector_sub_;
  
  /**
   * @brief Publisher for target waypoint
   */
  ros::Publisher waypoint_pub_;
  
  /**
   * @brief Publisher for marker rotation angle
   */
  ros::Publisher rotation_pub_;
  
  /**
   * @brief Publisher for corner waypoint
   */
  ros::Publisher corner_waypoint_pub_;
  
  /**
   * @brief Service server for starting the tracker
   */
  ros::ServiceServer start_service_;
  
  /**
   * @brief ArUco marker dictionary
   */
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  
  /**
   * @brief ArUco detector parameters
   */
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  
  /**
   * @brief Camera calibration matrix
   */
  cv::Mat camera_matrix_;
  
  /**
   * @brief Camera distortion coefficients
   */
  cv::Mat dist_coeffs_;
  
  /**
   * @brief Desired Z distance from camera to markers
   */
  double desired_z_;
  
  /**
   * @brief Current rotation angle of markers
   */
  double current_rotation_;
  
  /**
   * @brief Current pose of robot end effector
   */
  geometry_msgs::Pose end_effector_pose_;
  
  /**
   * @brief Generated waypoint for robot navigation
   */
  geometry_msgs::Point waypoint_;
  
  /**
   * @brief Generated corner waypoint for maze origin
   */
  geometry_msgs::Point corner_waypoint_;
  
  /**
   * @brief Flag indicating if snapshot has been taken
   */
  bool snapshot_taken_;
  
  /**
   * @brief Flag indicating if waypoint has been generated
   */
  bool waypoint_generated_;
  
  /**
   * @brief Flag indicating if rotation angle has been fixed
   */
  bool rotation_fixed_;
  
  /**
   * @brief Flag indicating if tracker is active
   */
  bool is_active_;
  
  /**
   * @brief Flag indicating if camera is aligned with target
   */
  bool is_aligned_;
  
  /**
   * @brief Flag indicating if corner waypoint has been generated
   */
  bool corner_waypoint_generated_;
  
  /**
   * @brief Current depth value at target point
   */
  float current_depth_;
  
  /**
   * @brief Path for saving snapshot images
   */
  std::string snapshot_folder_;
  std::string snapshot_path_;
  
  /**
   * @brief Time when markers were first detected
   */
  ros::Time markers_first_detected_time_;
  
  /**
   * @brief Flag indicating if markers are being tracked
   */
  bool markers_being_tracked_;
  
  /**
   * @brief Delay time before generating waypoint after detecting markers
   */
  double marker_tracking_delay_;
  
  /**
   * @brief Time when camera was first aligned with target
   */
  ros::Time first_aligned_time_;
  
  /**
   * @brief Delay time before taking snapshot after alignment
   */
  double snapshot_delay_;

  /**
   * @brief Callback for camera image
   * 
   * @param msg Image message from camera
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  
  /**
   * @brief Callback for depth image
   * 
   * @param msg Depth image message from camera
   */
  void depthCallback(const sensor_msgs::ImageConstPtr& msg);
  
  /**
   * @brief Callback for camera calibration information
   * 
   * @param msg Camera info message
   */
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
  
  /**
   * @brief Callback for robot end effector pose
   * 
   * @param msg Pose message from robot
   */
  void endEffectorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  
  /**
   * @brief Callback for start service
   * 
   * @param req Service request
   * @param res Service response
   * @return bool True if service call succeeded
   */
  bool startServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  
  /**
   * @brief Processes frame to detect ArUco markers
   * 
   * @param frame Input image frame
   * @param cv_ptr OpenCV image pointer for output visualization
   * @param original_frame Original unmodified frame for snapshot
   */
  void processFrame(const cv::Mat& frame, cv_bridge::CvImagePtr& cv_ptr, const cv::Mat& original_frame);
  
  /**
   * @brief Calculates center point between two ArUco markers
   * 
   * @param corners1 Corner points of first marker
   * @param corners2 Corner points of second marker
   * @return cv::Point2f Center point between markers
   */
  cv::Point2f calculateCenterBetweenMarkers(
      const std::vector<cv::Point2f>& corners1, 
      const std::vector<cv::Point2f>& corners2);
  
  /**
   * @brief Generates waypoint from target point in image
   * 
   * @param target Target point in image coordinates
   * @param depth Depth value at target point
   * @return geometry_msgs::Point Waypoint in world coordinates
   */
  geometry_msgs::Point generateWaypoint(
      const cv::Point2f& target, 
      float depth);
  
  /**
   * @brief Calculates rotation angle from marker orientations
   * 
   * @param corners1 Corner points of first marker
   * @param corners2 Corner points of second marker
   * @return double Rotation angle in degrees
   */
  double calculateRotation(
      const std::vector<cv::Point2f>& corners1,
      const std::vector<cv::Point2f>& corners2);
  
  /**
   * @brief Draws visualization elements on output image
   * 
   * @param cv_ptr OpenCV image pointer for visualization
   * @param corners1 Corner points of first marker
   * @param corners2 Corner points of second marker
   * @param target Target point in image
   * @param center Center of image
   * @param waypoint Generated waypoint
   * @param rotation Calculated rotation angle
   */
  void drawVisualization(
      cv_bridge::CvImagePtr& cv_ptr, 
      const std::vector<cv::Point2f>& corners1,
      const std::vector<cv::Point2f>& corners2,
      const cv::Point2f& target,
      const cv::Point2f& center, 
      const geometry_msgs::Point& waypoint,
      double rotation);
  
  /**
   * @brief Saves snapshot image
   * 
   * @param image Image to save
   * @return bool True if save was successful
   */
  bool saveSnapshot(const cv::Mat& image);
  
  /**
   * @brief Checks if camera is aligned with target
   * 
   * @param target Target point in image
   * @param center Center of image
   * @return bool True if aligned
   */
  bool isAligned(const cv::Point2f& target, const cv::Point2f& center);
  
  /**
   * @brief Rotates image without cropping
   * 
   * @param image Input image
   * @param angle Rotation angle in degrees
   * @return cv::Mat Rotated image
   */
  cv::Mat rotateImageWithoutCropping(const cv::Mat& image, double angle);
};

#endif // ARUCO_TRACKER_H
