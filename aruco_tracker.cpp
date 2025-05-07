#include <aruco_tracker/aruco_tracker.h>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

ArucoTracker::ArucoTracker(ros::NodeHandle& nh) : 
    nh_(nh), 
    it_(nh), 
    snapshot_taken_(false), 
    waypoint_generated_(false),
    rotation_fixed_(false),  // NEW: Initialize rotation fixed flag
    current_depth_(0.0), 
    current_rotation_(0.0),
    markers_being_tracked_(false),
    marker_tracking_delay_(5.0)  // NEW: Default 5 second delay
{
  // Get parameters
  nh_.param<double>("desired_z", desired_z_, 0.3); // Default 0.3 meters
  
  // NEW: Get the marker tracking delay parameter (if set)
  nh_.param<double>("marker_tracking_delay", marker_tracking_delay_, 5.0);
  ROS_INFO("Marker tracking delay set to %.1f seconds", marker_tracking_delay_);
  
  // Setup snapshot folder
  snapshot_folder_ = ros::package::getPath("aruco_tracker") + "/snapshot";
  
  // Create the snapshot directory if it doesn't exist
  struct stat st = {0};
  if (stat(snapshot_folder_.c_str(), &st) == -1) {
    mkdir(snapshot_folder_.c_str(), 0700);
    ROS_INFO("Created snapshot directory: %s", snapshot_folder_.c_str());
  }
  
  snapshot_path_ = snapshot_folder_ + "/current_snapshot.jpg";
  
  // Set up ArUco detector
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  parameters_ = cv::aruco::DetectorParameters::create();
  
  // Subscribe to camera feed
  image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ArucoTracker::imageCallback, this);
  
  // Subscribe to depth feed
  depth_sub_ = nh_.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &ArucoTracker::depthCallback, this);
  
  // Subscribe to camera info
  camera_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &ArucoTracker::cameraInfoCallback, this);
  
  // Subscribe to robot end effector position
  end_effector_sub_ = nh_.subscribe("/robot/end_effector_pose", 1, &ArucoTracker::endEffectorCallback, this);
  
  // Publish processed image
  image_pub_ = it_.advertise("aruco_tracker/output_image", 1);
  
  // Publish snapshot image
  snapshot_pub_ = it_.advertise("aruco_tracker/snapshot", 1, true); // Latched publisher
  
  // Publish waypoint (as Point)
  waypoint_pub_ = nh_.advertise<geometry_msgs::Point>("aruco_tracker/waypoint", 1, true);
  
  // Publish rotation value
  rotation_pub_ = nh_.advertise<std_msgs::Float64>("aruco_tracker/rotation", 1, true);
  
  // Initialize end effector pose (default to identity)
  end_effector_pose_.position.x = 0.0;
  end_effector_pose_.position.y = 0.0;
  end_effector_pose_.position.z = 0.0;
  end_effector_pose_.orientation.w = 1.0;
  
  // Initialize waypoint
  waypoint_.x = 0.0;
  waypoint_.y = 0.0;
  waypoint_.z = 0.0;
  
  ROS_INFO("ArUco Tracker initialized with %.1f second delay before generating waypoint", marker_tracking_delay_);
}

ArucoTracker::~ArucoTracker()
{
}

void ArucoTracker::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  // Extract camera matrix and distortion coefficients
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      camera_matrix_.at<double>(i, j) = msg->K[i * 3 + j];
    }
  }
  
  dist_coeffs_ = cv::Mat(1, 5, CV_64F);
  for (int i = 0; i < 5; i++) {
    dist_coeffs_.at<double>(0, i) = msg->D[i];
  }
  
  // Unsubscribe after we have the camera info
  camera_info_sub_.shutdown();
  ROS_INFO("Camera calibration received");
}

void ArucoTracker::endEffectorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  end_effector_pose_ = msg->pose;
  ROS_INFO_ONCE("Received first end effector pose");
}

void ArucoTracker::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    // Convert ROS depth image to OpenCV image
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    
    // Get frame center
    cv::Point2f center(cv_ptr->image.cols / 2.0f, cv_ptr->image.rows / 2.0f);
    
    // Get depth at center (convert from mm to m if needed)
    uint16_t depth_value = cv_ptr->image.at<uint16_t>(center.y, center.x);
    
    // Convert to meters (RealSense typically returns depth in mm)
    current_depth_ = depth_value / 1000.0f;
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception in depth callback: %s", e.what());
  }
}

void ArucoTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    // Create a clean copy of the original image before any drawing
    cv::Mat original_image = cv_ptr->image.clone();
    
    // Process frame
    processFrame(cv_ptr->image, cv_ptr, original_image);
    
    // Publish visualised image
    image_pub_.publish(cv_ptr->toImageMsg());
    
    // Publish rotation data (only if it has been fixed)
    if (rotation_fixed_) {
      std_msgs::Float64 rotation_msg;
      rotation_msg.data = current_rotation_;
      rotation_pub_.publish(rotation_msg);
    }
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception in image callback: %s", e.what());
  }
}

void ArucoTracker::processFrame(const cv::Mat& frame, cv_bridge::CvImagePtr& cv_ptr, const cv::Mat& original_frame)
{
  // Detect markers
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, parameters_);
  
  // Get frame center
  cv::Point2f center(frame.cols / 2.0f, frame.rows / 2.0f);
  
  // Draw center crosshair
  cv::line(cv_ptr->image, cv::Point(center.x - 10, center.y), cv::Point(center.x + 10, center.y), cv::Scalar(0, 0, 255), 2);
  cv::line(cv_ptr->image, cv::Point(center.x, center.y - 10), cv::Point(center.x, center.y + 10), cv::Scalar(0, 0, 255), 2);
  
  // If at least two markers are detected
  if (marker_ids.size() >= 2) {
    // Draw all detected markers
    cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);
    
    // NEW: Start tracking time if not already tracking
    if (!markers_being_tracked_) {
      markers_being_tracked_ = true;
      markers_first_detected_time_ = ros::Time::now();
      ROS_INFO("Markers detected - starting %.1f second tracking period", marker_tracking_delay_);
    }
    
    // We need to identify which marker is top-left and which is bottom-right
    // For simplicity, we'll take the first two markers
    // Determine which is top-left and which is bottom-right based on center positions
    cv::Point2f center1(0, 0), center2(0, 0);
    
    for (const auto& corner : marker_corners[0]) {
      center1 += corner;
    }
    center1 *= 0.25f;
    
    for (const auto& corner : marker_corners[1]) {
      center2 += corner;
    }
    center2 *= 0.25f;
    
    // Determine which is top-left and which is bottom-right
    std::vector<cv::Point2f> top_left_corners, bottom_right_corners;
    
    if (center1.x <= center2.x && center1.y <= center2.y) {
      // Marker 1 is top-left, Marker 2 is bottom-right
      top_left_corners = marker_corners[0];
      bottom_right_corners = marker_corners[1];
    } else if (center2.x <= center1.x && center2.y <= center1.y) {
      // Marker 2 is top-left, Marker 1 is bottom-right
      top_left_corners = marker_corners[1];
      bottom_right_corners = marker_corners[0];
    } else {
      // If the markers are diagonal, we need a better criterion
      // For now, just use the first marker as top-left and second as bottom-right
      // and print a warning
      top_left_corners = marker_corners[0];
      bottom_right_corners = marker_corners[1];
      ROS_WARN_THROTTLE(2.0, "Markers are not in expected positions (top-left and bottom-right)");
    }
    
    // Calculate the center between specific corners
    cv::Point2f target = calculateCenterBetweenMarkers(top_left_corners, bottom_right_corners);
    
    // NEW: Calculate elapsed time since markers were first detected
    double elapsed_time = 0.0;
    if (markers_being_tracked_) {
      elapsed_time = (ros::Time::now() - markers_first_detected_time_).toSec();
    }
    
    // Generate waypoint and fix rotation only after the tracking delay
    // and only if not already generated
    if (markers_being_tracked_ && elapsed_time >= marker_tracking_delay_ && !waypoint_generated_) {
      // Generate waypoint from target point and depth
      waypoint_ = generateWaypoint(target, current_depth_);
      waypoint_pub_.publish(waypoint_);
      waypoint_generated_ = true;
      
      // Fix rotation value
      current_rotation_ = calculateRotation(top_left_corners, bottom_right_corners);
      rotation_fixed_ = true;
      
      // Publish the rotation value
      std_msgs::Float64 rotation_msg;
      rotation_msg.data = current_rotation_;
      rotation_pub_.publish(rotation_msg);
      
      ROS_INFO("Waypoint and rotation generated after %.1f seconds of tracking", elapsed_time);
    } 
    // If waypoint not yet generated, calculate temporary rotation for display
    else if (!rotation_fixed_) {
      current_rotation_ = calculateRotation(top_left_corners, bottom_right_corners);
    }
    
    // Draw visualisation
    drawVisualization(cv_ptr, top_left_corners, bottom_right_corners, target, center, waypoint_, current_rotation_);
    
    // Check if we should take a snapshot
    if (!snapshot_taken_ && isAligned(target, center)) {
      // Use the original clean image for the snapshot instead of the one with UI elements
      if (saveSnapshot(original_frame)) {
        snapshot_taken_ = true;
        ROS_INFO("Snapshot saved successfully!");
        
        // Publish the snapshot
        try {
          cv::Mat snapshot = cv::imread(snapshot_path_);
          if (!snapshot.empty()) {
            sensor_msgs::ImagePtr snapshot_msg = 
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", snapshot).toImageMsg();
            snapshot_pub_.publish(snapshot_msg);
            ROS_INFO("Snapshot published to topic");
          }
        } catch (const cv::Exception& e) {
          ROS_ERROR("Exception publishing snapshot: %s", e.what());
        }
      }
    }
  } else {
    // Display warning if not enough markers are detected
    std::string warning = "Need at least 2 ArUco markers";
    cv::putText(cv_ptr->image, warning, cv::Point(20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    
    // Reset the tracking flag if markers are lost
    if (markers_being_tracked_ && !waypoint_generated_) {
      markers_being_tracked_ = false;
      ROS_INFO("Markers lost - tracking timer reset");
    }
  }
}

cv::Point2f ArucoTracker::calculateCenterBetweenMarkers(
    const std::vector<cv::Point2f>& corners1, 
    const std::vector<cv::Point2f>& corners2)
{
  // Get bottom right corner of top left marker (corners1)
  // ArUco corners are ordered: top-left, top-right, bottom-right, bottom-left
  cv::Point2f bottom_right_corner = corners1[2];
  
  // Get top left corner of bottom right marker (corners2)
  cv::Point2f top_left_corner = corners2[0];
  
  // Calculate center between these two corners
  cv::Point2f center = (bottom_right_corner + top_left_corner) * 0.5f;
  
  return center;
}

geometry_msgs::Point ArucoTracker::generateWaypoint(const cv::Point2f& target, float depth)
{
  geometry_msgs::Point waypoint;
  
  // Get camera intrinsics
  double fx = camera_matrix_.at<double>(0, 0);
  double fy = camera_matrix_.at<double>(1, 1);
  double cx = camera_matrix_.at<double>(0, 2);
  double cy = camera_matrix_.at<double>(1, 2);
  
  // Calculate 3D point in camera frame
  double x_camera = (target.x - cx) * depth / fx;
  double y_camera = (target.y - cy) * depth / fy;
  double z_camera = depth;
  
  // Create a transformation from end effector to camera frame
  tf2::Transform end_effector_to_camera;
  end_effector_to_camera.setIdentity(); // Modify if camera has offset from end effector
  
  // Create point in camera frame
  tf2::Vector3 point_in_camera(x_camera, y_camera, z_camera);
  
  // Convert end effector pose to tf2 format
  tf2::Transform end_effector_to_world;
  tf2::fromMsg(end_effector_pose_, end_effector_to_world);
  
  // Transform point from camera to world frame
  tf2::Vector3 point_in_world = end_effector_to_world * (end_effector_to_camera * point_in_camera);
  
  // Set waypoint position only (no orientation)
  waypoint.x = point_in_world.x();
  waypoint.y = point_in_world.y();
  waypoint.z = point_in_world.z();
  
  // Log waypoint information
  ROS_INFO("Generated waypoint at [%.3f, %.3f, %.3f]",
           waypoint.x, waypoint.y, waypoint.z);
  
  return waypoint;
}

double ArucoTracker::calculateRotation(
    const std::vector<cv::Point2f>& corners1,
    const std::vector<cv::Point2f>& corners2)
{
  // Calculate marker orientation for both markers
  cv::Point2f vec1 = corners1[1] - corners1[0]; // Vector from corner 0 to corner 1 of first marker
  cv::Point2f vec2 = corners2[1] - corners2[0]; // Vector from corner 0 to corner 1 of second marker
  
  // Calculate angles in degrees
  double angle1 = atan2(vec1.y, vec1.x) * 180.0 / CV_PI;
  double angle2 = atan2(vec2.y, vec2.x) * 180.0 / CV_PI;
  
  // Return average angle
  return (angle1 + angle2) / 2.0;
}

void ArucoTracker::drawVisualization(
    cv_bridge::CvImagePtr& cv_ptr, 
    const std::vector<cv::Point2f>& corners1,
    const std::vector<cv::Point2f>& corners2,
    const cv::Point2f& target,
    const cv::Point2f& center, 
    const geometry_msgs::Point& waypoint, 
    double rotation)
{
  // Draw line from center to target point
  cv::line(cv_ptr->image, center, target, cv::Scalar(0, 255, 0), 2);
  
  // Draw the target point (center between specific corners)
  cv::circle(cv_ptr->image, target, 5, cv::Scalar(255, 0, 255), -1);
  
  // Calculate pixel distance
  double pixel_distance = cv::norm(target - center);
  
  // Display pixel distance (near center crosshair)
  std::stringstream ss_distance;
  ss_distance << "Distance: " << std::fixed << std::setprecision(1) << pixel_distance << " px";
  cv::putText(cv_ptr->image, ss_distance.str(), cv::Point(center.x + 15, center.y + 15), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
  
  // Display rotation
  std::stringstream ss_rotation;
  ss_rotation << "Rotation: " << std::fixed << std::setprecision(1) << rotation << " degrees";
  cv::putText(cv_ptr->image, ss_rotation.str(), cv::Point(20, 30), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
  
  // Display waypoint info
  std::stringstream ss_waypoint;
  ss_waypoint << "Waypoint - X: " << std::fixed << std::setprecision(3) << waypoint.x
             << ", Y: " << waypoint.y
             << ", Z: " << waypoint.z;
  cv::putText(cv_ptr->image, ss_waypoint.str(), cv::Point(20, 60), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
  
  // NEW: Display tracking time or status
  if (markers_being_tracked_ && !waypoint_generated_) {
    double elapsed_time = (ros::Time::now() - markers_first_detected_time_).toSec();
    double remaining_time = marker_tracking_delay_ - elapsed_time;
    if (remaining_time > 0) {
      std::stringstream ss_tracking;
      ss_tracking << "Tracking: " << std::fixed << std::setprecision(1) << elapsed_time 
                  << "s / " << marker_tracking_delay_ << "s";
      cv::putText(cv_ptr->image, ss_tracking.str(), cv::Point(20, 90), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
    } else {
      cv::putText(cv_ptr->image, "Generating waypoint...", cv::Point(20, 90), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    }
  }
  
  // Display waypoint and rotation status
  std::string waypoint_status = waypoint_generated_ ? "GENERATED" : "NOT GENERATED";
  cv::putText(cv_ptr->image, "Waypoint: " + waypoint_status, cv::Point(20, 120), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, 
              waypoint_generated_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
  
  std::string rotation_status = rotation_fixed_ ? "FIXED" : "TRACKING";
  cv::putText(cv_ptr->image, "Rotation: " + rotation_status, cv::Point(20, 150), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, 
              rotation_fixed_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
  
  // Display depth info
  std::stringstream ss_depth;
  ss_depth << "Current depth: " << std::fixed << std::setprecision(3) << current_depth_ 
          << " m (Target: " << desired_z_ << " m)";
  cv::putText(cv_ptr->image, ss_depth.str(), cv::Point(20, 180), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
  
  // Draw the rectangular area between the markers
  cv::Point2f bottom_right_corner1 = corners1[2]; // Bottom-right of top-left marker
  cv::Point2f top_left_corner2 = corners2[0];     // Top-left of bottom-right marker
  
  // Draw rectangle defined by these two corners
  cv::rectangle(cv_ptr->image, bottom_right_corner1, top_left_corner2, cv::Scalar(255, 165, 0), 2);
  
  // Draw the snapshot status
  std::string snapshot_text = snapshot_taken_ ? "Snapshot: TAKEN" : "Snapshot: WAITING FOR ALIGNMENT";
  cv::putText(cv_ptr->image, snapshot_text, cv::Point(20, 210), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, 
              snapshot_taken_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
}

bool ArucoTracker::isAligned(const cv::Point2f& target, const cv::Point2f& center)
{
  // Calculate distance between target and center
  double distance = cv::norm(target - center);
  
  // Check if distance is below threshold (in pixels)
  const double alignment_threshold = 10.0; // pixels
  
  // Also check if depth is close to desired depth
  const double depth_threshold = 0.05; // meters
  bool depth_ok = fabs(current_depth_ - desired_z_) < depth_threshold;
  
  return (distance < alignment_threshold) && depth_ok;
}

bool ArucoTracker::saveSnapshot(const cv::Mat& image)
{
  try {
    // Always save to the same file to overwrite previous snapshot
    bool success = cv::imwrite(snapshot_path_, image);
    
    if (!success) {
      ROS_ERROR("Failed to save snapshot image");
      return false;
    }
    
    ROS_INFO("Saved snapshot to: %s", snapshot_path_.c_str());
    return true;
  }
  catch (const cv::Exception& e) {
    ROS_ERROR("Exception saving snapshot: %s", e.what());
    return false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_tracker_node");
  ros::NodeHandle nh;
  
  ArucoTracker tracker(nh);
  
  ros::spin();
  
  return 0;
}
