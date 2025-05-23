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
    rotation_fixed_(false),
    is_active_(false),        // NEW: Initialize as inactive
    is_aligned_(false),       // NEW: Initialize alignment flag
    corner_waypoint_generated_(false),  // NEW: Initialize corner waypoint flag
    current_depth_(0.0), 
    current_rotation_(0.0),
    markers_being_tracked_(false),
    marker_tracking_delay_(5.0),
    desired_z_(0.4)
{
  // Get parameters
  nh_.param<double>("desired_z", desired_z_, 0.4); // Default 0.3 meters
  
  // Get the marker tracking delay parameter (if set)
  nh_.param<double>("marker_tracking_delay", marker_tracking_delay_, 2.0);
  
  // NEW: Get snapshot delay parameter (default to same as marker_tracking_delay)
  nh_.param<double>("snapshot_delay", snapshot_delay_, 2.0);
  
  ROS_INFO("Marker tracking delay set to %.1f seconds", marker_tracking_delay_);
  ROS_INFO("Snapshot delay after alignment set to %.1f seconds", snapshot_delay_);
  
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
  
  // NEW: Publish corner waypoint (as Point)
  corner_waypoint_pub_ = nh_.advertise<geometry_msgs::Point>("aruco_tracker/cornerwaypoint", 1, true);
  
  // NEW: Create service server
  start_service_ = nh_.advertiseService("aruco_tracker/start", &ArucoTracker::startServiceCallback, this);
  
  // Initialize end effector pose (default to identity)
  end_effector_pose_.position.x = 0.0;
  end_effector_pose_.position.y = 0.0;
  end_effector_pose_.position.z = 0.0;
  end_effector_pose_.orientation.w = 1.0;
  
  // Initialize waypoint
  waypoint_.x = 0.0;
  waypoint_.y = 0.0;
  waypoint_.z = 0.0;
  
  // NEW: Initialize corner waypoint
  corner_waypoint_.x = 0.0;
  corner_waypoint_.y = 0.0;
  corner_waypoint_.z = 0.0;
  
  ROS_INFO("ArUco Tracker initialized but inactive. Call 'aruco_tracker/start' service to begin tracking.");
}

ArucoTracker::~ArucoTracker()
{
}

// NEW: Service callback implementation
bool ArucoTracker::startServiceCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (!is_active_) {
    is_active_ = true;
    snapshot_taken_ = false;
    waypoint_generated_ = false;
    rotation_fixed_ = false;
    is_aligned_ = false;
    markers_being_tracked_ = false;
    corner_waypoint_generated_ = false;  // NEW: Reset corner waypoint flag
    ROS_INFO("ArUco Tracker activated by service call");
  } else {
    ROS_INFO("ArUco Tracker is already active");
  }
  return true;
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
  // Only process images if the tracker is active
  if (!is_active_) {
    return;
  }
  
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
  
  // MODIFIED: Check if both required markers (ID 0 and ID 3) are detected
  bool marker0_found = false;
  bool marker3_found = false;
  int marker0_idx = -1;
  int marker3_idx = -1;
  
  // Find the indices of markers with ID 0 and ID 3
  for (size_t i = 0; i < marker_ids.size(); i++) {
    if (marker_ids[i] == 0) {
      marker0_found = true;
      marker0_idx = i;
    } else if (marker_ids[i] == 3) {
      marker3_found = true;
      marker3_idx = i;
    }
  }
  
  // If both required markers are detected
  if (marker0_found && marker3_found) {
    // Draw all detected markers
    cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);
    
    // Start tracking time if not already tracking
    if (!markers_being_tracked_) {
      markers_being_tracked_ = true;
      markers_first_detected_time_ = ros::Time::now();
      ROS_INFO("Required markers (ID 0 and ID 3) detected - starting %.1f second tracking period", marker_tracking_delay_);
    }
    
    // Get corners for marker ID 0 (top-left) and marker ID 3 (bottom-right)
    std::vector<cv::Point2f> top_left_corners = marker_corners[marker0_idx];
    std::vector<cv::Point2f> bottom_right_corners = marker_corners[marker3_idx];
    
    // Calculate the center between specific corners
    cv::Point2f target = calculateCenterBetweenMarkers(top_left_corners, bottom_right_corners);
    
    // Calculate elapsed time since markers were first detected
    double elapsed_time = 0.0;
    if (markers_being_tracked_) {
      elapsed_time = (ros::Time::now() - markers_first_detected_time_).toSec();
    }
    
    // Generate waypoint only after the tracking delay and only if not already generated
    if (markers_being_tracked_ && elapsed_time >= marker_tracking_delay_ && !waypoint_generated_) {
      // Generate waypoint from target point and depth
      waypoint_ = generateWaypoint(target, current_depth_);
      waypoint_pub_.publish(waypoint_);
      waypoint_generated_ = true;
      
      ROS_INFO("Waypoint generated after %.1f seconds of tracking", elapsed_time);
    }
    
    // Always calculate current rotation for display
    // But don't fix it until snapshot is taken
    current_rotation_ = calculateRotation(top_left_corners, bottom_right_corners);
    
    // Draw visualisation
    drawVisualization(cv_ptr, top_left_corners, bottom_right_corners, target, center, waypoint_, current_rotation_);
    
    // Check for alignment and start delay if newly aligned
    bool is_currently_aligned = isAligned(target, center);
    
    // If we just became aligned, record the time
    if (is_currently_aligned && !is_aligned_) {
      is_aligned_ = true;
      first_aligned_time_ = ros::Time::now();
      ROS_INFO("Alignment detected - starting %.1f second snapshot delay", snapshot_delay_);
    }
    // If we lost alignment, reset the flag
    else if (!is_currently_aligned && is_aligned_) {
      is_aligned_ = false;
      ROS_INFO("Alignment lost - snapshot delay reset");
    }
    
    // Check if we've been aligned long enough to take a snapshot
    if (is_aligned_ && !snapshot_taken_) {
      double alignment_time = (ros::Time::now() - first_aligned_time_).toSec();
      
      // Add alignment time to visualization
      std::stringstream ss_alignment;
      ss_alignment << "Aligned: " << std::fixed << std::setprecision(1) 
                   << alignment_time << "s / " << snapshot_delay_ << "s";
      cv::putText(cv_ptr->image, ss_alignment.str(), cv::Point(20, 240), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
      
      // If we've been aligned for the delay period, take the snapshot
      if (alignment_time >= snapshot_delay_) {
        // Calculate and fix rotation at snapshot time
        current_rotation_ = calculateRotation(top_left_corners, bottom_right_corners);
        rotation_fixed_ = true;
        
        // Publish the rotation value
        std_msgs::Float64 rotation_msg;
        rotation_msg.data = current_rotation_;
        rotation_pub_.publish(rotation_msg);
        
        // MOVED: Generate corner waypoint when snapshot is taken
        if (!corner_waypoint_generated_) {
          // Get the bottom right corner of the marker with ID 0 (index 2 in the corners array)
          // ArUco corners are ordered: top-left, top-right, bottom-right, bottom-left
          cv::Point2f bottom_right_corner = top_left_corners[2];
          
          // Generate corner waypoint
          corner_waypoint_ = generateWaypoint(bottom_right_corner, current_depth_);
          corner_waypoint_pub_.publish(corner_waypoint_);
          corner_waypoint_generated_ = true;
          
          ROS_INFO("Corner waypoint generated for bottom-right corner of marker ID 0: [%.3f, %.3f, %.3f]",
                   corner_waypoint_.x, corner_waypoint_.y, corner_waypoint_.z);
        }
        
        // Use the original clean image for the snapshot
        if (saveSnapshot(original_frame)) {
          snapshot_taken_ = true;
          ROS_INFO("Snapshot taken after %.1f seconds of alignment!", alignment_time);
          
          // Publish the snapshot
          try {
            cv::Mat snapshot = cv::imread(snapshot_path_);
            if (!snapshot.empty()) {
              // Rotate the snapshot in the opposite direction of the ArUco markers
              // Negative sign to make the image right way up
              cv::Mat rotated_snapshot = rotateImageWithoutCropping(snapshot, current_rotation_);
              
              // Save the rotated image (optional)
              std::string rotated_path = snapshot_folder_ + "/rotated_snapshot.jpg";
              cv::imwrite(rotated_path, rotated_snapshot);
              
              // Publish the rotated snapshot
              sensor_msgs::ImagePtr snapshot_msg = 
                  cv_bridge::CvImage(std_msgs::Header(), "bgr8", rotated_snapshot).toImageMsg();
              snapshot_pub_.publish(snapshot_msg);
              ROS_INFO("Rotated snapshot published to topic (rotation: %.2f degrees)", current_rotation_);
            }
          } catch (const cv::Exception& e) {
            ROS_ERROR("Exception publishing snapshot: %s", e.what());
          }
        }
      }
    }
  } else {
    // MODIFIED: Display warning if the required markers are not detected
    std::string warning;
    if (!marker0_found && !marker3_found) {
      warning = "Need markers with ID 0 and ID 3";
    } else if (!marker0_found) {
      warning = "Need marker with ID 0";
    } else {
      warning = "Need marker with ID 3";
    }
    
    cv::putText(cv_ptr->image, warning, cv::Point(20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    
    // Reset the tracking flag if markers are lost
    if (markers_being_tracked_ && !waypoint_generated_) {
      markers_being_tracked_ = false;
      ROS_INFO("Required markers lost - tracking timer reset");
    }
    
    // Reset the alignment flag if markers are lost
    if (is_aligned_) {
      is_aligned_ = false;
      ROS_INFO("Required markers lost - alignment timer reset");
    }
  }
  
  // Display service status
  std::string service_status = is_active_ ? "ACTIVE" : "INACTIVE";
  cv::putText(cv_ptr->image, "Service: " + service_status, cv::Point(frame.cols - 200, 30), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, 
              is_active_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
  
  // Display marker detection status
  std::string marker0_status = marker0_found ? "FOUND" : "NOT FOUND";
  std::string marker3_status = marker3_found ? "FOUND" : "NOT FOUND";
  
  cv::putText(cv_ptr->image, "Marker ID 0: " + marker0_status, cv::Point(20, 330), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, 
              marker0_found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
  
  cv::putText(cv_ptr->image, "Marker ID 3: " + marker3_status, cv::Point(20, 360), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, 
              marker3_found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
  
  // Display corner waypoint status
  std::string corner_waypoint_status = corner_waypoint_generated_ ? "GENERATED" : "NOT GENERATED";
  cv::putText(cv_ptr->image, "Corner Waypoint: " + corner_waypoint_status, cv::Point(20, 270), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, 
              corner_waypoint_generated_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
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
  waypoint.z = desired_z_;
  
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
  
  // MODIFIED: Label the markers with their IDs
  cv::Point2f center1(0, 0), center2(0, 0);
  for (int i = 0; i < 4; i++) {
    center1 += corners1[i];
    center2 += corners2[i];
  }
  center1 *= 0.25f;
  center2 *= 0.25f;
  
  cv::putText(cv_ptr->image, "ID 0", cv::Point(center1.x - 10, center1.y - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
  cv::putText(cv_ptr->image, "ID 3", cv::Point(center2.x - 10, center2.y - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
  
  // Highlight the bottom-right corner of marker ID 0
  cv::Point2f bottom_right_corner = corners1[2]; // Corner index 2 is bottom-right
  cv::circle(cv_ptr->image, bottom_right_corner, 5, cv::Scalar(0, 0, 255), -1);
  cv::putText(cv_ptr->image, "Corner", cv::Point(bottom_right_corner.x + 5, bottom_right_corner.y + 5),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
  
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
  
  // Display tracking time or status
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
  cv::Point2f bottom_right_corner1 = corners1[2]; // Bottom-right of marker ID 0
  cv::Point2f top_left_corner2 = corners2[0];     // Top-left of marker ID 3
  
  // Draw rectangle defined by these two corners
  cv::rectangle(cv_ptr->image, bottom_right_corner1, top_left_corner2, cv::Scalar(255, 165, 0), 2);
  
  // Draw the snapshot status
  std::string snapshot_text = snapshot_taken_ ? "Snapshot: TAKEN" : "Snapshot: WAITING";
  cv::putText(cv_ptr->image, snapshot_text, cv::Point(20, 210), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, 
              snapshot_taken_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
  
  // Display alignment status
  std::string alignment_text = is_aligned_ ? "ALIGNED" : "NOT ALIGNED";
  cv::putText(cv_ptr->image, "Alignment: " + alignment_text, cv::Point(20, 240), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, 
              is_aligned_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255), 2);
  
  // Display corner waypoint info if generated
  if (corner_waypoint_generated_) {
    std::stringstream ss_corner_waypoint;
    ss_corner_waypoint << "Corner Waypoint - X: " << std::fixed << std::setprecision(3) << corner_waypoint_.x
                      << ", Y: " << corner_waypoint_.y
                      << ", Z: " << corner_waypoint_.z;
    cv::putText(cv_ptr->image, ss_corner_waypoint.str(), cv::Point(20, 300), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
  }
}

bool ArucoTracker::isAligned(const cv::Point2f& target, const cv::Point2f& center)
{
  // Calculate distance between target and center
  double distance = cv::norm(target - center);
  
  // Check if distance is below threshold (in pixels)
  const double alignment_threshold = 15.0; // pixels
  
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

cv::Mat ArucoTracker::rotateImageWithoutCropping(const cv::Mat& image, double angle)
{
  // Get image dimensions
  int width = image.cols;
  int height = image.rows;
  
  // Calculate the size needed for the rotated image to avoid cropping
  cv::Point2f center(width/2.0f, height/2.0f);
  cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
  
  // Determine new dimensions to avoid cropping
  // Calculate bounding rect for the rotated image
  cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), image.size(), angle).boundingRect2f();
  
  // Adjust the rotation matrix to move the image to the center of the new canvas
  rotation_matrix.at<double>(0,2) += bbox.width/2.0 - image.cols/2.0;
  rotation_matrix.at<double>(1,2) += bbox.height/2.0 - image.rows/2.0;
  
  // Create output image with proper dimensions to avoid cropping
  cv::Mat rotated_image;
  cv::warpAffine(image, rotated_image, rotation_matrix, bbox.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255, 255, 255));
  
  return rotated_image;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_tracker_node");
  ros::NodeHandle nh;
  
  ArucoTracker tracker(nh);
  
  ros::spin();
  
  return 0;
}