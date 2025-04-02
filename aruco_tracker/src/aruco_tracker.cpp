#include <aruco_tracker/aruco_tracker.h>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>
#include <ros/package.h>

ArucoTracker::ArucoTracker(ros::NodeHandle& nh) : nh_(nh), it_(nh), snapshot_taken_(false), current_depth_(0.0), current_rotation_(0.0)
{
  // Get parameters
  nh_.param<double>("desired_z", desired_z_, 0.3); // Default 0.5 meters
  
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
  
  // Publish processed image
  image_pub_ = it_.advertise("aruco_tracker/output_image", 1);
  
  // Publish snapshot image
  snapshot_pub_ = it_.advertise("aruco_tracker/snapshot", 1, true); // Latched publisher
  
  // Publish velocity commands
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("aruco_tracker/velocity", 1);
  
  // Publish rotation value
  rotation_pub_ = nh_.advertise<std_msgs::Float64>("aruco_tracker/rotation", 1);
  
  // Initialize velocity
  current_velocity_.linear.x = 0;
  current_velocity_.linear.y = 0;
  current_velocity_.linear.z = 0;
  current_velocity_.angular.x = 0;
  current_velocity_.angular.y = 0;
  current_velocity_.angular.z = 0;
  
  ROS_INFO("ArUco Tracker initialized for tracking two markers");
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
    
    // ROS_INFO("Current depth: %.3f m", current_depth_);
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
    
    // Publish velocity and rotation data
    velocity_pub_.publish(current_velocity_);
    
    std_msgs::Float64 rotation_msg;
    rotation_msg.data = current_rotation_;
    rotation_pub_.publish(rotation_msg);
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
      ROS_WARN("Markers are not in expected positions (top-left and bottom-right)");
    }
    
    // Calculate the center between specific corners
    cv::Point2f target = calculateCenterBetweenMarkers(top_left_corners, bottom_right_corners);
    
    // Calculate velocity
    current_velocity_ = calculateVelocity(target, center);
    
    // Calculate rotation (average of both markers)
    current_rotation_ = calculateRotation(top_left_corners, bottom_right_corners);
    
    // Draw visualisation
    drawVisualization(cv_ptr, top_left_corners, bottom_right_corners, target, center, current_velocity_, current_rotation_);
    
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
    // Reset velocity and rotation if no markers are detected
    current_velocity_.linear.x = 0;
    current_velocity_.linear.y = 0;
    current_velocity_.linear.z = 0;
    current_velocity_.angular.x = 0;
    current_velocity_.angular.y = 0;
    current_velocity_.angular.z = 0;
    current_rotation_ = 0.0;
    
    // Publish zeroed data
    velocity_pub_.publish(current_velocity_);
    std_msgs::Float64 rotation_msg;
    rotation_msg.data = current_rotation_;
    rotation_pub_.publish(rotation_msg);
    
    // Display warning if not enough markers are detected
    std::string warning = "Need at least 2 ArUco markers";
    cv::putText(cv_ptr->image, warning, cv::Point(20, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
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

geometry_msgs::Twist ArucoTracker::calculateVelocity(const cv::Point2f& target, const cv::Point2f& center)
{
  // Calculate vector from center to target
  cv::Point2f direction = target - center;
  
  // Create velocity message
  geometry_msgs::Twist vel;
  
  // Scale for better control (these can be adjusted)
  float scale_x = 0.001f;
  float scale_y = 0.001f;
  float scale_z = 0.3f; // For Z-axis control using depth
  
  // Calculate velocity components
  vel.linear.x = -direction.y * scale_y; // Forward/backward (depends on camera orientation)
  vel.linear.y = -direction.x * scale_x; // Left/right
  
  // Z-axis velocity based on depth sensor
  if (current_depth_ > 0.01) { // Valid depth reading
    vel.linear.z = scale_z * (desired_z_ - current_depth_);
  } else {
    vel.linear.z = 0; // No valid depth, don't move in Z
  }
  
  return vel;
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
    const geometry_msgs::Twist& velocity, 
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
  
  // Display velocity info
  std::stringstream ss_velocity;
  ss_velocity << "Velocity - X: " << std::fixed << std::setprecision(3) << velocity.linear.x
             << ", Y: " << velocity.linear.y
             << ", Z: " << velocity.linear.z;
  cv::putText(cv_ptr->image, ss_velocity.str(), cv::Point(20, 60), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
  
  // Display depth info
  std::stringstream ss_depth;
  ss_depth << "Current depth: " << std::fixed << std::setprecision(3) << current_depth_ 
          << " m (Target: " << desired_z_ << " m)";
  cv::putText(cv_ptr->image, ss_depth.str(), cv::Point(20, 90), 
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
  
  // Draw the rectangular area between the markers
  cv::Point2f bottom_right_corner1 = corners1[2]; // Bottom-right of top-left marker
  cv::Point2f top_left_corner2 = corners2[0];     // Top-left of bottom-right marker
  
  // Draw rectangle defined by these two corners
  cv::rectangle(cv_ptr->image, bottom_right_corner1, top_left_corner2, cv::Scalar(255, 165, 0), 2);
  
  // Draw the snapshot status
  std::string snapshot_text = snapshot_taken_ ? "Snapshot: TAKEN" : "Snapshot: WAITING FOR ALIGNMENT";
  cv::putText(cv_ptr->image, snapshot_text, cv::Point(20, 120), 
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