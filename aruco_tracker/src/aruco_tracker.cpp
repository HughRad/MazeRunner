#include <aruco_tracker/aruco_tracker.h>
#include <iomanip>

ArucoTracker::ArucoTracker(ros::NodeHandle& nh) : nh_(nh), it_(nh)
{
  // Get parameters
  nh_.param<double>("desired_z", desired_z_, 0); // Default 0
  
  // Set up ArUco detector
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  parameters_ = cv::aruco::DetectorParameters::create();
  
  // Subscribe to camera feed
  image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ArucoTracker::imageCallback, this);
  
  // Subscribe to camera info
  camera_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &ArucoTracker::cameraInfoCallback, this);
  
  // Publish processed image
  image_pub_ = it_.advertise("aruco_tracker/output_image", 1);
  
  // Publish velocity commands
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  ROS_INFO("ArUco Tracker initialised");
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

void ArucoTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try {
    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    // Process frame
    processFrame(cv_ptr->image, cv_ptr);
    
    // Publish visualised image
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void ArucoTracker::processFrame(const cv::Mat& frame, cv_bridge::CvImagePtr& cv_ptr)
{
  // Detect markers
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(frame, dictionary_, marker_corners, marker_ids, parameters_);
  
  // Get frame centre
  cv::Point2f centre(frame.cols / 2.0f, frame.rows / 2.0f);
  
  // Draw centre crosshair
  cv::line(cv_ptr->image, cv::Point(centre.x - 10, centre.y), cv::Point(centre.x + 10, centre.y), cv::Scalar(0, 0, 255), 2);
  cv::line(cv_ptr->image, cv::Point(centre.x, centre.y - 10), cv::Point(centre.x, centre.y + 10), cv::Scalar(0, 0, 255), 2);
  
  // If markers are detected
  if (marker_ids.size() > 0) {
    // Draw detected markers
    cv::aruco::drawDetectedMarkers(cv_ptr->image, marker_corners, marker_ids);
    
    // Get the first marker
    std::vector<cv::Point2f> corners = marker_corners[0];
    
    // Calculate velocity
    geometry_msgs::Twist velocity = calculateVelocity(corners, centre);
    
    // Calculate rotation
    double rotation = calculateRotation(corners);
    
    // Draw visualisation
    drawVisualisation(cv_ptr, corners, centre, velocity, rotation);
    
    // Publish velocity command
    velocity_pub_.publish(velocity);
  }
}

geometry_msgs::Twist ArucoTracker::calculateVelocity(const std::vector<cv::Point2f>& corners, const cv::Point2f& centre)
{
  // Calculate marker centre
  cv::Point2f marker_centre(0, 0);
  for (const auto& corner : corners) {
    marker_centre += corner;
  }
  marker_centre *= 0.25f;
  
  // Calculate vector from centre to marker
  cv::Point2f direction = marker_centre - centre;
  
  // Create velocity message
  geometry_msgs::Twist vel;
  
  // Scale for better control (these can be adjusted)
  float scale_x = 0.001f;
  float scale_y = 0.001f;
  
  // Calculate velocity components
  vel.linear.x = -direction.y * scale_y; // Forward/backward (depends on camera orientation)
  vel.linear.y = -direction.x * scale_x; // Left/right
  vel.linear.z = desired_z_; // Set to desired distance
  
  return vel;
}

double ArucoTracker::calculateRotation(const std::vector<cv::Point2f>& corners)
{
  // Calculate marker orientation
  // Using the vector from marker corner 0 to corner 1
  cv::Point2f vec = corners[1] - corners[0];
  
  // Calculate angle in degrees
  double angle = atan2(vec.y, vec.x) * 180.0 / CV_PI;
  
  return angle;
}

void ArucoTracker::drawVisualisation(cv_bridge::CvImagePtr& cv_ptr, const std::vector<cv::Point2f>& corners, 
                                     const cv::Point2f& centre, const geometry_msgs::Twist& velocity, double rotation)
{
  // Calculate marker centre
  cv::Point2f marker_centre(0, 0);
  for (const auto& corner : corners) {
    marker_centre += corner;
  }
  marker_centre *= 0.25f;
  
  // Draw line from centre to marker
  cv::line(cv_ptr->image, centre, marker_centre, cv::Scalar(0, 255, 0), 2);
  
  // Calculate pixel distance
  double pixel_distance = cv::norm(marker_centre - centre);
  
  // Display pixel distance (near centre crosshair)
  std::stringstream ss_distance;
  ss_distance << "Distance: " << std::fixed << std::setprecision(1) << pixel_distance << " px";
  cv::putText(cv_ptr->image, ss_distance.str(), cv::Point(centre.x + 15, centre.y + 15), 
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
  
  // Draw bounding box around marker
  cv::Rect boundingBox = cv::boundingRect(corners);
  cv::rectangle(cv_ptr->image, boundingBox, cv::Scalar(255, 0, 0), 2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_tracker_node");
  ros::NodeHandle nh;
  
  ArucoTracker tracker(nh);
  
  ros::spin();
  
  return 0;
}