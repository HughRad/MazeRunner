#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <iostream>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <sensor_msgs/JointState.h>
#include "maze_solver.h"
#include "image_processing.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>

class FrameTransformer {
    private:
        // Transform from camera frame to end effector frame
        tf2::Transform camera_to_ee_;
        bool transform_initialized_;
        
        // Scale factor for converting image pixels to meters
        double scale_factor_;
    
    public:
        FrameTransformer() : 
            transform_initialized_(false),
            scale_factor_(0.001) // Default 1mm per pixel
        {
            // Initialize with identity transform
            camera_to_ee_.setIdentity();
        }
        
        // Initialize the camera to end effector transform based on calibration data
        void initializeCameraToEETransform(const tf2::Vector3& translation, const tf2::Quaternion& rotation) {
            camera_to_ee_.setOrigin(translation);
            camera_to_ee_.setRotation(rotation);
            transform_initialized_ = true;
            
            ROS_INFO("Camera to end effector transform initialized: Translation [%.3f, %.3f, %.3f], Rotation [%.3f, %.3f, %.3f, %.3f]",
                translation.x(), translation.y(), translation.z(),
                rotation.x(), rotation.y(), rotation.z(), rotation.w());
        }
        
        // Initialize from pose - useful for setting up from ROS messages
        void initializeCameraToEETransform(const geometry_msgs::Pose& pose) {
            tf2::Vector3 translation(pose.position.x, pose.position.y, pose.position.z);
            tf2::Quaternion rotation(pose.orientation.x, pose.orientation.y, 
                                    pose.orientation.z, pose.orientation.w);
            initializeCameraToEETransform(translation, rotation);
        }
        
        // Set scale factor (meters per pixel)
        void setScaleFactor(double scale) {
            scale_factor_ = scale;
            ROS_INFO("Scale factor set to %.5f meters per pixel", scale_factor_);
        }
        
        // Transform a point from camera frame to end effector frame
        tf2::Vector3 transformCameraToEE(const tf2::Vector3& point_in_camera) {
            if (!transform_initialized_) {
                ROS_WARN("Camera to end effector transform not initialized, using identity!");
            }
            
            return camera_to_ee_ * point_in_camera;
        }
        
        // Transform a point from camera frame to robot base frame using current end effector pose
        tf2::Vector3 transformCameraToBase(const tf2::Vector3& point_in_camera, const geometry_msgs::Pose& ee_pose) {
            // First transform from camera to end effector
            tf2::Vector3 point_in_ee = transformCameraToEE(point_in_camera);
            
            // Create transform from end effector to base (from current end effector pose)
            tf2::Transform ee_to_base;
            tf2::Vector3 ee_translation(ee_pose.position.x, ee_pose.position.y, ee_pose.position.z);
            tf2::Quaternion ee_rotation(ee_pose.orientation.x, ee_pose.orientation.y, 
                                      ee_pose.orientation.z, ee_pose.orientation.w);
            ee_to_base.setOrigin(ee_translation);
            ee_to_base.setRotation(ee_rotation);
            
            // Apply end effector to base transform
            return ee_to_base * point_in_ee;
        }
        
        // Transform an image point to robot base frame 
        // x, y are pixel coordinates, z is depth from camera
        geometry_msgs::Pose transformImagePointToRobotTarget(
            double x, double y, double z,               // Point in image/camera coordinates
            const geometry_msgs::Pose& current_ee_pose, // Current end effector pose
            double rotation_angle = 0.0)                // Optional rotation in degrees
        {
            // Scale the x and y coordinates - may need to flip depending on camera orientation
            tf2::Vector3 point_in_camera(
                x * scale_factor_,   // Scale X 
                y * scale_factor_,   // Scale Y
                z                   // Z is already in meters (depth)
            );
            
            // Transform point from camera to end effector
            tf2::Vector3 point_in_ee = transformCameraToEE(point_in_camera);
            
            // Create end effector to base transform
            tf2::Transform ee_to_base;
            tf2::fromMsg(current_ee_pose, ee_to_base);
            
            // Calculate the target point in base frame
            tf2::Vector3 point_in_base = ee_to_base * point_in_ee;
            
            // Create the target pose
            geometry_msgs::Pose target_pose;
            target_pose.position.x = point_in_base.x();
            target_pose.position.y = point_in_base.y();
            target_pose.position.z = point_in_base.z();
            
            // Set orientation - start with current end effector orientation
            target_pose.orientation = current_ee_pose.orientation;
            
            // Apply additional rotation if specified
            if (rotation_angle != 0.0) {
                // Convert rotation from degrees to radians
                double rotation_rad = rotation_angle * M_PI / 180.0;
                
                // Extract current orientation as quaternion
                tf2::Quaternion q_current;
                tf2::fromMsg(current_ee_pose.orientation, q_current);
                
                // Convert to RPY (Roll, Pitch, Yaw)
                double roll, pitch, yaw;
                tf2::Matrix3x3(q_current).getRPY(roll, pitch, yaw);
                
                // Apply additional rotation to yaw
                tf2::Quaternion q_target;
                q_target.setRPY(roll, pitch, yaw + rotation_rad);
                
                // Set the new orientation
                tf2::convert(q_target, target_pose.orientation);
            }
            
            return target_pose;
        }
        
        // Transform maze coordinates to robot waypoints
        std::vector<geometry_msgs::Pose> transformMazePathToRobotWaypoints(
            const std::vector<std::pair<double, double>>& maze_path,
            const geometry_msgs::Pose& current_ee_pose,
            const std::pair<double, double>& maze_origin,
            double rotation_angle,
            double drawing_depth)
        {
            std::vector<geometry_msgs::Pose> robot_waypoints;
            
            // Convert rotation from degrees to radians
            double rotation_rad = rotation_angle * M_PI / 180.0;
            
            // Create rotation matrix for the maze rotation
            Eigen::Matrix2d rot_matrix;
            rot_matrix << cos(rotation_rad), -sin(rotation_rad),
                          sin(rotation_rad), cos(rotation_rad);
            
            // Extract current end effector pose and orientation
            tf2::Transform ee_to_base;
            tf2::fromMsg(current_ee_pose, ee_to_base);
            
            // For each point in the maze path
            ROS_INFO("Transforming %lu maze path points to robot coordinates", maze_path.size());
            ROS_INFO("Using maze origin: (%.3f, %.3f)", maze_origin.first, maze_origin.second);
            ROS_INFO("Using scale factor: %.5f", scale_factor_);
            ROS_INFO("Using rotation angle: %.2f degrees", rotation_angle);
            ROS_INFO("Using drawing depth: %.3f meters", drawing_depth);
            
            for (size_t i = 0; i < maze_path.size(); i++) {
                const auto& point = maze_path[i];
                
                // Apply maze rotation to the point
                Eigen::Vector2d rotated_point = rot_matrix * Eigen::Vector2d(point.first, point.second);
                
                // Scale the point to convert from pixels to meters
                double x_scaled = rotated_point.x() * scale_factor_;
                double y_scaled = rotated_point.y() * scale_factor_;
                
                // Create the point in camera frame
                // Note: Maze coordinates are in the image plane (x,y), but the camera
                // coordinate system typically has Z pointing forward, X right, Y down
                tf2::Vector3 point_in_camera(
                    maze_origin.first + x_scaled,  // X in camera frame
                    maze_origin.second + y_scaled, // Y in camera frame
                    drawing_depth                  // Z in camera frame (depth)
                );
                
                // Transform from camera to end effector frame
                tf2::Vector3 point_in_ee = transformCameraToEE(point_in_camera);
                
                // Transform from end effector to robot base frame
                tf2::Vector3 point_in_base = ee_to_base * point_in_ee;
                
                // Create the waypoint pose
                geometry_msgs::Pose waypoint;
                waypoint.position.x = point_in_base.x();
                waypoint.position.y = point_in_base.y();
                waypoint.position.z = point_in_base.z();
                
                // Use a consistent orientation for drawing (pen pointing down)
                // This orientation should match your robot's drawing configuration
                waypoint.orientation = current_ee_pose.orientation;
                waypoint.position.z = 0.145;  // Force the z-value to be 0.145 meters
                ROS_INFO("Waypoint %zu: maze(%.1f,%.1f) -> robot(%.3f,%.3f,%.3f)",
                    i, point.first, point.second,
                    waypoint.position.x, waypoint.position.y, waypoint.position.z);
                robot_waypoints.push_back(waypoint);
            }
            
            return robot_waypoints;
        }
    };
// Direction Extractor class definition
class DirectionExtractor {
    private:
        // Current movement state
        geometry_msgs::Twist current_velocity_;
        double current_rotation_;
        double target_distance_;
        double target_angle_;
        
        // Movement direction vector in 3D space
        Eigen::Vector3d movement_direction_;
        
    public:
        // Constructor
        DirectionExtractor() : 
            current_rotation_(0.0),
            target_distance_(0.0),
            target_angle_(0.0) 
        {
            // Initialize vectors
            movement_direction_ = Eigen::Vector3d::Zero();
        }
        
        /**
         * Extract movement direction from velocity and rotation commands
         * @param velocity The velocity command from ArUco tracker
         * @param rotation The rotation command from ArUco tracker
         * @return The normalized direction vector
         */
        Eigen::Vector3d extractDirection(const geometry_msgs::Twist& velocity, double rotation) {
            // Store the current values
            current_velocity_ = velocity;
            current_rotation_ = rotation;
    
            // Calculate the magnitude of velocity in the XY plane
            double velocity_magnitude = std::sqrt(
                velocity.linear.x * velocity.linear.x + 
                velocity.linear.y * velocity.linear.y
            );
    
            // If velocity is very small, return zero vector (no significant movement)
            if (velocity_magnitude < 0.001) {
                movement_direction_ = Eigen::Vector3d::Zero();
                target_angle_ = 0.0;
                target_distance_ = 0.0;
                return movement_direction_;
            }
    
            // Calculate the angle in the XY plane
            target_angle_ = std::atan2(velocity.linear.y, velocity.linear.x);
            
            // Convert to degrees for logging
            double angle_degrees = target_angle_ * 180.0 / M_PI;
            
            // Store target distance (magnitude of velocity)
            target_distance_ = velocity_magnitude;
    
            // Normalize direction vector
            if (velocity_magnitude > 0) {
                movement_direction_ = Eigen::Vector3d(
                    velocity.linear.x / velocity_magnitude,
                    velocity.linear.y / velocity_magnitude,
                    0.0  // We're focusing on XY plane here
                );
            }
    
            ROS_INFO("Extracted direction: [%.3f, %.3f, %.3f], angle: %.2f degrees, distance: %.3f",
                    movement_direction_.x(), movement_direction_.y(), movement_direction_.z(),
                    angle_degrees, target_distance_);
    
            return movement_direction_;
        }
    
        /**
         * Generate a waypoint given the current pose and movement parameters
         * @param current_pose The current pose of the robot end-effector
         * @param step_size The step size to move in the calculated direction
         * @param apply_rotation Whether to apply rotation correction
         * @return The target pose (waypoint)
         */
        geometry_msgs::Pose generateWaypoint(
            const geometry_msgs::Pose& current_pose, 
            double step_size = 0.01,
            bool apply_rotation = false) 
        {
            geometry_msgs::Pose target_pose = current_pose;
            
            // Extract current orientation as quaternion
            tf::Quaternion q_current(
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w
            );
            
            // Convert to RPY (Roll, Pitch, Yaw)
            double roll, pitch, yaw;
            tf::Matrix3x3(q_current).getRPY(roll, pitch, yaw);
            
            // Adjust position based on direction vector and step size
            // Scale the step size by the velocity magnitude to move proportionally
            double actual_step = step_size;
            if (target_distance_ > 0) {
                // Scale the step size, but cap it to avoid large movements
                actual_step = std::min(step_size * target_distance_ * 10.0, step_size * 5.0);
            }
            
            // Apply movement in the XY plane
            target_pose.position.x += movement_direction_.x() * actual_step;
            target_pose.position.y += movement_direction_.y() * actual_step;
            
            // Apply Z movement directly from velocity (simplified)
            target_pose.position.z += current_velocity_.linear.z * step_size;

            // Set orientation as identity quaternion (no rotation)
            target_pose.orientation.x = 1.0; // For re-orienting goal trajectory about x-axis
            target_pose.orientation.y = 0.0;
            target_pose.orientation.z = 0.0;
            target_pose.orientation.w = 0.0;
            
            
            // Apply rotation if requested
            if (apply_rotation) {
                // Convert desired rotation from degrees to radians
                double target_rotation_rad = current_rotation_ * M_PI / 180.0;
                
                // Create target orientation - for this example, we only adjust yaw
                // Keep the same roll and pitch
                tf::Quaternion q_target;
                q_target.setRPY(roll, pitch, target_rotation_rad);
                
                // Convert back to geometry_msgs::Quaternion
                target_pose.orientation.x = q_target.x();
                target_pose.orientation.y = q_target.y();
                target_pose.orientation.z = q_target.z();
                target_pose.orientation.w = q_target.w();
            }
            
            return target_pose;
        }
        
        // Getter methods
        double getTargetDistance() const { return target_distance_; }
        double getTargetAngle() const { return target_angle_; }
        Eigen::Vector3d getMovementDirection() const { return movement_direction_; }
    };

class DrawingRobot {
private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    std::string planning_group_;
    
    // Add the frame transformer
    FrameTransformer frame_transformer_;
    bool camera_calibrated_;

    // Offset for hovering above the drawing surface
    double hover_offset_; 

    // Joint state subscriber for monitoring
    ros::Subscriber joint_state_sub_;
    sensor_msgs::JointState latest_joint_state_;
    bool joint_state_received_;

    // Velocity control subscribers
    ros::Subscriber velocity_sub_;
    ros::Subscriber rotation_sub_;

    // Imaging subscriber
    ros::Subscriber image_sub_;
    bool image_received_;

    // Snapshot subscriber for maze processing
    ros::Subscriber snapshot_sub_;
    bool snapshot_received_;

    // Synchronization timeout parameters
    const double sync_timeout_ = 10.0; // Maximum time to wait for sync in seconds
    const double position_tolerance_ = 0.01; // Tolerance for position checking

    // Store the latest velocity and rotation commands
    geometry_msgs::Twist latest_velocity_;
    double latest_rotation_;
    bool velocity_received_;
    bool rotation_received_;

    // Velocity control parameters
    double velocity_scale_factor_;
    double rotation_scale_factor_;
    bool velocity_control_active_;
    ros::Time last_velocity_command_time_;
    double velocity_timeout_;

    // Timer for velocity control updates
    ros::Timer velocity_control_timer_;

    // Direction extractor for processing velocity commands
    DirectionExtractor direction_extractor_;
    Eigen::Vector3d current_direction_;

    // Direction extractor configuration parameters
    double min_velocity_threshold_;
    double max_step_scale_;
    
    // Add publisher for end effector pose
    ros::Publisher end_effector_pub_;
    
    // Timer for publishing end effector position
    ros::Timer end_effector_pub_timer_;
    
    // End effector publishing rate (Hz)
    double end_effector_pub_rate_;

    // End effector position publishing callback
    void publishEndEffectorPose(const ros::TimerEvent& event) {
        try {
            // Get current end effector pose
            geometry_msgs::Pose current_pose = getCurrentPose();
            
            // Create PoseStamped message
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "world"; // Use the correct frame ID
            pose_stamped.pose = current_pose;
            
            // Publish the pose
            end_effector_pub_.publish(pose_stamped);
            
            // Log occasionally for debugging
            ROS_DEBUG_THROTTLE(10.0, "Published end effector pose: [%.3f, %.3f, %.3f]",
                            current_pose.position.x, current_pose.position.y, current_pose.position.z);
        }
        catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(5.0, "Exception in publishEndEffectorPose: %s", e.what());
        }
    }

    void initializeCameraCalibration() {
        // These values need to be determined through camera calibration
        // They represent the transform from the camera to the end effector
        
        // Translation vector from camera to end effector (in meters)
        // Assuming camera is mounted near the end effector with offsets:
        tf2::Vector3 translation(0.05, 0.0, 0.03);  // 5cm in x, 3cm in z from EE
        
        // Rotation from camera to end effector
        // This example assumes the camera is looking downward
        tf2::Quaternion rotation;
        rotation.setRPY(0.0, 0.0, 0.0);  
        
        // Initialize the frame transformer with these values
        frame_transformer_.initializeCameraToEETransform(translation, rotation);
        
        // Set the scale factor for image to real-world conversion
        // This needs to be calibrated for your specific camera and working distance
        // A typical value might be 0.001 m/pixel (1mm per pixel) at a working distance of 30cm
        frame_transformer_.setScaleFactor(0.001);  // 1mm per pixel
        
        camera_calibrated_ = true;
        ROS_INFO("Camera calibration initialized with improved parameters");
    }
        
        
        // Enhanced direction extractor configuration
    void configureDirectionExtractor(double min_velocity_threshold = 0.001, 
            double max_step_scale = 5.0) {
    // Store configuration parameters in the class for reuse
    min_velocity_threshold_ = min_velocity_threshold;
    max_step_scale_ = max_step_scale;
    }

    // Velocity command callback
    void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    latest_velocity_ = *msg;
    velocity_received_ = true;
    last_velocity_command_time_ = ros::Time::now();
    }

    // Rotation command callback
    void rotationCallback(const std_msgs::Float64::ConstPtr& msg) {
    latest_rotation_ = msg->data;
    rotation_received_ = true;
    }
    // Velocity control update function (called by timer)
    void velocityControlUpdate(const ros::TimerEvent& event) {
        if (!velocity_control_active_ || !velocity_received_) {
            return;
        }
        
        // Check if the velocity command has timed out
        if ((ros::Time::now() - last_velocity_command_time_).toSec() > velocity_timeout_) {
            ROS_WARN_THROTTLE(2.0, "Velocity command timeout! Stopping robot.");
            stopRobot();
            return;
        }
        
        try {
            // Use a local copy of the latest velocity and rotation for thread safety
            geometry_msgs::Twist current_velocity_cmd = latest_velocity_;
            double current_rotation_cmd = latest_rotation_;
            
            // Extract movement direction using DirectionExtractor
            current_direction_ = direction_extractor_.extractDirection(current_velocity_cmd, current_rotation_cmd);
            
            // Get more detailed information from the direction extractor
            double target_distance = direction_extractor_.getTargetDistance();
            double target_angle = direction_extractor_.getTargetAngle();
            
            // Skip if movement is below threshold
            if (target_distance < min_velocity_threshold_) {
                ROS_DEBUG_THROTTLE(1.0, "Movement below threshold (%.4f < %.4f). Skipping update.", 
                                target_distance, min_velocity_threshold_);
                return;
            }
            
            // Get current state
            geometry_msgs::Pose current_pose = getCurrentPose();
            
            // Generate waypoint using the extracted direction
            geometry_msgs::Pose target_pose = direction_extractor_.generateWaypoint(
                current_pose, 
                velocity_scale_factor_,  // Scale the step size with our configured scale factor
                rotation_received_       // Only apply rotation if we've received rotation data
            );
            
            // Log information about the movement (avoiding non-ASCII degree symbol)
            ROS_INFO_THROTTLE(0.5, "Moving based on velocity command: distance=%.3f, angle=%.1f deg, "
                                "direction=[%.2f, %.2f, %.2f]",
                            target_distance, target_angle * 180.0 / M_PI,
                            current_direction_.x(), current_direction_.y(), current_direction_.z());
            
            // Execute the motion without planning (direct movement)
            executeCartesianMotion(target_pose);
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception in velocity control update: %s", e.what());
            stopRobot();
        }
        catch (...) {
            ROS_ERROR("Unknown exception in velocity control update");
            stopRobot();
        }
    }

    // Execute immediate Cartesian motion without full planning
    void executeCartesianMotion(const geometry_msgs::Pose& target_pose) {
        try {       
            // Set start state to current
            move_group_.setStartStateToCurrentState();
            
            // Create a vector of waypoints with just the target
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(target_pose);
            
            // Compute cartesian path with very small step size
            moveit_msgs::RobotTrajectory trajectory;
            double fraction = move_group_.computeCartesianPath(
                waypoints, 0.01, 0.0, trajectory);
            
            if (fraction > 0.95) {
                // Execute the trajectory directly
                move_group_.execute(trajectory);
            }
            else {
                ROS_WARN_THROTTLE(2.0, "Could not compute full Cartesian path (%.2f%%), skipping motion", 
                                fraction * 100.0);
            }
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception in executeCartesianMotion: %s", e.what());
        }
        catch (...) {
            ROS_ERROR("Unknown exception in executeCartesianMotion");
        }
    }

    // Stop the robot
    void stopRobot() {
        // Simply stop robot motion
        move_group_.stop();
    }

public:
    // Image processor object
    bool image_processed_;
    cv::Mat received_image_;

  
    // Constructor
    DrawingRobot(const std::string& planning_group = "manipulator") 
        : move_group_(planning_group), 
          planning_group_(planning_group), 
          camera_calibrated_(false),
          joint_state_received_(false),
          image_received_(false),
          snapshot_received_(false),
          image_processed_(false),
          velocity_received_(false),
          rotation_received_(false),
          velocity_control_active_(false),
          velocity_scale_factor_(1.0),
          rotation_scale_factor_(0.01),
          velocity_timeout_(0.5),
          latest_rotation_(0.0),
          min_velocity_threshold_(0.001),
          max_step_scale_(5.0),
          end_effector_pub_rate_(10.0), // 10 Hz publishing rate
          direction_extractor_() {
        
        // Initialize current_direction_
        current_direction_ = Eigen::Vector3d::Zero();
        
        // Configure move group settings
        configureMovement();
        
        // Configure direction extractor
        configureDirectionExtractor(0.001, 5.0);
        
        // Set default values for drawing
        hover_offset_ = 0.01; // waypoint when hovering
        
        // Initialize camera calibration
        initializeCameraCalibration();
        
        // Setup joint state subscriber for monitoring
        joint_state_sub_ = nh_.subscribe("/joint_states", 10, &DrawingRobot::jointStateCallback, this);
        
        // Initialize end effector publisher
        end_effector_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot/end_effector_pose", 10);
        
        // Start timer for end effector pose publishing
        end_effector_pub_timer_ = nh_.createTimer(ros::Duration(1.0/end_effector_pub_rate_), 
                                                &DrawingRobot::publishEndEffectorPose, this);
        
        ROS_INFO("End effector pose publisher initialized at %.1f Hz", end_effector_pub_rate_);
        
        // Wait for first joint state to ensure connection
        ROS_INFO("Waiting for first joint state message...");
        ros::Time start_time = ros::Time::now();
        while (!joint_state_received_ && (ros::Time::now() - start_time).toSec() < sync_timeout_) {
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        if (!joint_state_received_) {
            ROS_WARN("No joint state received within timeout. Robot synchronization might be poor.");
        } else {
            ROS_INFO("Joint state monitoring active.");
        }
    }

    // Method to manually calibrate the camera parameters
    void manualCalibrate(double tx, double ty, double tz, 
                        double roll, double pitch, double yaw,
                        double scale) {
        // Create translation vector
        tf2::Vector3 translation(tx, ty, tz);
        
        // Create rotation quaternion from RPY angles (in radians)
        tf2::Quaternion rotation;
        rotation.setRPY(roll, pitch, yaw);
        
        // Initialize the frame transformer with these values
        frame_transformer_.initializeCameraToEETransform(translation, rotation);
        
        // Set the scale factor
        frame_transformer_.setScaleFactor(scale);
        
        camera_calibrated_ = true;
        ROS_INFO("Manual camera calibration set: T(%.3f,%.3f,%.3f), R(%.3f,%.3f,%.3f), S=%.5f",
                tx, ty, tz, roll, pitch, yaw, scale);
    }

    // Method to test a single coordinate transformation
    geometry_msgs::Pose testCoordinateTransform(double maze_x, double maze_y, 
                                            double maze_origin_x, double maze_origin_y,
                                            double drawing_depth) {
        // Get current end effector pose
        geometry_msgs::Pose current_pose = getCurrentPose();
        
        // Create a test point
        std::pair<double, double> test_point(maze_x, maze_y);
        
        // Set maze origin
        std::pair<double, double> maze_origin(maze_origin_x, maze_origin_y);
        
        // Get current rotation from ArUco tracker or use 0
        double rotation = getMazeRotation();
        
        // Create a vector with just one point
        std::vector<std::pair<double, double>> test_path = {test_point};
        
        // Transform point to get robot coordinates
        std::vector<geometry_msgs::Pose> waypoints = 
            frame_transformer_.transformMazePathToRobotWaypoints(
                test_path, 
                current_pose,
                maze_origin,
                rotation,
                drawing_depth
            );
        
        if (waypoints.empty()) {
            ROS_ERROR("Transform test failed - no waypoints generated");
            return current_pose; // Return current pose as fallback
        }
        
        ROS_INFO("Transform test: maze(%.1f,%.1f) -> robot(%.3f,%.3f,%.3f)",
                maze_x, maze_y,
                waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z);
        
        return waypoints[0];
    }

    // Generate robot waypoints from maze solution using proper transformations
    std::vector<geometry_msgs::Pose> generateRobotWaypointsFromMaze(
        std::vector<std::pair<double, double>> maze_path)
    {
        // Set fixed drawing depth
        const double drawing_depth = 0.145;
        
        if (!camera_calibrated_) {
            ROS_ERROR("Camera calibration is not initialized! Cannot transform coordinates.");
            return std::vector<geometry_msgs::Pose>();
        }
        
        // Get current end effector pose
        geometry_msgs::Pose current_ee_pose = getCurrentPose();
        
        // Get maze origin in camera frame
        std::pair<double, double> maze_origin = getMazeWorldCoordinates();
        
        // Get maze rotation from ArUco tracker
        double rotation = getMazeRotation();
        
        // Transform maze path to robot waypoints using the frame transformer
        std::vector<geometry_msgs::Pose> robot_waypoints =
            frame_transformer_.transformMazePathToRobotWaypoints(
                maze_path,
                current_ee_pose,
                maze_origin,
                rotation,
                drawing_depth
            );
            
        ROS_INFO("Generated %lu robot waypoints from maze solution", robot_waypoints.size());
        
        // Add a safety hover point at the beginning
        if (!robot_waypoints.empty()) {
            geometry_msgs::Pose hover_pose = robot_waypoints.front();
            hover_pose.position.z += hover_offset_;
            robot_waypoints.insert(robot_waypoints.begin(), hover_pose);
            
            // Also add hover point at the end
            hover_pose = robot_waypoints.back();
            hover_pose.position.z += hover_offset_;
            robot_waypoints.push_back(hover_pose);
        }
        
        return robot_waypoints;
    }
    // Get the current movement direction vector
    Eigen::Vector3d getCurrentDirection() const {
        return current_direction_;
    }
    
    // Get the current target distance from the direction extractor
    double getCurrentTargetDistance() const {
        return direction_extractor_.getTargetDistance();
    }
    
    // Get the current target angle from the direction extractor
    double getCurrentTargetAngle() const {
        return direction_extractor_.getTargetAngle();
    }
    
    // Advanced method to adjust trajectory based on direction feedback
    bool adjustTrajectoryUsingDirection(const geometry_msgs::Pose& target_pose,
                                         double adjustment_factor = 0.5) {
        try {
            // Copy current_direction_ for thread safety
            Eigen::Vector3d dir = current_direction_;
            
            // Only adjust if we have a significant movement direction
            if (dir.norm() < min_velocity_threshold_) {
                ROS_INFO("No significant direction detected, skipping adjustment");
                return false;
            }
            
            // Create an adjusted pose based on the current direction
            geometry_msgs::Pose adjusted_pose = target_pose;
            
            // Apply directional adjustment
            adjusted_pose.position.x += dir.x() * adjustment_factor;
            adjusted_pose.position.y += dir.y() * adjustment_factor;
            
            // Execute the adjusted motion
            return moveToPose(adjusted_pose);
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception in adjustTrajectoryUsingDirection: %s", e.what());
            return false;
        }
    }

    // Set the scale factors for the direction extractor
    void setDirectionExtractorScaling(double step_scale) {
        max_step_scale_ = step_scale;
        ROS_INFO("Direction extractor step scale set to: %.2f", max_step_scale_);
    }
    
    // Set the minimum velocity threshold for the direction extractor
    void setDirectionExtractorThreshold(double threshold) {
        min_velocity_threshold_ = threshold;
        ROS_INFO("Direction extractor velocity threshold set to: %.5f", min_velocity_threshold_);
    }
    
    // Get a reference to the direction extractor for advanced usage
    DirectionExtractor& getDirectionExtractor() {
        return direction_extractor_;
    }

    std::pair<double, double> getMazeWorldCoordinates() {
        // Get current end effector pose after camera alignment
        geometry_msgs::Pose current_pose = getCurrentPose();
        
        // Convert the coordinate to the top-left corner of the maze
        // You may need to add offsets based on your camera position vs. end effector position
        double x = current_pose.position.x;
        double y = current_pose.position.y;
        
        return std::make_pair(x, y);
    }
    
    // Return the latest rotation value received from ArUco tracker
    double getMazeRotation() {
        return latest_rotation_;
    }

    // Return the current Z-position of the end effector
    double getCurrentDepth() {
        return getCurrentPose().position.z;
    }

     // Enables velocity based control 
    void enableVelocityControl() {
        try {
            // Initialize current_direction_ to zero vector
            current_direction_ = Eigen::Vector3d::Zero();
            
            // Reset flags
            velocity_received_ = false;
            rotation_received_ = false;
            
            // Subscribe to velocity and rotation topics
            velocity_sub_ = nh_.subscribe("aruco_tracker/velocity", 10, &DrawingRobot::velocityCallback, this);
            rotation_sub_ = nh_.subscribe("aruco_tracker/rotation", 10, &DrawingRobot::rotationCallback, this);
            
            // Initialize velocity control parameters
            velocity_scale_factor_ = 0.1;
            rotation_scale_factor_ = 0.01;
            velocity_timeout_ = 0.5; // Stop if no commands received for 0.5 seconds
            velocity_control_active_ = true;
            
            // Start the velocity control timer (10 Hz updates)
            velocity_control_timer_ = nh_.createTimer(ros::Duration(0.1), &DrawingRobot::velocityControlUpdate, this);
            
            ROS_INFO("Velocity-based control enabled. Listening to aruco_tracker topics.");
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception in enableVelocityControl: %s", e.what());
            velocity_control_active_ = false;
        }
    }

    // Disable velocity based control
    void disableVelocityControl() {
        velocity_control_active_ = false;
        velocity_control_timer_.stop();
        velocity_sub_.shutdown();
        rotation_sub_.shutdown();
        image_sub_.shutdown(); // Also shut down the image subscriber
        ROS_INFO("Velocity-based control disabled.");
    }

    // Sets the scaling factors for velocity commands
    void setVelocityScaleFactors(double linear_scale, double rotation_scale) {
        velocity_scale_factor_ = linear_scale;
        rotation_scale_factor_ = rotation_scale;
        ROS_INFO("Velocity scale factors set to: linear=%.2f, rotation=%.2f", 
                    velocity_scale_factor_, rotation_scale_factor_);
    }

    // Enables image based control with snapshot subscription for maze processing
    void enableImageBasedControl() {
        // Enable velocity control first
        enableVelocityControl();
        
        // Reset the image received flag
        image_received_ = false;
        
        // Subscribe to the image topic for velocity control
        image_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &DrawingRobot::imageCallback, this);
        
        // Also subscribe to the snapshot topic for maze processing
        snapshot_received_ = false;
        snapshot_sub_ = nh_.subscribe("aruco_tracker/snapshot", 1, &DrawingRobot::snapshotCallback, this);
        
        ROS_INFO("Velocity control enabled. Will automatically switch to maze processing when snapshot is received.");
    }   

    // Joint state callback
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        latest_joint_state_ = *msg;
        joint_state_received_ = true;
    }

    // Image callback function for velocity control
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (!image_received_) {
            ROS_INFO("Camera feed active for velocity control.");
            image_received_ = true;
        }
        
        // NOTE: We keep receiving camera images for velocity control
        // but don't use them for maze processing

        // Only use the camera image for normal operation
        // Don't process the maze based on these images
    }

    // Snapshot callback function for maze processing
    void snapshotCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (!snapshot_received_) {
            ROS_INFO("Snapshot received from ArUco tracker! Processing maze.");
            snapshot_received_ = true;
            
            // Unsubscribe from the snapshot topic
            snapshot_sub_.shutdown();
            
            // Convert the ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                received_image_ = cv_ptr->image; // Store the image in the class variable
                image_processed_ = true;
                ROS_INFO("Snapshot processed successfully. Ready for maze processing.");
                
                // Now that we have the snapshot for maze processing,
                // we can safely disable velocity control
                disableVelocityControl();
                
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                image_processed_ = false;
            }
            
            // Allow time for the robot to stabilize
            ros::Duration(2.0).sleep();
        }
    }

    // Configure movement parameters
    void configureMovement() {
        // Set max velocity and acceleration scaling - lower for drawing precision
        move_group_.setMaxVelocityScalingFactor(0.2);  // 20% of maximum velocity
        move_group_.setMaxAccelerationScalingFactor(0.2);  // 20% of maximum acceleration
        
        // Set planning time
        move_group_.setPlanningTime(5.0);  // Give the planner more time (5 seconds)
        
        // Set goal tolerances - slightly increased for better success rate
        move_group_.setGoalJointTolerance(0.05); 
        move_group_.setGoalPositionTolerance(0.01); 
        move_group_.setGoalOrientationTolerance(0.05);
        
        // Print reference frame and end effector info
        ROS_INFO("Reference frame: %s", move_group_.getPlanningFrame().c_str());
        ROS_INFO("End effector link: %s", move_group_.getEndEffectorLink().c_str());
    }


    // Improved sync function that waits for controller to stabilize
    bool waitForControllerSync(double timeout = 2.0) {
        ROS_INFO("Waiting for controller to synchronize...");
        
        ros::Time start_time = ros::Time::now();
        std::vector<double> prev_positions;
        bool is_stable = false;
        
        // Get initial state to compare
        std::vector<double> initial_positions = getCurrentJointPositions();
        prev_positions = initial_positions;
        
        while ((ros::Time::now() - start_time).toSec() < timeout) {
            // Sleep a bit before checking again
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            
            // Get current joint positions
            std::vector<double> current_positions = getCurrentJointPositions();
            
            // Check if current positions are close to previous positions (stabilized)
            bool all_stable = true;
            for (size_t i = 0; i < current_positions.size() && i < prev_positions.size(); ++i) {
                if (std::abs(current_positions[i] - prev_positions[i]) > 0.001) { // 0.001 rad movement threshold
                    all_stable = false;
                    break;
                }
            }
            
            if (all_stable) {
                // We've detected stability for one cycle, but let's confirm
                // by waiting a bit more and checking again
                ros::Duration(0.2).sleep();
                ros::spinOnce();
                current_positions = getCurrentJointPositions();
                
                all_stable = true;
                for (size_t i = 0; i < current_positions.size() && i < prev_positions.size(); ++i) {
                    if (std::abs(current_positions[i] - prev_positions[i]) > 0.001) {
                        all_stable = false;
                        break;
                    }
                }
                
                if (all_stable) {
                    is_stable = true;
                    break;
                }
            }
            
            prev_positions = current_positions;
        }
        
        if (is_stable) {
            ROS_INFO("Controller synchronized successfully.");
            return true;
        } else {
            ROS_WARN("Controller synchronization timed out. Proceeding with caution.");
            return false;
        }
    }

    // Get current joint positions
    std::vector<double> getCurrentJointPositions() {
        return move_group_.getCurrentJointValues();
    }

    // Validate that joint positions are within expected tolerance of target
    bool validateJointPositions(const std::vector<double>& target_joints, double tolerance = 0.05) {
        std::vector<double> current_joints = getCurrentJointPositions();
        
        if (current_joints.size() != target_joints.size()) {
            ROS_ERROR("Joint validation failed: different number of joints in target vs current");
            return false;
        }
        
        for (size_t i = 0; i < current_joints.size(); ++i) {
            if (std::abs(current_joints[i] - target_joints[i]) > tolerance) {
                ROS_WARN("Joint %zu position mismatch: target=%f, current=%f", 
                        i, target_joints[i], current_joints[i]);
                return false;
            }
        }
        
        return true;
    }

    // Initialize robot to a specific joint configuration with improved synchronization
    bool initializeToPosition(const std::vector<double>& target_joint_positions) {
        ROS_INFO("Initializing robot to specified joint configuration...");
        
        // Get current joint positions for logging
        std::vector<double> current_joints = getCurrentJointPositions();
        
        std::string current_joints_str = "";
        for (double val : current_joints) {
            current_joints_str += std::to_string(val) + " ";
        }
        ROS_INFO("Current joint positions: [%s]", current_joints_str.c_str());
        
        std::string target_joints_str = "";
        for (double val : target_joint_positions) {
            target_joints_str += std::to_string(val) + " ";
        }
        ROS_INFO("Target joint positions: [%s]", target_joints_str.c_str());
        
        // IMPORTANT: Explicitly sync the start state with the current robot state
        move_group_.setStartStateToCurrentState();
        
        // Wait for controller to stabilize
        waitForControllerSync();
        
        // Set joint value target
        move_group_.setJointValueTarget(target_joint_positions);
        
        // Plan and execute motion
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
        if (success) {
            ROS_INFO("Executing movement to initial configuration...");
            move_group_.execute(my_plan);
            
            // Wait for execution to complete and controller to stabilize
            waitForControllerSync(3.0);
            
            // Validate that we reached the target position
            if (validateJointPositions(target_joint_positions)) {
                ROS_INFO("Successfully moved to specified joint configuration.");
                return true;
            } else {
                ROS_WARN("Robot did not reach target configuration within tolerance.");
                return false;
            }
        } else {
            ROS_ERROR("Failed to plan movement to initial configuration.");
            return false;
        }
    }

    // Get current pose
    geometry_msgs::Pose getCurrentPose() {
        return move_group_.getCurrentPose().pose;
    }

    // Create a hover pose above a drawing pose
    geometry_msgs::Pose getHoverPose(const geometry_msgs::Pose& drawing_pose) {
        geometry_msgs::Pose hover_pose = drawing_pose;
        hover_pose.position.z += hover_offset_;
        return hover_pose;
    }
    
    // Create a pose from x,y,z coordinates with fixed orientation (pointing down with +x alignment)
    geometry_msgs::Pose makePose(double x, double y, double z) {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        
        // Set orientation - end effector pointing downward with its x-axis aligned with the robot's +x direction
        // This quaternion represents a 90-degree rotation around the Y axis
        // This makes the Z-axis of the end effector point down, while keeping its X-axis aligned with the robot's +X
        pose.orientation.x = 1.0;
        pose.orientation.y = 0.0;  // sin(π/4)
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;  // cos(π/4)
        
        return pose;
    }
    
    // Move to a specific pose with improved synchronization
    bool moveToPose(const geometry_msgs::Pose& target_pose) {
        // Print the target coordinates
        ROS_INFO("Moving to: [%f, %f, %f]", 
                 target_pose.position.x, 
                 target_pose.position.y, 
                 target_pose.position.z);
        
        // IMPORTANT: Explicitly sync the start state with the current robot state
        move_group_.setStartStateToCurrentState();
        
        // Wait for controller to stabilize
        waitForControllerSync();
        
        // Set the target pose
        move_group_.setPoseTarget(target_pose);
        
        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            // Record start time to measure execution duration
            ros::Time start_time = ros::Time::now();
            
            // Execute the motion
            move_group_.execute(plan);
            
            // Wait for execution to complete and controller to stabilize
            waitForControllerSync();
            
            ros::Time end_time = ros::Time::now();
            double duration = (end_time - start_time).toSec();
            
            ROS_INFO("Reached target pose (execution time: %.2f seconds)", duration);
            return true;
        } else {
            ROS_ERROR("Planning failed");
            return false;
        }
    }

    // Execute Cartesian path through waypoints with improved sync
    bool executeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints) {
    if (waypoints.empty()) {
        ROS_ERROR("No waypoints provided for path");
        return false;
    }
    // sleep a bit to allow for any last-minute adjustments
    ros::Duration(2.0).sleep();
    
    ROS_INFO("Starting Cartesian path execution with %lu waypoints", waypoints.size());
    
    // Get first waypoint
    geometry_msgs::Pose first_pose = waypoints[0];
    
    // Start by moving to hover position above first waypoint
    geometry_msgs::Pose hover_pose = getHoverPose(first_pose);
    
    if (!moveToPose(hover_pose)) {
        ROS_ERROR("Failed to move to hover position above starting point");
        return false;
    }
    
    // Wait for controller to stabilize
    waitForControllerSync();
    
    // Lower to starting point
    if (!moveToPose(first_pose)) {
        ROS_ERROR("Failed to lower to starting point");
        return false;
    }
    
    // Wait for controller to stabilize
    waitForControllerSync();
    
    // IMPORTANT: Explicitly sync the start state with the current robot state
    move_group_.setStartStateToCurrentState();
    
    // For continuous line drawing, we'll use all waypoints except the first one
    // since we're already at the first position
    std::vector<geometry_msgs::Pose> cartesian_waypoints;
    for (size_t i = 1; i < waypoints.size(); i++) {
        cartesian_waypoints.push_back(waypoints[i]);
        ROS_INFO("Added waypoint %lu: [%f, %f, %f]", 
                i, waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);
    }
    
    if (cartesian_waypoints.empty()) {
        ROS_ERROR("No valid waypoints to follow after the first one");
        return false;
    }
    
    // Execute the Cartesian path for continuous line drawing
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_.computeCartesianPath(
        cartesian_waypoints, 0.005, 0.0, trajectory);
    
    ROS_INFO("Computed Cartesian path (%.2f%% achieved)", fraction * 100.0);
    
    if (fraction >= 0.95) {
        // Record start time to measure execution duration
        ros::Time start_time = ros::Time::now();
        
        // Execute the motion
        move_group_.execute(trajectory);
        
        // Wait for execution to complete and controller to stabilize
        waitForControllerSync();
        
        ros::Time end_time = ros::Time::now();
        double duration = (end_time - start_time).toSec();
        
        ROS_INFO("Executed Cartesian path successfully (execution time: %.2f seconds)", duration);
        
        // Lift the pen after completing the entire path
        hover_pose = getHoverPose(move_group_.getCurrentPose().pose);
        moveToPose(hover_pose);
        
        return true;
    } else {
        ROS_WARN("Could only compute %.2f%% of the Cartesian path", fraction * 100.0);
        
        // Try to execute the partial path if it's reasonably complete
        if (fraction > 0.5) {
            ROS_INFO("Executing partial Cartesian path...");
            
            // Record start time to measure execution duration
            ros::Time start_time = ros::Time::now();
            
            // Execute the motion
            move_group_.execute(trajectory);
            
            // Wait for execution to complete and controller to stabilize
            waitForControllerSync();
            
            ros::Time end_time = ros::Time::now();
            double duration = (end_time - start_time).toSec();
            
            ROS_INFO("Executed partial Cartesian path (execution time: %.2f seconds)", duration);
            
            // Lift the pen after completion
            hover_pose = getHoverPose(move_group_.getCurrentPose().pose);
            moveToPose(hover_pose);
            
            return true;
        }
        
        return false;
    }
}
    // Add a robust recovery method to handle joint state errors
    bool recoverFromJointStateError(int retry_count = 3) {
        ROS_INFO("Attempting to recover from joint state error...");
        
        for (int i = 0; i < retry_count; i++) {
            ROS_INFO("Recovery attempt %d of %d", i+1, retry_count);
            
            // Get current joint positions
            std::vector<double> current_joints = getCurrentJointPositions();
            
            // IMPORTANT: Explicitly sync the start state with the current robot state
            move_group_.setStartStateToCurrentState();
            
            // Wait longer to ensure state sync
            waitForControllerSync(1.0);
            
            // Plan and execute a small "stay in place" move to reset controllers
            // Add a tiny offset to force a new trajectory
            std::vector<double> slightly_modified_joints = current_joints;
            for (size_t j = 0; j < slightly_modified_joints.size(); j++) {
                slightly_modified_joints[j] += 0.01; // Small offset to force movement
            }
            
            move_group_.setJointValueTarget(slightly_modified_joints);
            
            moveit::planning_interface::MoveGroupInterface::Plan recovery_plan;
            bool success = (move_group_.plan(recovery_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            
            if (success) {
                move_group_.execute(recovery_plan);
                
                // Wait for execution to complete and controller to stabilize
                bool sync_success = waitForControllerSync(2.0);
                
                if (sync_success) {
                    ROS_INFO("Recovery move succeeded");
                    return true;
                } else {
                    ROS_WARN("Recovery move executed but controller sync failed");
                }
            } else {
                ROS_ERROR("Recovery move planning failed");
            }
            
            // If we're here, the attempt failed - wait before next try
            ros::Duration(2.0).sleep();
        }
        
        ROS_ERROR("All recovery attempts failed");
        return false;
    }
    
    // Set end effector publishing rate
    void setEndEffectorPublishRate(double rate) {
        end_effector_pub_rate_ = rate;
        
        // Restart timer with new rate if it's active
        if (end_effector_pub_timer_.isValid()) {
            end_effector_pub_timer_.stop();
            end_effector_pub_timer_ = nh_.createTimer(ros::Duration(1.0/end_effector_pub_rate_), 
                                                   &DrawingRobot::publishEndEffectorPose, this);
        }
        
        ROS_INFO("End effector publish rate updated to %.1f Hz", end_effector_pub_rate_);
    }
    
    // Publish end effector pose once (for manual triggering)
    void publishEndEffectorPoseOnce() {
        ros::TimerEvent event; // Empty event
        publishEndEffectorPose(event);
        ROS_INFO("End effector pose published manually");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur3_cartesian_waypoints");
    ros::AsyncSpinner spinner(2); // Use 2 threads for better responsiveness
    spinner.start();
    
    ROS_INFO("Starting UR3 Cartesian waypoint following program");
    
    try {
        // Initialize the robot interface
        DrawingRobot robot;
        
        // Configure the direction extractor with more sensitive thresholds
        robot.setDirectionExtractorThreshold(0.0005);  // More sensitive
        robot.setDirectionExtractorScaling(1.0);       // Moderate scaling
        
        // Manual calibration override - update these values based on calibration results
        // These are example values and should be adjusted for your specific setup
        double tx = 0.0;   // Translation X (m)
        double ty = 0.0;    // Translation Y (m)
        double tz = 0.0;   // Translation Z (m)
        double roll = 0.0;  // Roll (rad)
        double pitch = 0.0; // Pitch
        double yaw = 0.0;   // Yaw (rad)
        double scale = 0.001; // Scale factor (m/pixel) - 1mm per pixel
        
        // Apply manual calibration
        robot.manualCalibrate(tx, ty, tz, roll, pitch, yaw, scale);

        // Initialize image processor
        ImageProcessor processor; 
        
        // Allow more time for ROS to fully initialize and connect to the robot
        ros::Duration(5.0).sleep();
        
        // Print current joint positions
        std::vector<double> current_joints = robot.getCurrentJointPositions();
        std::string joints_str = "";
        for (double val : current_joints) {
            joints_str += std::to_string(val) + " ";
        }
        ROS_INFO("Robot starting joint positions: [%s]", joints_str.c_str());
        
        // Define the desired initial joint configuration
        std::vector<double> initial_joint_positions = {0.0, -1.452, 0.623, -0.748, -1.571, 0.0};
        
        // Initialize robot with the specified joint configuration
        bool init_success = robot.initializeToPosition(initial_joint_positions);
        
        // If initialization fails, try recovery and retry
        if (!init_success) {
            ROS_WARN("Initial movement failed. Attempting recovery...");
            robot.recoverFromJointStateError();
            
            // Try again after recovery
            init_success = robot.initializeToPosition(initial_joint_positions);
            if (!init_success) {
                ROS_ERROR("Failed to initialize robot even after recovery attempt");
                return 1;
            }
        }
        
        ROS_INFO("Robot successfully initialized to specified joint configuration");
        
        // Allow more time for ROS to fully initialize and connect to the robot
        ros::Duration(5.0).sleep();
        
        // Enable image-based control (which also subscribes to snapshot topic)
        robot.enableImageBasedControl();

        // Wait for snapshot to be received and processed
        ros::Time start_time = ros::Time::now();
        const double timeout = 120.0; // 120 seconds timeout (longer to allow for proper alignment)
        ROS_INFO("Waiting for properly aligned snapshot from ArUco tracker...");
        while (!robot.image_processed_ && (ros::Time::now() - start_time).toSec() < timeout) {
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        if (!robot.image_processed_) {
            ROS_ERROR("No snapshot received within timeout period. Aborting maze processing.");
            return 1;
        } else {
            ROS_INFO("Snapshot received and processed successfully!");
        }

        // Process the maze using the received snapshot
        std::vector<std::string> maze = processor.processMaze(robot.received_image_, nullptr);
        maze_solver solver(maze);

        // DEBUGGING: Display the maze we're solving
        ROS_INFO("Maze to solve:");
        for (const auto& row : maze) {
            ROS_INFO("%s", row.c_str());
        }

        // Get the maze solution path
        // This returns path points in image/maze coordinates
        std::vector<geometry_msgs::Pose> maze_path = solver.pathPlaner();

        // DEBUGGING: Print the solution path
        ROS_INFO("Solution path in maze coordinates:");
        for (size_t i = 0; i < maze_path.size(); i++) {
            ROS_INFO("Point %zu: (%.1f, %.1f, %.1f)", i, 
                    maze_path[i].position.x, maze_path[i].position.y, maze_path[i].position.z);
        }

        // Adapter: extract 2D coordinates
        std::vector<std::pair<double, double>> maze_path_2d;
        for (const auto& pose : maze_path) {
            maze_path_2d.emplace_back(pose.position.x, pose.position.y);
        }
        
        // Get depth for drawing (offset from camera to drawing surface)
        double drawing_depth = robot.getCurrentDepth();
   
        // Generate robot waypoints using the proper coordinate transformation
        std::vector<geometry_msgs::Pose> robotWaypoints = 
            robot.generateRobotWaypointsFromMaze(maze_path_2d);
            
        // DEBUGGING: Print robot waypoints
        ROS_INFO("Generated %lu robot waypoints:", robotWaypoints.size());
        for (size_t i = 0; i < robotWaypoints.size(); i++) {
            ROS_INFO("Waypoint %zu: (%.3f, %.3f, %.3f)", i, 
                    robotWaypoints[i].position.x, 
                    robotWaypoints[i].position.y, 
                    robotWaypoints[i].position.z);
        }

        std::cout << "Press Enter to execute the path...";
        std::cin.get();
            
        ROS_INFO("Executing Cartesian path through defined waypoints");
        
        // Execute the Cartesian path - using the robotWaypoints from maze solver
        bool path_success = robot.executeCartesianPath(robotWaypoints);
        
        // Return to the initial joint configuration with improved error handling
        ROS_INFO("Returning to initial configuration");
        bool return_success = robot.initializeToPosition(initial_joint_positions);
        
        if (!return_success) {
            ROS_WARN("Failed to return to initial configuration. Attempting recovery...");
            robot.recoverFromJointStateError();
            robot.initializeToPosition(initial_joint_positions);
        }
        
        ROS_INFO("Program completed");
        ros::shutdown();
        return 0;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }
    catch (...) {
        ROS_ERROR("Unknown exception in main");
        return 1;
    }
}
