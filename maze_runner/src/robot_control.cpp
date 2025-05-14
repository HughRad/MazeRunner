/**
 * @file robot_control.cpp
 * @brief Implementation of the DrawingRobot class for controlling the robot to solve mazes
 * 
 * This file contains the implementation of the DrawingRobot class that integrates
 * the image processing and maze solving components to control a robot to draw
 * the solution path through a detected maze.
 * 
 * @author Original author
 * @date May 2025
 */

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
#include <std_srvs/Empty.h>

/**
 * @class DrawingRobot
 * @brief Class for controlling the robot to solve and draw a maze
 * 
 * Provides functionality to control a robot to locate a maze using camera tracking,
 * process the maze image, solve the maze, and then draw the solution path using
 * precise robotic movements.
 */
class DrawingRobot {
private:
    /**
     * @brief ROS node handle
     */
    ros::NodeHandle nh_;
    
    /**
     * @brief MoveIt interface for robot motion planning and execution
     */
    moveit::planning_interface::MoveGroupInterface move_group_;
    
    /**
     * @brief Name of the planning group for MoveIt
     */
    std::string planning_group_;

    /**
     * @brief Offset for hovering above the drawing surface
     */
    double hover_offset_; 

    /**
     * @brief Joint state subscriber for monitoring robot state
     */
    ros::Subscriber joint_state_sub_;
    
    /**
     * @brief Latest joint state received from the robot
     */
    sensor_msgs::JointState latest_joint_state_;
    
    /**
     * @brief Flag indicating if a joint state has been received
     */
    bool joint_state_received_;

    /**
     * @brief Subscriber for waypoints from ArUco tracker
     */
    ros::Subscriber waypoint_sub_;
    
    /**
     * @brief Subscriber for rotation from ArUco tracker
     */
    ros::Subscriber rotation_sub_;
    
    /**
     * @brief Subscriber for corner waypoint from ArUco tracker
     */
    ros::Subscriber corner_waypoint_sub_;

    /**
     * @brief Subscriber for camera images
     */
    ros::Subscriber image_sub_;
    
    /**
     * @brief Flag indicating if an image has been received
     */
    bool image_received_;

    /**
     * @brief Subscriber for maze snapshots from ArUco tracker
     */
    ros::Subscriber snapshot_sub_;
    
    /**
     * @brief Flag indicating if a snapshot has been received
     */
    bool snapshot_received_;

    /**
     * @brief Maximum time to wait for synchronization in seconds
     */
    const double sync_timeout_ = 10.0;
    
    /**
     * @brief Tolerance for position checking in meters
     */
    const double position_tolerance_ = 0.01;

    /**
     * @brief Latest target point received from ArUco tracker
     */
    geometry_msgs::Point latest_point_;
    
    /**
     * @brief Latest rotation angle received from ArUco tracker
     */
    double latest_rotation_;
    
    /**
     * @brief Flag indicating if a point has been received
     */
    bool point_received_;
    
    /**
     * @brief Flag indicating if a rotation has been received
     */
    bool rotation_received_;
    
    /**
     * @brief Flag indicating if a maze corner has been received
     */
    bool maze_corner_received_;

    /**
     * @brief Scale factor for rotation commands
     */
    double rotation_scale_factor_;
    
    /**
     * @brief Flag indicating if point control is active
     */
    bool point_control_active_;
    
    /**
     * @brief Time of last velocity command
     */
    ros::Time last_velocity_command_time_;
    
    /**
     * @brief Timeout for velocity commands in seconds
     */
    double velocity_timeout_;

    /**
     * @brief Timer for position control updates
     */
    ros::Timer position_control_timer_;

    /**
     * @brief Minimum velocity threshold
     */
    double min_velocity_threshold_;
    
    /**
     * @brief Maximum step scale for velocity control
     */
    double max_step_scale_;
    
    /**
     * @brief Publisher for end effector pose
     */
    ros::Publisher end_effector_pub_;
    
    /**
     * @brief Timer for publishing end effector position
     */
    ros::Timer end_effector_pub_timer_;
    
    /**
     * @brief End effector publishing rate in Hz
     */
    double end_effector_pub_rate_;

    /**
     * @brief Publishes the current end effector pose
     * 
     * Callback for the end effector publishing timer that publishes the current
     * pose of the robot's end effector.
     * 
     * @param event Timer event
     */
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

    /**
     * @brief Callback for waypoint messages from ArUco tracker
     * 
     * @param msg Waypoint point message
     */
    void waypointCallback(const geometry_msgs::Point::ConstPtr& msg) {
        latest_point_ = *msg;
        point_received_ = true;
        last_velocity_command_time_ = ros::Time::now();
    }

    /**
     * @brief Callback for rotation messages from ArUco tracker
     * 
     * @param msg Rotation value message
     */
    void rotationCallback(const std_msgs::Float64::ConstPtr& msg) {
        latest_rotation_ = msg->data;
        rotation_received_ = true;
    }

    /**
     * @brief Updates position control based on received waypoints
     * 
     * Callback for the position control timer that moves the robot toward the
     * latest received waypoint.
     * 
     * @param event Timer event
     */
    void positionControlUpdate(const ros::TimerEvent& event) {
        if (!point_control_active_ || !point_received_) {
            return;
        }
        
        // Check if the waypoint command has timed out
        if ((ros::Time::now() - last_velocity_command_time_).toSec() > velocity_timeout_) {
            ROS_WARN_THROTTLE(2.0, "Servoing command timeout! Stopping robot.");
            stopRobot();
            return;
        }
        
        try {
            // Use a local copy of the latest point and rotation for thread safety
            geometry_msgs::Point current_point_cmd = latest_point_;
            double current_rotation_cmd = latest_rotation_;           

            // Get current state
            geometry_msgs::Pose target_pose = getCurrentPose();
        
            target_pose.position = current_point_cmd;  // Update only the position component
     
            ROS_INFO_THROTTLE(0.5, "Moving based on waypoint command: distance=%.3f, angle=%.1f deg, "
                                "waypoint=[%.2f, %.2f, %.2f]",
                            target_pose.position.x, target_pose.position.y, target_pose.position.z);
            
            // Execute the motion without planning (direct movement)
            executeCartesianMotion(target_pose);
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception in position control update: %s", e.what());
            stopRobot();
        }
        catch (...) {
            ROS_ERROR("Unknown exception in position control update");
            stopRobot();
        }
    }

    /**
     * @brief Executes immediate Cartesian motion to a target pose
     * 
     * Moves the robot directly to a target pose without full planning,
     * using small steps to ensure smooth motion.
     * 
     * @param target_pose Target pose for the end effector
     */
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

    /**
     * @brief Stops the robot's motion
     * 
     * Immediately stops all robot motion for safety purposes.
     */
    void stopRobot() {
        // Simply stop robot motion
        move_group_.stop();
    }

public:
    /**
     * @brief Flag indicating if an image has been processed
     */
    bool image_processed_;
    
    /**
     * @brief Received image for maze processing
     */
    cv::Mat received_image_;

    /**
     * @brief Waypoint representing the corner of the maze
     */
    geometry_msgs::Point corner_waypoint_;
  
    /**
     * @brief Constructor for the DrawingRobot class
     * 
     * @param planning_group MoveIt planning group name
     */
    DrawingRobot(const std::string& planning_group = "manipulator") 
        : move_group_(planning_group), 
          planning_group_(planning_group), 
          joint_state_received_(false),
          image_received_(false),
          snapshot_received_(false),
          image_processed_(false),
          point_received_(false),
          rotation_received_(false),
          maze_corner_received_(false),
          point_control_active_(false),
          rotation_scale_factor_(0.01),
          velocity_timeout_(0.5),
          latest_rotation_(0.0),
          min_velocity_threshold_(0.001),
          max_step_scale_(5.0),
          end_effector_pub_rate_(10.0) // 10 Hz publishing rate
          {
        
        // Set default values for drawing
        hover_offset_ = 0.01; // waypoint when hovering
 
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

   /**
    * @brief Callback for maze corner waypoint messages
    * 
    * Processes the corner waypoint for the maze, adding small offsets
    * to account for marker positioning.
    * 
    * @param msg Corner waypoint point message
    */
    void mazeCornerCallback(const geometry_msgs::Point::ConstPtr& msg) {
        corner_waypoint_ = *msg;
        corner_waypoint_.x += 0.04;
        corner_waypoint_.y += 0.024;
        maze_corner_received_ = true;
    }

    /**
     * @brief Gets the latest maze rotation value
     * 
     * @return double Rotation angle in degrees
     */
    double getMazeRotation() {
        return latest_rotation_;
    }

    /**
     * @brief Gets the current depth (Z-coordinate) of the end effector
     * 
     * @return double Z-coordinate in meters
     */
    double getCurrentDepth() {
        return getCurrentPose().position.z;
    }

     /**
      * @brief Enables servo control for camera-based positioning
      * 
      * Sets up subscribers and timers for controlling the robot based on
      * camera tracking of ArUco markers.
      */
    void enableServoControl() {
        try {
            // Reset flags
            point_received_ = false;
            rotation_received_ = false;
            maze_corner_received_ = false;
            
            // Subscribe to waypoint and rotation topics
            waypoint_sub_ = nh_.subscribe("aruco_tracker/waypoint", 1, &DrawingRobot::waypointCallback, this);
            rotation_sub_ = nh_.subscribe("aruco_tracker/rotation", 1, &DrawingRobot::rotationCallback, this);
            corner_waypoint_sub_ = nh_.subscribe("aruco_tracker/cornerwaypoint", 1, &DrawingRobot::mazeCornerCallback, this);

            velocity_timeout_ = 0.5; // Stop if no commands received for 0.5 seconds
            point_control_active_ = true;
            
            // Start the velocity control timer (10 Hz updates)
            position_control_timer_ = nh_.createTimer(ros::Duration(0.1), &DrawingRobot::positionControlUpdate, this);
            
            ROS_INFO("Velocity-based control enabled. Listening to aruco_tracker topics.");
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception in enableServoControl: %s", e.what());
            point_control_active_ = false;
        }
    }

    /**
     * @brief Disables camera-based servo control
     * 
     * Shuts down subscribers and timers for camera-based control.
     */
    void disableServoControl() {
        point_control_active_ = false;
        position_control_timer_.stop();
        waypoint_sub_.shutdown();
        rotation_sub_.shutdown();
        corner_waypoint_sub_.shutdown();
        image_sub_.shutdown(); // Also shut down the image subscriber
        ROS_INFO("Velocity-based control disabled.");
    }

    /**
     * @brief Enables image-based control with snapshot subscription
     * 
     * Sets up servo control and subscribes to camera feeds and snapshot topic
     * for maze image processing.
     */
    void enableImageBasedControl() {
        // Enable servo control first
        enableServoControl();
        
        // Reset the image received flag
        image_received_ = false;
        
        // Subscribe to the image topic for camera control
        image_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &DrawingRobot::imageCallback, this);
        
        // Also subscribe to the snapshot topic for maze processing
        snapshot_received_ = false;
        snapshot_sub_ = nh_.subscribe("aruco_tracker/snapshot", 1, &DrawingRobot::snapshotCallback, this);
        
        ROS_INFO("Servo control enabled. Will automatically switch to maze processing when snapshot is received.");
    }   

    /**
     * @brief Callback for joint state messages
     * 
     * @param msg Joint state message from the robot
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        latest_joint_state_ = *msg;
        joint_state_received_ = true;
    }

    /**
     * @brief Callback for camera image messages
     * 
     * @param msg Image message from camera
     */
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (!image_received_) {
            ROS_INFO("Camera feed active for camera control.");
            image_received_ = true;
        }
    }

    /**
     * @brief Callback for snapshot messages from ArUco tracker
     * 
     * Processes the snapshot image for maze detection and solving.
     * 
     * @param msg Snapshot image message
     */
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
                
                disableServoControl();
                
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                image_processed_ = false;
            }
            
            // Allow time for the robot to stabilize
            ros::Duration(2.0).sleep();
        }
    }

    /**
     * @brief Waits for controller to synchronize
     * 
     * Waits until the robot controller stabilizes by monitoring joint positions.
     * 
     * @param timeout Maximum time to wait in seconds
     * @return bool True if controller synchronized successfully
     */
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

    /**
     * @brief Gets the current joint positions of the robot
     * 
     * @return std::vector<double> Vector of joint positions in radians
     */
    std::vector<double> getCurrentJointPositions() {
        return move_group_.getCurrentJointValues();
    }

    /**
     * @brief Validates that joint positions are within expected tolerance
     * 
     * @param target_joints Target joint positions
     * @param tolerance Tolerance in radians
     * @return bool True if current joints are within tolerance of target
     */
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

    /**
     * @brief Initializes robot to a specific joint configuration
     * 
     * @param target_joint_positions Target joint positions in radians
     * @return bool True if initialization succeeded
     */
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

    /**
     * @brief Gets the current pose of the robot's end effector
     * 
     * @return geometry_msgs::Pose Current end effector pose
     */
    geometry_msgs::Pose getCurrentPose() {
        return move_group_.getCurrentPose().pose;
    }

    /**
     * @brief Creates a hover pose above a drawing pose
     * 
     * @param drawing_pose Drawing pose on the surface
     * @return geometry_msgs::Pose Hover pose above the drawing pose
     */
    geometry_msgs::Pose getHoverPose(const geometry_msgs::Pose& drawing_pose) {
        geometry_msgs::Pose hover_pose = drawing_pose;
        hover_pose.position.z += hover_offset_;
        return hover_pose;
    }
    
    /**
     * @brief Creates a pose from x, y, z coordinates with fixed orientation
     * 
     * @param x X-coordinate in meters
     * @param y Y-coordinate in meters
     * @param z Z-coordinate in meters
     * @return geometry_msgs::Pose Pose with specified position and default orientation
     */
    geometry_msgs::Pose makePose(double x, double y, double z) {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        
        // Set orientation - end effector pointing downward with its x-axis aligned with the robot's +x direction
        // This quaternion represents a 90-degree rotation around the Y axis
        // This makes the Z-axis of the end effector point down, while keeping its X-axis aligned with the robot's +X
        pose.orientation.x = 1.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
        
        return pose;
    }
    
    /**
     * @brief Moves the robot to a specific pose
     * 
     * Plans and executes a motion to move the robot's end effector to a target pose.
     * 
     * @param target_pose Target pose for the end effector
     * @return bool True if motion succeeded
     */
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

    /**
     * @brief Executes a Cartesian path through a series of waypoints
     * 
     * Plans and executes a continuous path through a series of waypoints,
     * suitable for drawing the maze solution.
     * 
     * @param waypoints Vector of waypoints to follow
     * @return bool True if path execution succeeded
     */
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

    /**
     * @brief Attempts to recover from joint state errors
     * 
     * Executes small movements to reset controllers and recover from errors.
     * 
     * @param retry_count Number of recovery attempts
     * @return bool True if recovery succeeded
     */
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
    
    /**
     * @brief Sets the end effector publishing rate
     * 
     * @param rate Publishing rate in Hz
     */
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
    
    /**
     * @brief Publishes the current end effector pose once
     * 
     * Manually triggers the end effector pose publication once.
     */
    void publishEndEffectorPoseOnce() {
        ros::TimerEvent event; // Empty event
        publishEndEffectorPose(event);
        ROS_INFO("End effector pose published manually");
    }
};

/**
 * @brief Main function for the drawing robot node
 * 
 * Initializes the ROS node, sets up the drawing robot, and executes the
 * maze solving and drawing process.
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return int Exit code
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur3_cartesian_waypoints");
    ros::AsyncSpinner spinner(2); // Use 2 threads for better responsiveness
    spinner.start();
    
    ROS_INFO("Starting UR3 Cartesian waypoint following program");
    
    try {
        // Initialize the robot interface
        DrawingRobot robot;
        
        // Initialize image processor
        ImageProcessor processor; 
        ImageProcessor::DebugInfo debugInfo;

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

        // Create a service client for ArUco tracker
        ros::NodeHandle nh;

        ros::ServiceClient aruco_client = nh.serviceClient<std_srvs::Empty>("aruco_tracker/start");
        
        // Wait for the service to be available
        ROS_INFO("Waiting for aruco_tracker/start service...");
        if (!ros::service::waitForService("aruco_tracker/start", ros::Duration(10.0))) {
            ROS_ERROR("Service aruco_tracker/start not available after waiting");
            return 1;
        }
        
        // Call the service
        std_srvs::Empty srv;
        ROS_INFO("Calling aruco_tracker/start service...");
        if(aruco_client.call(srv)) {
            ROS_INFO("successfully called aruco tracker service");
        } else{
            ROS_ERROR("Failed to call service :(");
            return 1;
        }
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
        std::vector<std::string> maze = processor.processMaze(robot.received_image_, &debugInfo);
       
        // Create a maze_solver object with the maze string as an input
        maze_solver solver(maze);
        geometry_msgs::Point maze_corner = robot.corner_waypoint_;
        //set waypoint parameters
        double scale = 0.008; //set as distance between maze grid points (manually measured) 0.0084 was too big, changed to this but untested
        std::pair<double, double>  world = {maze_corner.x, maze_corner.y}; //set as the world coords of the mazes left top most point (should be given by nick)
        double rotation = 90 + robot.getMazeRotation(); // Rotation of maze in degrees - clockwise rotation (should be given by nick)
        double depth = 0.158;
        // ~14.8 cm end effector

        solver.scaleSet(scale);
        solver.worldSet(world);
        solver.rotationSet(rotation);
        solver.depthSet(depth);

        // Get the maze solution path
        // This returns path points in image/maze coordinates
        std::vector<geometry_msgs::Pose> maze_path = solver.pathPlaner();

        ROS_INFO("Moving to execute Cartesian path through defined waypoints");
        ros::Duration(1.0).sleep();
            
        ROS_INFO("Executing Cartesian path through defined waypoints");
        
        // Execute the Cartesian path - using the robotWaypoints from maze solver
        bool path_success = robot.executeCartesianPath(maze_path);
        
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
