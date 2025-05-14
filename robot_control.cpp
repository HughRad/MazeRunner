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

class DrawingRobot {
private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    std::string planning_group_;

    // Offset for hovering above the drawing surface
    double hover_offset_; 

    // Joint state subscriber for monitoring
    ros::Subscriber joint_state_sub_;
    sensor_msgs::JointState latest_joint_state_;
    bool joint_state_received_;

    // Servo control subscribers
    ros::Subscriber waypoint_sub_;
    ros::Subscriber rotation_sub_;
    ros::Subscriber corner_waypoint_sub_;

    // Imaging subscriber
    ros::Subscriber image_sub_;
    bool image_received_;

    // Snapshot subscriber for maze processing
    ros::Subscriber snapshot_sub_;
    bool snapshot_received_;

    // Synchronization timeout parameters
    const double sync_timeout_ = 10.0; // Maximum time to wait for sync in seconds
    const double position_tolerance_ = 0.01; // Tolerance for position checking

    // Store the latest point and rotation commands
    geometry_msgs::Point latest_point_;
    double latest_rotation_;
    bool point_received_;
    bool rotation_received_;
    bool maze_corner_received_;

    // Velocity control parameters
    // double velocity_scale_factor_;
    double rotation_scale_factor_;
    bool point_control_active_;
    ros::Time last_velocity_command_time_;
    double velocity_timeout_;

    // Timer for velocity control updates
    ros::Timer position_control_timer_;

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

    // waypoint command callback
    void waypointCallback(const geometry_msgs::Point::ConstPtr& msg) {
    latest_point_ = *msg;
    point_received_ = true;
    last_velocity_command_time_ = ros::Time::now();
    }

    // Rotation command callback
    void rotationCallback(const std_msgs::Float64::ConstPtr& msg) {
    latest_rotation_ = msg->data;
    rotation_received_ = true;
    }
    // position control update function (called by timer)
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
            // target_pose.position.z = 0.35;
     
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

    //Store the the waypoint representation of the maze corner
    geometry_msgs::Point corner_waypoint_;
  
    // Constructor
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

   // Get the current maze corner point
    void mazeCornerCallback(const geometry_msgs::Point::ConstPtr& msg) {
        corner_waypoint_ = *msg;
        corner_waypoint_.x += 0.04;
        corner_waypoint_.y += 0.024;
        maze_corner_received_ = true;
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
    void enableServoControl() {
        try {
 
            // Reset flags
            point_received_ = false;
            rotation_received_ = false;
            maze_corner_received_ = false;
            
            // Subscribe to wapoint and rotation topics
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

    // Disable camera based control
    void disableServoControl() {
        point_control_active_ = false;
        position_control_timer_.stop();
        waypoint_sub_.shutdown();
        rotation_sub_.shutdown();
        corner_waypoint_sub_.shutdown();
        image_sub_.shutdown(); // Also shut down the image subscriber
        ROS_INFO("Velocity-based control disabled.");
    }

    // Enables image based control with snapshot subscription for maze processing
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

    // Joint state callback
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        latest_joint_state_ = *msg;
        joint_state_received_ = true;
    }

    // Image callback function for camera control
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (!image_received_) {
            ROS_INFO("Camera feed active for camera control.");
            image_received_ = true;
        }
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
                
                disableServoControl();
                
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                image_processed_ = false;
            }
            
            // Allow time for the robot to stabilize
            ros::Duration(2.0).sleep();
        }
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
        double scale = 0.008; //set as distance between maze grid points (manualy measured) 0.0084 was too big, changed to this but untested
        std::pair<double, double>  world = {maze_corner.x, maze_corner.y}; //set as the world coords of the mazes left top most point (should be given by nick)
        double rotation = 90 + robot.getMazeRotation(); // Rotation of maze in degrees - clockwise rotation (should be given by nick)
        double depth = 0.158;
        // ~14.8 cm end effector

        solver.scaleSet(scale);
        solver.worldSet(world);
        solver.rotationSet(rotation);
        solver.depthSet(depth);
        // // DEBUGGING: Display the maze we're solving
        // ROS_INFO("Maze to solve:");
        // for (const auto& row : maze) {
        //     ROS_INFO("%s", row.c_str());
        // }

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
