/**
 * @file robot_control.cpp
 * @brief Implementation of the DrawingRobot class for controlling the robot to solve mazes
 * 
 * This file contains the implementation of the DrawingRobot class that integrates
 * the image processing and maze solving components to control a robot to draw
 * the solution path through a detected maze.
 * 
 * @author Corso
 * @date May 2025
 */

#include <maze_runner/robot_control.h>

    DrawingRobot::DrawingRobot(const std::string& planning_group)  //= "manipulator"
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
    end_effector_pub_rate_(10.0), // 10 Hz publishing rate
    initial_joint_positions({0.0, -1.386, 0.723, -0.915, -1.556, 0.0})
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

    maze_path_pub_ = nh_.advertise<nav_msgs::Path>("/maze_path", 1);

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

    void DrawingRobot::publishEndEffectorPose(const ros::TimerEvent& event) {
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

    void DrawingRobot::waypointCallback(const geometry_msgs::Point::ConstPtr& msg) {
        latest_point_ = *msg;
        point_received_ = true;
        last_velocity_command_time_ = ros::Time::now();
    }

    void DrawingRobot::rotationCallback(const std_msgs::Float64::ConstPtr& msg) {
        latest_rotation_ = msg->data;
        rotation_received_ = true;
    }

    void DrawingRobot::positionControlUpdate(const ros::TimerEvent& event) {
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

    void DrawingRobot::executeCartesianMotion(const geometry_msgs::Pose& target_pose) {
        try {       
            // Set start state to current
            move_group_.setStartStateToCurrentState();
            
            // Create a vector of waypoints with just the target
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(target_pose);

            // move_group_.setMaxVelocityScalingFactor(0.8);  for changing speed
            // move_group_.setMaxAccelerationScalingFactor(0.8);

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

    void DrawingRobot::stopRobot() {
        // Simply stop robot motion
        move_group_.stop();
    }

void DrawingRobot::publishMazePath(std::vector<geometry_msgs::Pose> maze_path) {
nav_msgs::Path path_msg;
path_msg.header.stamp = ros::Time::now();
path_msg.header.frame_id = "world"; // Use your frame_id

// Convert poses to PoseStamped
for (const auto& pose : maze_path) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = path_msg.header;
    pose_stamped.pose = pose;
    path_msg.poses.push_back(pose_stamped);
}
// Publish the path
maze_path_pub_.publish(path_msg);
ROS_INFO("Published maze path with %lu poses", maze_path.size());
}

std::vector<double> DrawingRobot::getInitialJointPosition(){
    return initial_joint_positions;
}

void DrawingRobot::mazeCornerCallback(const geometry_msgs::Point::ConstPtr& msg) {
    corner_waypoint_ = *msg;
    std::vector<double> joints = getInitialJointPosition();
    double base = joints[0]; 

    double offsetX;  
    double offsetY; 
    double x;
    double y;

    if (base == 0.0)
    {
        offsetX = -0.0075;  
        offsetY = -0.008; 
        x = 0.05;
        y = 0.032;
    } else if (base == 1.57)
    {
        offsetX = 0.008;  
        offsetY = -0.0075; 
        x = -0.032;
        y = 0.05;
    } else if (base == 3.14)
    {
        offsetX = 0.0075;  
        offsetY = 0.008; 
        x = -0.05;
        y = -0.032;
    } else {
        return;
    }
    
    // ..........For base link = 90 deg (forward Pos)..........
    // double offsetX = 0.008; 
    // double offsetY = -0.0075; 

    // corner_waypoint_.y += 0.05;
    // corner_waypoint_.x -= 0.032;

    // ..........For base link = 0 deg (right Pos)..........
    // double offsetX = -0.0075;  
    // double offsetY = -0.008; 

    // corner_waypoint_.x += 0.05;
    // corner_waypoint_.y += 0.032;

    // ..........For base link = 180 deg (left Pos)..........
    // double offsetX = 0.0075;  
    // double offsetY = 0.008; 

    // corner_waypoint_.x -= 0.05;
    // corner_waypoint_.y -= 0.032;

    // Calculate the radius of this orbit
    double radius = sqrt(offsetX*offsetX + offsetY*offsetY);
    
    // Calculate the base angle from these offset values
    double baseAngle = atan2(offsetY, offsetX);
    
    // Get rotation in radians for orbit calculation
    double rotationRad = latest_rotation_ * M_PI / 180.0;
    
    // Apply rotation to determine position in the orbit
    double orbitAngle = baseAngle - rotationRad;
    
    // Calculate the new offset position 
    double orbitalOffsetX = radius * cos(orbitAngle);
    double orbitalOffsetY = radius * sin(orbitAngle);
    
    // Apply the orbital offset
    corner_waypoint_.x += orbitalOffsetX;
    corner_waypoint_.y += orbitalOffsetY;
    
    // Apply the second fixed offset directly from above selection
    corner_waypoint_.x += x;
    corner_waypoint_.y += y;
    
    maze_corner_received_ = true;
}

    double DrawingRobot::getMazeRotation() {
        return latest_rotation_;
    }

    double DrawingRobot::getCurrentDepth() {
        return getCurrentPose().position.z;
    }

    void DrawingRobot::enableServoControl() {
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

    void DrawingRobot::disableServoControl() {
        point_control_active_ = false;
        position_control_timer_.stop();
        waypoint_sub_.shutdown();
        rotation_sub_.shutdown();
        corner_waypoint_sub_.shutdown();
        image_sub_.shutdown(); // Also shut down the image subscriber
        ROS_INFO("Velocity-based control disabled.");
    }

    void DrawingRobot::enableImageBasedControl() {
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

    void DrawingRobot::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        latest_joint_state_ = *msg;
        joint_state_received_ = true;
    }

    void DrawingRobot::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if (!image_received_) {
        GUI_LOG("Camera feed active for camera control.");
        image_received_ = true;
    }
    }

    void DrawingRobot::snapshotCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (!snapshot_received_) {
        GUI_LOG("Snapshot received from ArUco tracker! Processing maze.");
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

    bool DrawingRobot::waitForControllerSync(double timeout) {
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

    std::vector<double> DrawingRobot::getCurrentJointPositions() {
        return move_group_.getCurrentJointValues();
    }

    bool DrawingRobot::validateJointPositions(const std::vector<double>& target_joints, double tolerance) {
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

    bool DrawingRobot::initializeToPosition(const std::vector<double>& target_joint_positions) {
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

    geometry_msgs::Pose DrawingRobot::getCurrentPose() {
        return move_group_.getCurrentPose().pose;
    }

    geometry_msgs::Pose DrawingRobot::getHoverPose(const geometry_msgs::Pose& drawing_pose) {
        geometry_msgs::Pose hover_pose = drawing_pose;
        hover_pose.position.z += hover_offset_;
        return hover_pose;
    }
    
    geometry_msgs::Pose DrawingRobot::makePose(double x, double y, double z) {
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
    
    bool DrawingRobot::moveToPose(const geometry_msgs::Pose& target_pose) {
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

    bool DrawingRobot::executeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints) {
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

    bool DrawingRobot::recoverFromJointStateError(int retry_count) {
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
    
    void DrawingRobot::setEndEffectorPublishRate(double rate) {
        end_effector_pub_rate_ = rate;
        
        // Restart timer with new rate if it's active
        if (end_effector_pub_timer_.isValid()) {
            end_effector_pub_timer_.stop();
            end_effector_pub_timer_ = nh_.createTimer(ros::Duration(1.0/end_effector_pub_rate_), 
                                                &DrawingRobot::publishEndEffectorPose, this);
        }
        
        ROS_INFO("End effector publish rate updated to %.1f Hz", end_effector_pub_rate_);
    }
    
    void DrawingRobot::publishEndEffectorPoseOnce() {
        ros::TimerEvent event; // Empty event
        publishEndEffectorPose(event);
        ROS_INFO("End effector pose published manually");
    }

int main(int argc, char** argv)
{
ros::init(argc, argv, "maze_runner");
ros::AsyncSpinner spinner(2); // Use 2 threads for better responsiveness
spinner.start();

// Initialize the node handle
ros::NodeHandle nh;

// Advertise the log topic
ros::Publisher gui_log_pub = nh.advertise<std_msgs::String>("/maze_gui/log", 10);
ros::Duration(2).sleep();

// Initialize stop parameter
nh.setParam("/robot_stop", false);

// Function to publish log messages to the GUI
auto publish_gui_log = [&gui_log_pub](const std::string& message) {
    std_msgs::String msg;
    msg.data = message;
    gui_log_pub.publish(msg);
    ROS_INFO("%s", message.c_str());
};

// Now use GUI_LOG
ROS_INFO("Node initialized and ready.");

bool loop = true;  // Change to false if you want the main logic to run only once
bool count = false;

try
{
    DrawingRobot robot;
    ros::Duration(2.0).sleep(); // Ensure robot is connected

    std::vector<double> current_joints = robot.getCurrentJointPositions();
    std::string joints_str = "";
    for (double val : current_joints)
        joints_str += std::to_string(val) + " ";

    ROS_INFO("Robot starting joint positions: [%s]", joints_str.c_str());

    bool init_success = robot.initializeToPosition(robot.getInitialJointPosition());

    if (!init_success) {
        ROS_WARN("Initial movement failed. Attempting recovery...");
        robot.recoverFromJointStateError();
        init_success = robot.initializeToPosition(robot.getInitialJointPosition());
        if (!init_success) {
            ROS_ERROR("Failed to initialize robot after recovery attempt");
        }
    }
} catch (const std::exception& e) {
    ROS_ERROR("Exception in main: %s", e.what());
    publish_gui_log(std::string("Error: ") + e.what());
    
    // Try to reset to home position even after error
    try {
        DrawingRobot robot;
        ros::Duration(2.0).sleep();
        bool return_success = robot.initializeToPosition(robot.getInitialJointPosition());
        if (return_success) {
            publish_gui_log("Robot returned to home position after error");
        }
    } catch (...) {
        publish_gui_log("Failed to return robot to home position after error");
    }
}

while (ros::ok())
{
    // Wait for the GUI to set /robot_ready to true
    publish_gui_log("Waiting for next start signal...");
    ros::Rate rate(2);  // 2 Hz check
    bool ready = false;

    while (ros::ok())
    {
        nh.getParam("/robot_ready", ready);
        if (!count)
        {
            publish_gui_log("Ready to Start Maze Runner");
            count=true;
        }
        if (ready) break;
        rate.sleep();
    }
    
    // Reset flag so it must be pressed again if loop is true
    nh.setParam("/robot_ready", false);

    publish_gui_log("Starting Maze Runner...");

    try {
        DrawingRobot robot;
        ImageProcessor processor;

        ros::Duration(1.0).sleep(); // Ensure robot is connected

        std::vector<double> current_joints = robot.getCurrentJointPositions();
        std::string joints_str = "";
        for (double val : current_joints)
            joints_str += std::to_string(val) + " ";

        ROS_INFO("Robot starting joint positions: [%s]", joints_str.c_str());

        bool init_success = robot.initializeToPosition(robot.getInitialJointPosition());

        if (!init_success) {
            ROS_WARN("Initial movement failed. Attempting recovery...");
            robot.recoverFromJointStateError();
            init_success = robot.initializeToPosition(robot.getInitialJointPosition());
            if (!init_success) {
                ROS_ERROR("Failed to initialize robot after recovery attempt");
                continue; // Skip this cycle and wait for next GUI trigger
            }
        }

        ros::Duration(2.0).sleep();

        ros::ServiceClient aruco_client = nh.serviceClient<std_srvs::Empty>("aruco_tracker/start");

        ROS_INFO("Waiting for aruco_tracker/start service...");
        if (!ros::service::waitForService("aruco_tracker/start", ros::Duration(10.0))) {
            ROS_ERROR("Service aruco_tracker/start not available after waiting");
            continue;
        }

        std_srvs::Empty srv;
        if(aruco_client.call(srv)) {
            ROS_INFO("Successfully called aruco_tracker/start service");
        } else {
            ROS_ERROR("Failed to call aruco_tracker/start service");
            continue;
        }

        robot.enableImageBasedControl();

        ros::Time start_time = ros::Time::now();
        const double timeout = 120.0;
        ROS_INFO("Waiting for snapshot from ArUco tracker...");

        while (!robot.image_processed_ && (ros::Time::now() - start_time).toSec() < timeout) {
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        if (!robot.image_processed_) {
            ROS_ERROR("Snapshot not received in time. Aborting this run.");
            continue;
        }

        ROS_INFO("Snapshot received successfully.");

        std::vector<std::string> maze = processor.processMaze(robot.received_image_);
        cv::Mat display = processor.getLatestBinaryImage();

        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(
            std_msgs::Header(), "bgr8", display
        ).toImageMsg();
        
        ros::Publisher maze_image_pub_ = nh.advertise<sensor_msgs::Image>("/maze/visualisation", 1);
        maze_image_pub_.publish(img_msg);

        maze_solver solver(maze);
        geometry_msgs::Point maze_corner = robot.corner_waypoint_;
        double scale = 0.008;
        std::pair<double, double> world = {maze_corner.x, maze_corner.y};
        std::vector<double> joints = robot.getInitialJointPosition();
        double base_rotation= joints[0] * (180/3.14);
        double rotation = 90 - base_rotation + robot.getMazeRotation(); 
        double depth = 0.158;

        solver.scaleSet(scale);
        solver.worldSet(world);
        solver.rotationSet(rotation);
        solver.depthSet(depth);

        std::vector<geometry_msgs::Pose> maze_path = solver.pathPlaner();
        std::vector<double> inital_pos = robot.getInitialJointPosition();

        robot.publishMazePath(maze_path);
        
        // Function to check if stop was requested
        auto check_stop_requested = [&nh, &publish_gui_log, &robot, &inital_pos]() {
            bool stop_requested = false;
            nh.getParam("/robot_stop", stop_requested);
            
            if (stop_requested) {
                publish_gui_log("Stop requested! Returning to home position...");
                ROS_INFO("Stop requested, returning to home position");
                
                // Return to home position
                bool return_success = robot.initializeToPosition(robot.getInitialJointPosition());
                if (!return_success) {
                    ROS_WARN("Return to home failed. Attempting recovery...");
                    robot.recoverFromJointStateError();
                    return_success = robot.initializeToPosition(robot.getInitialJointPosition());
                }
                
                if (return_success) {
                    publish_gui_log("Robot returned to home position");
                } else {
                    publish_gui_log("Failed to return robot to home position");
                }
                
                // Reset the stop flag
                nh.setParam("/robot_stop", false);
                return true;
            }
            return false;
        };

        ROS_INFO("Executing Cartesian path...");
        ros::Duration(1.0).sleep();
        
        // Check for stop request before executing path
        if (check_stop_requested()) {
            continue;  // Skip to next iteration if stop was requested
        }
        
        bool path_success = robot.executeCartesianPath(maze_path);
        
        // Check for stop request after path execution
        if (check_stop_requested()) {
            continue;  // Skip to next iteration if stop was requested
        }

        ROS_INFO("Returning to initial joint configuration...");
        bool return_success = robot.initializeToPosition(robot.getInitialJointPosition());
        if (!return_success) {
            ROS_WARN("Return failed. Attempting recovery...");
            robot.recoverFromJointStateError();
            return_success = robot.initializeToPosition(robot.getInitialJointPosition());
        }
        
        if (return_success) {
            publish_gui_log("Maze execution completed successfully.");
        } else {
            publish_gui_log("Warning: Failed to return to home position after maze execution.");
        }
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        publish_gui_log(std::string("Error: ") + e.what());
        
        // Try to reset to home position even after error
        try {
            DrawingRobot robot;
            ros::Duration(2.0).sleep();
            bool return_success = robot.initializeToPosition(robot.getInitialJointPosition());
            if (return_success) {
                publish_gui_log("Robot returned to home position after error");
            }
        } catch (...) {
            publish_gui_log("Failed to return robot to home position after error");
        }
    }
    catch (...) {
        ROS_ERROR("Unknown exception in main");
        publish_gui_log("Error: Unknown exception occurred");
        
        // Try to reset to home position even after error
        try {
            DrawingRobot robot;
            ros::Duration(2.0).sleep();
            bool return_success = robot.initializeToPosition(robot.getInitialJointPosition());
            if (return_success) {
                publish_gui_log("Robot returned to home position after error");
            }
        } catch (...) {
            publish_gui_log("Failed to return robot to home position after error");
        }
    }

    if (!loop) {
        break;  // Exit after one loop
    }
}

ros::shutdown();
return 0;
}