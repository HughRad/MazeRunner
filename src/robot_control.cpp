void mazeCornerCallback(const geometry_msgs::Point::ConstPtr& msg) {
    corner_waypoint_ = *msg;
    
    // Define the radius of the orbit
    double offsetX = -0.009;  // 9mm toward base (negative X)
    double offsetY = -0.0085; // 8.5mm to the right (negative Y)
    
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
    
    // Apply the second fixed offset directly
    corner_waypoint_.x += 0.04;
    corner_waypoint_.y += 0.024;
    
    maze_corner_received_ = true;
}




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


class DrawingRobot {
private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    std::string planning_group_;
    double hover_offset_; // Offset for hovering above the drawing surface
    
    // Add a joint state subscriber for monitoring
    ros::Subscriber joint_state_sub_;
    sensor_msgs::JointState latest_joint_state_;
    bool joint_state_received_;
    
    // Add synchronization timeout parameters
    const double sync_timeout_ = 10.0; // Maximum time to wait for sync in seconds
    const double position_tolerance_ = 0.5; // Tolerance for position checking

public:
    // Constructor
    DrawingRobot(const std::string& planning_group = "manipulator") 
        : move_group_(planning_group), planning_group_(planning_group), joint_state_received_(false) {
        
        // Configure move group settings
        configureMovement();
        
        // Set default values for drawing
        hover_offset_ = 0.01; // waypoint when hovering
        
        // Setup joint state subscriber for monitoring
        joint_state_sub_ = nh_.subscribe("/joint_states", 10, &DrawingRobot::jointStateCallback, this);
        
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

    

    // Joint state callback
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        latest_joint_state_ = *msg;
        joint_state_received_ = true;
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
    
    // Create a pose from x,y,z coordinates
    geometry_msgs::Pose makePose(double x, double y, double z) {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        
        // Set orientation - pen pointing down (may need adjustment for your specific setup)
        pose.orientation.x = 0.0;
        pose.orientation.y = 1.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 0.0;
        
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur3_cartesian_waypoints");
    ros::AsyncSpinner spinner(2); // Use 2 threads for better responsiveness
    spinner.start();
    
    ROS_INFO("Starting UR3 Cartesian waypoint following program");
    
    // Initialize the robot interface
    DrawingRobot robot;
    
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
    std::vector<double> initial_joint_positions = {0.0, -1.569, 2.145, -2.143, -1.618, 0.0};
    
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
    
    // Need to add a new function here for integration with nicks code
    // Velocity based end effector control to image the maze
    
    //get the maze layout as a string (should be given by Ryan)
    std::vector<std::string> maze = {
        "####################",
        "#...#..............#",
        "###.#.############.#",
        "#...#.#..........#.#",
        "#.###.#.##########.#",
        "#.#...#.#........#.#",
        "#.#.###.#.######.#.#",
        "S...#...#.#....#.#.#",
        "###.#.###.#.##.#.#.#",
        "#...#.....#.#..#.#.#",
        "#.#######.#.#.##.#.#",
        "#.#.......#.#....#.#",
        "#.#.###.###.######.E",
        "#...#.....#......#.#",
        "#####.###.######.#.#",
        "#.....#...#....#.#.#",
        "#.#####.###.#.##.#.#",
        "#....#...#..#......#",
        "#.####.###.#######.#",
        "####################"
    };

    // Create a maze_solver object with the maze string as an input
    maze_solver solver(maze);

    //set waypoint parameters
    double scale = 0.01; //set as distance between maze grid points (manualy measured)
    std::pair<double, double>  world = {0.2, 0.2}; //set as the world coords of the mazes left top most point (should be given by nick)
    double rotation = 0; // Rotation of maze in degrees - clockwise rotation (should be given by nick)
    double depth = 0.145; // set as the drawing depth (should be given by nick)
    // ~14.5 cm end effector

    solver.scaleSet(scale);
    solver.worldSet(world);
    solver.rotationSet(rotation);
    solver.depthSet(depth);

    //solve the maze and output the required robot waypoints in order of execution
    std::vector<geometry_msgs::Pose> robotWaypoints = solver.pathPlaner();

    std::cout << "Press any key to continue...";
    std::cin.get();
        
    ROS_INFO("Executing Cartesian path through defined waypoints");
    
    // Execute the Cartesian path - using the robotWaypoints from your maze solver
    bool path_success = robot.executeCartesianPath(robotWaypoints);

    // If path execution fails, try recovery and retry with more robust error handling
    if (!path_success) {
        ROS_WARN("Cartesian path execution failed. Attempting recovery...");
        
        if (robot.recoverFromJointStateError(3)) {
            // Try again after recovery, but with a smaller subset of waypoints
            // to increase chances of success
            ROS_INFO("Retrying with simplified path");
            std::vector<geometry_msgs::Pose> simplified_waypoints;
            
            // Only keep every other waypoint to simplify the path
            for (size_t i = 0; i < robotWaypoints.size(); i += 2) {
                simplified_waypoints.push_back(robotWaypoints[i]);
            }
            
            // Make sure we still have the last point to close the path
            if (simplified_waypoints.back().position.x != robotWaypoints.back().position.x ||
                simplified_waypoints.back().position.y != robotWaypoints.back().position.y ||
                simplified_waypoints.back().position.z != robotWaypoints.back().position.z) {
                simplified_waypoints.push_back(robotWaypoints.back());
            }
            
            path_success = robot.executeCartesianPath(simplified_waypoints);
            
            if (!path_success) {
                ROS_ERROR("Failed to execute even simplified Cartesian path");
            }
        }
    }
    
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
