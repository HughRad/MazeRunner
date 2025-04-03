#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <iostream>
#include <limits>
#include "maze_solver.h"


class RobotController {
private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    std::string planning_group_;

public:
    // Constructor
    RobotController(const std::string& planning_group = "manipulator") 
        : move_group_(planning_group), planning_group_(planning_group) {
        
        // Configure move group settings
        configureMovement();
    }

    // Configure movement parameters
    void configureMovement() {
        // Set max velocity and acceleration scaling
        move_group_.setMaxVelocityScalingFactor(0.3);  // 30% of maximum velocity
        move_group_.setMaxAccelerationScalingFactor(0.3);  // 30% of maximum acceleration
        
        // Print reference frame and end effector info
        ROS_INFO("Reference frame: %s", move_group_.getPlanningFrame().c_str());
        ROS_INFO("End effector link: %s", move_group_.getEndEffectorLink().c_str());
    }

    // Move to a named target (like home position)
    bool moveToNamedTarget(const std::string& target_name) {
        ROS_INFO("Moving to %s position...", target_name.c_str());
        
        // Set the named target
        move_group_.setNamedTarget(target_name);
        
        // Plan the movement
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            // Execute the plan
            move_group_.execute(plan);
            ROS_INFO("Reached %s position", target_name.c_str());
            return true;
        } else {
            ROS_ERROR("Failed to plan to %s position", target_name.c_str());
            return false;
        }
    }

    // Move to a specific pose
    bool moveToPose(const geometry_msgs::Pose& target_pose) {
        // Print the target coordinates
        ROS_INFO("Target coordinates: [%f, %f, %f]", 
                 target_pose.position.x, 
                 target_pose.position.y, 
                 target_pose.position.z);
        
        // Set the target pose
        move_group_.setPoseTarget(target_pose);
        
        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            ROS_INFO("Planning succeeded");
            move_group_.execute(plan);
            ROS_INFO("Reached target pose");
            return true;
        } else {
            ROS_ERROR("Planning failed");
            return false;
        }
    }

    // Get current pose
    geometry_msgs::Pose getCurrentPose() {
        return move_group_.getCurrentPose().pose;
    }

    // Get user-defined pose via terminal input
    geometry_msgs::Pose getUserDefinedPose() {
        geometry_msgs::Pose target_pose;
        
        std::cout << "Enter target pose coordinates (x y z):\n";
        std::cout << "X coordinate (meters): ";
        std::cin >> target_pose.position.x;
        
        std::cout << "Y coordinate (meters): ";
        std::cin >> target_pose.position.y;
        
        std::cout << "Z coordinate (meters): ";
        std::cin >> target_pose.position.z;
        
        // Set default orientation 
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 1.0;
        
        return target_pose;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_motion_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    RobotController robot;
    
    
    if (!robot.moveToNamedTarget("home")) {
        ROS_ERROR("Could not move to home position");
        return 1;
    }
    
    // Main interaction loop
    char continue_input = 'y';
    while (ros::ok() && (continue_input == 'y' || continue_input == 'Y')) {
        geometry_msgs::Pose target_pose = robot.getUserDefinedPose();
        
        robot.moveToPose(target_pose);
        
        std::cout << "Do you want to move to another coordinate? (y/n): ";
        std::cin >> continue_input;
        
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    robot.moveToNamedTarget("up");
    
    robot.moveToNamedTarget("home");
    
    ros::shutdown();
    return 0;
}
