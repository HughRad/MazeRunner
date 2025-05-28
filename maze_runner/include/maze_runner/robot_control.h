/**
 * @file robot_control.h
 * @brief Header file for the DrawingRobot class for controlling the robot to solve mazes
 * 
 * This file contains the declaration of the DrawingRobot class that integrates
 * the image processing and maze solving components to control a robot to draw
 * the solution path through a detected maze.
 * 
 * @author Corso
 * @date May 2025
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <iostream>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include <std_srvs/Empty.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <maze_runner/maze_visualizer.h>
#include <maze_runner/maze_solver.h>
#include <maze_runner/image_processing.h>
#include <nav_msgs/Path.h>

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
     * @brief Initial joint position 
     */
    std::vector<double> initial_joint_positions;

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
     * @brief Publisher for logging messages to the GUI
     */
    ros::Publisher gui_log_pub;
   
    /**
     * @brief Macro for logging messages to the GUI
     * 
     * @param msg Message to log
     */
    #define GUI_LOG(msg)                                      \
   {                                                     \
       ROS_INFO_STREAM(msg);                             \
       std_msgs::String log_msg;                         \
       log_msg.data = msg;                               \
       gui_log_pub.publish(log_msg);                     \
   }
    
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
     * @brief Latest rotation angle received from ArUco tracker for maze
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
     * @brief Publisher for maze visualization in GUI
     */
    ros::Publisher gui_pub_maze_;
    
    /**
     * @brief Timer for publishing end effector position
     */
    ros::Timer end_effector_pub_timer_;
    
    /**
     * @brief End effector publishing rate in Hz
     */
    double end_effector_pub_rate_;
       
    /**
     * @brief Publisher for rviz maze path visualization
     */
    ros::Publisher maze_path_pub_;

    /**
     * @brief Publishes the current end effector pose
     * 
     * Callback for the end effector publishing timer that publishes the current
     * pose of the robot's end effector.
     * 
     * @param event Timer event
     */
    void publishEndEffectorPose(const ros::TimerEvent& event);

    /**
     * @brief Callback for waypoint messages from ArUco tracker
     * 
     * @param msg Waypoint point message
     */
    void waypointCallback(const geometry_msgs::Point::ConstPtr& msg);

    /**
     * @brief Callback for rotation messages from ArUco tracker
     * 
     * @param msg Rotation value message
     */
    void rotationCallback(const std_msgs::Float64::ConstPtr& msg);

    /**
     * @brief Updates position control based on received waypoints
     *
     * Callback for the position control timer that moves the robot toward the
     * latest received waypoint. The waypoint is transformed by the current pose
     * to determine the true target position.
     *
     * @param event Timer event
     */
    void positionControlUpdate(const ros::TimerEvent& event);

    /**
     * @brief Executes immediate Cartesian motion to a target pose
     * 
     * Moves the robot directly to a target pose without full planning,
     * using small steps to ensure smooth motion.
     * 
     * @param target_pose Target pose for the end effector
     */
    void executeCartesianMotion(const geometry_msgs::Pose& target_pose);

    /**
     * @brief Stops the robot's motion
     * 
     * Immediately stops all robot motion for safety purposes.
     */
    void stopRobot();

    /**
     * @brief Callback for joint state messages
     * 
     * @param msg Joint state message from the robot
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    /**
     * @brief Callback for camera image messages
     * 
     * @param msg Image message from camera
     */
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

    /**
     * @brief Callback for snapshot messages from ArUco tracker
     * 
     * Processes the snapshot image for maze detection and solving.
     * 
     * @param msg Snapshot image message
     */
    void snapshotCallback(const sensor_msgs::Image::ConstPtr& msg);

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
    DrawingRobot(const std::string& planning_group = "manipulator");
   
    /**
     * @brief Destructor for the DrawingRobot class
     */
    ~DrawingRobot() = default;

    /**
     * @brief Maze path publisher
     * 
     * Publishes the maze path as a nav_msgs::Path message for rviz visualization.
     * 
     * @param maze_path Vector of geometry_msgs::Pose representing the maze path
     */
    void publishMazePath(std::vector<geometry_msgs::Pose> maze_path);

    /**
     * @brief Getter for joint initialisation
     * 
     * @return std::vector<double> Initial joint positions
     */
    std::vector<double> getInitialJointPosition();

    /**
     * @brief Callback for maze corner waypoint messages
     * 
     * Processes the corner waypoint for the maze, adding small offsets
     * to account for marker positioning.
     * 
     * @param msg Corner waypoint point message
     */
    void mazeCornerCallback(const geometry_msgs::Point::ConstPtr& msg);

    /**
     * @brief Gets the latest maze rotation value
     * 
     * @return double Rotation angle in degrees
     */
    double getMazeRotation();

    /**
     * @brief Gets the current depth (Z-coordinate) of the end effector
     * 
     * @return double Z-coordinate in meters
     */
    double getCurrentDepth();

    /**
     * @brief Enables servo control for camera-based positioning
     * 
     * Sets up subscribers and timers for controlling the robot based on
     * camera tracking of ArUco markers.
     */
    void enableServoControl();

    /**
     * @brief Disables camera-based servo control
     * 
     * Shuts down subscribers and timers for camera-based control.
     */
    void disableServoControl();

    /**
     * @brief Enables image-based control with snapshot subscription
     * 
     * Sets up servo control and subscribes to camera feeds and snapshot topic
     * for maze image processing.
     */
    void enableImageBasedControl();

    /**
     * @brief Waits for controller to synchronize
     * 
     * Waits until the robot controller stabilizes by monitoring joint positions.
     * 
     * @param timeout Maximum time to wait in seconds
     * @return bool True if controller synchronized successfully
     */
    bool waitForControllerSync(double timeout = 2.0);

    /**
     * @brief Gets the current joint positions of the robot
     * 
     * @return std::vector<double> Vector of joint positions in radians
     */
    std::vector<double> getCurrentJointPositions();

    /**
     * @brief Validates that joint positions are within expected tolerance
     * 
     * @param target_joints Target joint positions
     * @param tolerance Tolerance in radians
     * @return bool True if current joints are within tolerance of target
     */
    bool validateJointPositions(const std::vector<double>& target_joints, double tolerance = 0.05);

    /**
     * @brief Initializes robot to a specific joint configuration
     * 
     * @param target_joint_positions Target joint positions in radians
     * @return bool True if initialization succeeded
     */
    bool initializeToPosition(const std::vector<double>& target_joint_positions);

    /**
     * @brief Gets the current pose of the robot's end effector
     * 
     * @return geometry_msgs::Pose Current end effector pose
     */
    geometry_msgs::Pose getCurrentPose();

    /**
     * @brief Creates a hover pose above a drawing pose
     * 
     * @param drawing_pose Drawing pose on the surface
     * @return geometry_msgs::Pose Hover pose above the drawing pose
     */
    geometry_msgs::Pose getHoverPose(const geometry_msgs::Pose& drawing_pose);
    
    /**
     * @brief Creates a pose from x, y, z coordinates with fixed orientation
     * 
     * @param x X-coordinate in meters
     * @param y Y-coordinate in meters
     * @param z Z-coordinate in meters
     * @return geometry_msgs::Pose Pose with specified position and default orientation
     */
    geometry_msgs::Pose makePose(double x, double y, double z);
    
    /**
     * @brief Moves the robot to a specific pose
     * 
     * Plans and executes a motion to move the robot's end effector to a target pose.
     * 
     * @param target_pose Target pose for the end effector
     * @return bool True if motion succeeded
     */
    bool moveToPose(const geometry_msgs::Pose& target_pose);

    /**
     * @brief Executes a Cartesian path through a series of waypoints
     * 
     * Plans and executes a continuous path through a series of waypoints,
     * suitable for drawing the maze solution.
     * 
     * @param waypoints Vector of waypoints to follow
     * @return bool True if path execution succeeded
     */
    bool executeCartesianPath(const std::vector<geometry_msgs::Pose>& waypoints);

    /**
     * @brief Attempts to recover from joint state errors
     * 
     * Executes small movements to reset controllers and recover from errors.
     * 
     * @param retry_count Number of recovery attempts
     * @return bool True if recovery succeeded
     */
    bool recoverFromJointStateError(int retry_count = 3);
    
    /**
     * @brief Sets the end effector publishing rate
     * 
     * @param rate Publishing rate in Hz
     */
    void setEndEffectorPublishRate(double rate);
    
    /**
     * @brief Publishes the current end effector pose once
     * 
     * Manually triggers the end effector pose publication once.
     */
    void publishEndEffectorPoseOnce();
};

#endif // ROBOT_CONTROL_H