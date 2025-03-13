#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>

class RobotControl
{
private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    std::string maze_file_;

public:
    RobotControl(ros::NodeHandle& nh) : 
        nh_(nh),
        move_group_("manipulator") // "manipulator" should match the planning group name
    {
        // Get parameters
        nh_.param<std::string>("maze_file", maze_file_, "default_maze.txt");
        
        // Print basic information
        ROS_INFO("Reference frame: %s", move_group_.getPlanningFrame().c_str());
        ROS_INFO("End effector link: %s", move_group_.getEndEffectorLink().c_str());
        ROS_INFO("Using maze file: %s", maze_file_.c_str());
    }

    void printCurrentPose()
    {
        // Get current pose and display it
        geometry_msgs::PoseStamped current_pose = move_group_.getCurrentPose();
        ROS_INFO("Current position: x=%f, y=%f, z=%f", 
                current_pose.pose.position.x, 
                current_pose.pose.position.y, 
                current_pose.pose.position.z);
    }

    void run()
    {
        printCurrentPose();
        
        // TODO: Implement maze reading and drawing logic
        ROS_INFO("Robot control is running. Maze runner ready.");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_control");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
  
    RobotControl robot_control(nh);
    robot_control.run();
  
    ros::waitForShutdown();
    return 0;
}