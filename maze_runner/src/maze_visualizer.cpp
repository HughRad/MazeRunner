/**
 * @file maze_visualizer.cpp
 * @brief Implementation of the MazeVisualizer class for RViz visualization
 * 
 * This file contains the implementation of the MazeVisualizer class that creates
 * RViz markers for maze walls and solution paths to help with collision detection
 * and path planning visualization.
 * 
 * @author Corso
 * @date May 2025
 */

#include <maze_runner/maze_visualizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <ros/ros.h>

MazeVisualizer::MazeVisualizer(const std::string& frame_id) 
    : frame_id_(frame_id), 
      wall_height_(0.05), 
      wall_thickness_(0.005),
      cell_size_(0.008),
      maze_rotation_(0.0),
      marker_id_counter_(0) {
    
    // Initialize publishers
    wall_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/maze_visualization/walls", 10);
    path_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/maze_visualization/solution_path", 10);
    trajectory_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/maze_visualizaion/robot_trajectory", 10);
    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/maze_visualization/complete", 10);
    
    // Initialize maze origin
    maze_origin_.x = 0.0;
    maze_origin_.y = 0.0;
    maze_origin_.z = 0.0;
    
    ROS_INFO("MazeVisualizer initialized with frame_id: %s", frame_id_.c_str());
}

MazeVisualizer::~MazeVisualizer() {
    clearAllMarkers();
}

std_msgs::ColorRGBA MazeVisualizer::createColor(double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

geometry_msgs::Point MazeVisualizer::transformPoint(double local_x, double local_y, double local_z) {
    geometry_msgs::Point point;
    
    // Apply rotation first
    double rotation_rad = maze_rotation_ * M_PI / 180.0;
    double cos_rot = cos(rotation_rad);
    double sin_rot = sin(rotation_rad);
    
    double rotated_x = local_x * cos_rot - local_y * sin_rot;
    double rotated_y = local_x * sin_rot + local_y * cos_rot;
    
    // Then apply translation
    point.x = maze_origin_.x + rotated_x;
    point.y = maze_origin_.y + rotated_y;
    point.z = maze_origin_.z + local_z;
    
    return point;
}

visualization_msgs::Marker MazeVisualizer::createWallMarker(const geometry_msgs::Point& start, 
                                                          const geometry_msgs::Point& end,
                                                          const std_msgs::ColorRGBA& color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "maze_walls";
    marker.id = marker_id_counter_++;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Calculate position (midpoint between start and end)
    marker.pose.position.x = (start.x + end.x) / 2.0;
    marker.pose.position.y = (start.y + end.y) / 2.0;
    marker.pose.position.z = (start.z + end.z) / 2.0 + wall_height_ / 2.0;
    
    // Calculate orientation to align cylinder with the wall
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double wall_length = sqrt(dx*dx + dy*dy);
    
    if (wall_length > 0.001) { // Avoid division by zero
        double yaw = atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(M_PI/2.0, 0.0, yaw); // Rotate cylinder to be horizontal
        marker.pose.orientation = tf2::toMsg(q);
    } else {
        marker.pose.orientation.w = 1.0;
    }
    
    // Set scale
    marker.scale.x = wall_thickness_; // Cylinder radius
    marker.scale.y = wall_thickness_; // Cylinder radius  
    marker.scale.z = wall_length;     // Cylinder height (length of wall)
    
    // Set color
    marker.color = color;
    
    // Set lifetime
    marker.lifetime = ros::Duration(0); // Persistent
    
    return marker;
}

void MazeVisualizer::setMazeParameters(const geometry_msgs::Point& origin, 
                                     double rotation, 
                                     double cell_size) {
    maze_origin_ = origin;
    maze_rotation_ = rotation;
    cell_size_ = cell_size;
    
    ROS_INFO("Maze parameters set: origin=[%.3f, %.3f, %.3f], rotation=%.1f°, cell_size=%.4f", 
             origin.x, origin.y, origin.z, rotation, cell_size);
}

void MazeVisualizer::setVisualizationParameters(double wall_height, double wall_thickness) {
    wall_height_ = wall_height;
    wall_thickness_ = wall_thickness;
    
    ROS_INFO("Visualization parameters set: wall_height=%.4f, wall_thickness=%.4f", 
             wall_height, wall_thickness);
}

void MazeVisualizer::publishMazeWalls(const std::vector<std::string>& maze) {
    if (maze.empty()) {
        ROS_WARN("Empty maze provided for visualization");
        return;
    }
    
    visualization_msgs::MarkerArray wall_markers;
    std_msgs::ColorRGBA wall_color = createColor(0.8, 0.2, 0.2, 0.8); // Red walls
    
    int rows = maze.size();
    int cols = maze[0].size();
    
    ROS_INFO("Creating wall markers for maze: %dx%d", rows, cols);
    
    // Create walls based on maze structure
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (maze[i][j] == '#' || maze[i][j] == '1') { // Wall character
                // Create a cube marker for each wall cell
                visualization_msgs::Marker wall_marker;
                wall_marker.header.frame_id = frame_id_;
                wall_marker.header.stamp = ros::Time::now();
                wall_marker.ns = "maze_walls";
                wall_marker.id = marker_id_counter_++;
                wall_marker.type = visualization_msgs::Marker::CUBE;
                wall_marker.action = visualization_msgs::Marker::ADD;
                
                // Position the wall at the cell center
                double local_x = j * cell_size_;
                double local_y = i * cell_size_;
                geometry_msgs::Point wall_position = transformPoint(local_x, local_y, wall_height_/2.0);
                wall_marker.pose.position = wall_position;
                
                // Set orientation based on maze rotation
                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, maze_rotation_ * M_PI / 180.0);
                wall_marker.pose.orientation = tf2::toMsg(q);
                
                // Set scale
                wall_marker.scale.x = cell_size_;
                wall_marker.scale.y = cell_size_;
                wall_marker.scale.z = wall_height_;
                
                // Set color
                wall_marker.color = wall_color;
                
                // Set lifetime
                wall_marker.lifetime = ros::Duration(0); // Persistent
                
                wall_markers.markers.push_back(wall_marker);
            }
        }
    }
    
    ROS_INFO("Publishing %lu wall markers", wall_markers.markers.size());
    wall_markers_pub_.publish(wall_markers);
}

void MazeVisualizer::publishSolutionPath(const std::vector<geometry_msgs::Pose>& path, 
                                       const std_msgs::ColorRGBA& color) {
    if (path.empty()) {
        ROS_WARN("Empty solution path provided for visualization");
        return;
    }
    
    visualization_msgs::MarkerArray path_markers;
    std_msgs::ColorRGBA path_color = color;
    
    // Use default green color if no color specified
    if (path_color.r == 0.0 && path_color.g == 0.0 && path_color.b == 0.0 && path_color.a == 0.0) {
        path_color = createColor(0.2, 0.8, 0.2, 0.9); // Green path
    }
    
    // Create line strip marker for the path
    visualization_msgs::Marker path_line;
    path_line.header.frame_id = frame_id_;
    path_line.header.stamp = ros::Time::now();
    path_line.ns = "solution_path";
    path_line.id = marker_id_counter_++;
    path_line.type = visualization_msgs::Marker::LINE_STRIP;
    path_line.action = visualization_msgs::Marker::ADD;
    
    // Set line properties
    path_line.scale.x = 0.002; // Line width
    path_line.color = path_color;
    path_line.lifetime = ros::Duration(0);
    
    // Add all path points
    for (const auto& pose : path) {
        path_line.points.push_back(pose.position);
    }
    
    path_markers.markers.push_back(path_line);
    
    // Also create sphere markers at key points
    for (size_t i = 0; i < path.size(); i += std::max(1, (int)(path.size() / 20))) { // Show ~20 points max
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = frame_id_;
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "solution_points";
        point_marker.id = marker_id_counter_++;
        point_marker.type = visualization_msgs::Marker::SPHERE;
        point_marker.action = visualization_msgs::Marker::ADD;
        
        point_marker.pose = path[i];
        point_marker.scale.x = 0.003;
        point_marker.scale.y = 0.003;
        point_marker.scale.z = 0.003;
        point_marker.color = createColor(0.2, 0.2, 0.8, 1.0); // Blue points
        point_marker.lifetime = ros::Duration(0);
        
        path_markers.markers.push_back(point_marker);
    }
    
    ROS_INFO("Publishing solution path with %lu markers", path_markers.markers.size());
    path_markers_pub_.publish(path_markers);
}

void MazeVisualizer::publishRobotTrajectory(const std::vector<geometry_msgs::Pose>& trajectory,
                                          const std_msgs::ColorRGBA& color) {
    if (trajectory.empty()) {
        ROS_WARN("Empty robot trajectory provided for visualization");
        return;
    }
    
    visualization_msgs::MarkerArray trajectory_markers;
    std_msgs::ColorRGBA traj_color = color;
    
    // Use default blue color if no color specified
    if (traj_color.r == 0.0 && traj_color.g == 0.0 && traj_color.b == 0.0 && traj_color.a == 0.0) {
        traj_color = createColor(0.2, 0.2, 0.8, 0.7); // Blue trajectory
    }
    
    // Create line strip for trajectory
    visualization_msgs::Marker traj_line;
    traj_line.header.frame_id = frame_id_;
    traj_line.header.stamp = ros::Time::now();
    traj_line.ns = "robot_trajectory";
    traj_line.id = marker_id_counter_++;
    traj_line.type = visualization_msgs::Marker::LINE_STRIP;
    traj_line.action = visualization_msgs::Marker::ADD;
    
    traj_line.scale.x = 0.001; // Thinner line for trajectory
    traj_line.color = traj_color;
    traj_line.lifetime = ros::Duration(0);
    
    for (const auto& pose : trajectory) {
        traj_line.points.push_back(pose.position);
    }
    
    trajectory_markers.markers.push_back(traj_line);
    
    ROS_INFO("Publishing robot trajectory with %lu points", trajectory.size());
    trajectory_markers_pub_.publish(trajectory_markers);
}

void MazeVisualizer::publishCompleteVisualization(const std::vector<std::string>& maze,
                                                const std::vector<geometry_msgs::Pose>& solution_path,
                                                const std::vector<geometry_msgs::Pose>& robot_trajectory) {
    visualization_msgs::MarkerArray complete_markers;
    
    // Reset marker counter for consistent numbering
    int saved_counter = marker_id_counter_;
    marker_id_counter_ = 0;
    
    // Add maze walls
    if (!maze.empty()) {
        std_msgs::ColorRGBA wall_color = createColor(0.8, 0.2, 0.2, 0.8);
        
        int rows = maze.size();
        int cols = maze[0].size();
        
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (maze[i][j] == '#' || maze[i][j] == '1') {
                    visualization_msgs::Marker wall_marker;
                    wall_marker.header.frame_id = frame_id_;
                    wall_marker.header.stamp = ros::Time::now();
                    wall_marker.ns = "complete_walls";
                    wall_marker.id = marker_id_counter_++;
                    wall_marker.type = visualization_msgs::Marker::CUBE;
                    wall_marker.action = visualization_msgs::Marker::ADD;
                    
                    double local_x = j * cell_size_;
                    double local_y = i * cell_size_;
                    geometry_msgs::Point wall_position = transformPoint(local_x, local_y, wall_height_/2.0);
                    wall_marker.pose.position = wall_position;
                    
                    tf2::Quaternion q;
                    q.setRPY(0.0, 0.0, maze_rotation_ * M_PI / 180.0);
                    wall_marker.pose.orientation = tf2::toMsg(q);
                    
                    wall_marker.scale.x = cell_size_;
                    wall_marker.scale.y = cell_size_;
                    wall_marker.scale.z = wall_height_;
                    wall_marker.color = wall_color;
                    wall_marker.lifetime = ros::Duration(0);
                    
                    complete_markers.markers.push_back(wall_marker);
                }
            }
        }
    }
    
    // Add solution path
    if (!solution_path.empty()) {
        visualization_msgs::Marker path_line;
        path_line.header.frame_id = frame_id_;
        path_line.header.stamp = ros::Time::now();
        path_line.ns = "complete_solution";
        path_line.id = marker_id_counter_++;
        path_line.type = visualization_msgs::Marker::LINE_STRIP;
        path_line.action = visualization_msgs::Marker::ADD;
        
        path_line.scale.x = 0.002;
        path_line.color = createColor(0.2, 0.8, 0.2, 0.9);
        path_line.lifetime = ros::Duration(0);
        
        for (const auto& pose : solution_path) {
            path_line.points.push_back(pose.position);
        }
        
        complete_markers.markers.push_back(path_line);
    }
    
    // Add robot trajectory
    if (!robot_trajectory.empty()) {
        visualization_msgs::Marker traj_line;
        traj_line.header.frame_id = frame_id_;
        traj_line.header.stamp = ros::Time::now();
        traj_line.ns = "complete_trajectory";
        traj_line.id = marker_id_counter_++;
        traj_line.type = visualization_msgs::Marker::LINE_STRIP;
        traj_line.action = visualization_msgs::Marker::ADD;
        
        traj_line.scale.x = 0.001;
        traj_line.color = createColor(0.2, 0.2, 0.8, 0.7);
        traj_line.lifetime = ros::Duration(0);
        
        for (const auto& pose : robot_trajectory) {
            traj_line.points.push_back(pose.position);
        }
        
        complete_markers.markers.push_back(traj_line);
    }
    
    // Restore counter
    marker_id_counter_ = saved_counter + complete_markers.markers.size();
    
    ROS_INFO("Publishing complete visualization with %lu markers", complete_markers.markers.size());
    marker_array_pub_.publish(complete_markers);
}

// New method to visualize path as maze walls
void MazeVisualizer::publishPathAsMaze(const std::vector<geometry_msgs::Pose>& maze_path) {
    if (maze_path.empty()) {
        ROS_WARN("Empty maze path provided for visualization");
        return;
    }
    
    visualization_msgs::MarkerArray path_maze_markers;
    std_msgs::ColorRGBA path_color = createColor(0.2, 0.8, 0.2, 0.9); // Green for path
    std_msgs::ColorRGBA wall_color = createColor(0.8, 0.2, 0.2, 0.8); // Red for inferred walls
    
    // Create line strip for the main path
    visualization_msgs::Marker path_line;
    path_line.header.frame_id = frame_id_;
    path_line.header.stamp = ros::Time::now();
    path_line.ns = "maze_path";
    path_line.id = marker_id_counter_++;
    path_line.type = visualization_msgs::Marker::LINE_STRIP;
    path_line.action = visualization_msgs::Marker::ADD;
    
    path_line.scale.x = 0.003; // Thicker line for path
    path_line.color = path_color;
    path_line.lifetime = ros::Duration(0);
    
    for (const auto& pose : maze_path) {
        path_line.points.push_back(pose.position);
    }
    
    path_maze_markers.markers.push_back(path_line);
    
    // Create sphere markers at each path point to show the traversable area
    for (size_t i = 0; i < maze_path.size(); ++i) {
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = frame_id_;
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "maze_path_points";
        point_marker.id = marker_id_counter_++;
        point_marker.type = visualization_msgs::Marker::SPHERE;
        point_marker.action = visualization_msgs::Marker::ADD;
        
        point_marker.pose = maze_path[i];
        point_marker.scale.x = cell_size_ * 0.8; // Slightly smaller than cell size
        point_marker.scale.y = cell_size_ * 0.8;
        point_marker.scale.z = wall_height_ * 0.5;
        
        // Alternate colors or use gradient
        if (i == 0) {
            point_marker.color = createColor(0.2, 0.8, 0.2, 1.0); // Start - bright green
        } else if (i == maze_path.size() - 1) {
            point_marker.color = createColor(0.8, 0.2, 0.8, 1.0); // End - magenta
        } else {
            point_marker.color = createColor(0.4, 0.6, 0.4, 0.7); // Path - muted green
        }
        
        point_marker.lifetime = ros::Duration(0);
        path_maze_markers.markers.push_back(point_marker);
    }
    
    ROS_INFO("Publishing maze path visualization with %lu markers", path_maze_markers.markers.size());
    wall_markers_pub_.publish(path_maze_markers);
}

void MazeVisualizer::clearAllMarkers() {
    visualization_msgs::MarkerArray clear_markers;
    
    // Create a single delete-all marker
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = frame_id_;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    
    clear_markers.markers.push_back(delete_marker);
    
    // Publish to all topics
    wall_markers_pub_.publish(clear_markers);
    path_markers_pub_.publish(clear_markers);
    trajectory_markers_pub_.publish(clear_markers);
    marker_array_pub_.publish(clear_markers);
    
    ROS_INFO("Cleared all maze visualization markers");
    marker_id_counter_ = 0;
}

void MazeVisualizer::updateVisualization(const std::vector<std::string>& maze,
                                       const std::vector<geometry_msgs::Pose>& solution_path) {
    clearAllMarkers();
    ros::Duration(0.1).sleep(); // Brief pause to ensure clearing completes
    publishCompleteVisualization(maze, solution_path);
}

void MazeVisualizer::updatePathVisualization(const std::vector<geometry_msgs::Pose>& maze_path) {
    clearAllMarkers();
    ros::Duration(0.1).sleep(); // Brief pause to ensure clearing completes
    publishPathAsMaze(maze_path);
}

int MazeVisualizer::getMarkerCount() const {
    return marker_id_counter_;
}

void MazeVisualizer::resetMarkerCounter() {
    marker_id_counter_ = 0;
}

// Global visualizer pointer for callback access
MazeVisualizer* visualizer_ptr = nullptr;

// Callback function to handle incoming maze path messages
void mazePathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    if (!visualizer_ptr) {
        ROS_WARN("Visualizer not initialized yet");
        return;
    }
    
    if (msg->poses.empty()) {
        ROS_WARN("Received empty maze path");
        return;
    }
    
    ROS_INFO("Received maze path with %lu poses", msg->poses.size());
    
    // Convert nav_msgs::Path to std::vector<geometry_msgs::Pose>
    std::vector<geometry_msgs::Pose> maze_path;
    maze_path.reserve(msg->poses.size());
    
    for (const auto& pose_stamped : msg->poses) {
        maze_path.push_back(pose_stamped.pose);
    }
    
    // Clear previous visualization and update with new path
    visualizer_ptr->updatePathVisualization(maze_path);
    
    ROS_INFO("Updated maze visualization with new path");
}

// Main function with ROS subscriber
int main(int argc, char** argv)
{
    ros::init(argc, argv, "maze_visualizer_node");
    ros::NodeHandle nh;

    // Get parameters
    std::string frame_id = nh.param<std::string>("frame_id", "world");
    std::string maze_path_topic = nh.param<std::string>("maze_path_topic", "/maze_path");
    
    // Initialize visualizer
    MazeVisualizer visualizer(frame_id);
    visualizer_ptr = &visualizer;
    
    // Set up visualization parameters (can be made into ROS parameters)
    double wall_height = nh.param<double>("wall_height", 0.05);
    double wall_thickness = nh.param<double>("wall_thickness", 0.005);
    double cell_size = nh.param<double>("cell_size", 0.008);
    
    visualizer.setVisualizationParameters(wall_height, wall_thickness);
    
    geometry_msgs::Point origin;
    origin.x = nh.param<double>("maze_origin_x", 0.0);
    origin.y = nh.param<double>("maze_origin_y", 0.0);
    origin.z = nh.param<double>("maze_origin_z", 0.0);
    double rotation = nh.param<double>("maze_rotation", 0.0);
    
    visualizer.setMazeParameters(origin, rotation, cell_size);
    
    // Subscribe to maze path topic
    ros::Subscriber maze_path_sub = nh.subscribe(maze_path_topic, 1, mazePathCallback);
    
    ROS_INFO("Maze Visualizer Node started");
    ROS_INFO("Listening for maze paths on topic: %s", maze_path_topic.c_str());
    ROS_INFO("Publishing visualizations on:");
    ROS_INFO("  - /maze_visualization/walls");
    ROS_INFO("  - /maze_visualization/solution_path"); 
    ROS_INFO("  - /maze_visualization/robot_trajectory");
    ROS_INFO("  - /maze_visualization/complete");
    
    ros::spin();
    return 0;
}