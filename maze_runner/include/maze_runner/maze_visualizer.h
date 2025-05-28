/**
 * @file maze_visualizer.h
 * @brief Header file for the MazeVisualizer class for RViz visualization
 * 
 * This file contains the declaration of the MazeVisualizer class that creates
 * RViz markers for maze walls and solution paths to help with collision detection
 * and path planning visualization.
 * 
 * @author Corso
 * @date May 2025
 */

#ifndef MAZE_VISUALIZER_H
#define MAZE_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <vector>
#include <string>

/**
 * @class MazeVisualizer
 * @brief Class for visualizing maze structures in RViz
 * 
 * Provides functionality to convert maze data into RViz markers for
 * visualization of walls, paths, and solution trajectories.
 */
class MazeVisualizer {
private:
    /**
     * @brief ROS node handle
     */
    ros::NodeHandle nh_;
    
    /**
     * @brief Publisher for maze wall markers
     */
    ros::Publisher wall_markers_pub_;
    
    /**
     * @brief Publisher for solution path markers
     */
    ros::Publisher path_markers_pub_;
    
    /**
     * @brief Publisher for robot trajectory markers
     */
    ros::Publisher trajectory_markers_pub_;
    
    /**
     * @brief Publisher for combined marker array
     */
    ros::Publisher marker_array_pub_;
    
    /**
     * @brief Frame ID for all markers
     */
    std::string frame_id_;
    
    /**
     * @brief Wall height for visualization
     */
    double wall_height_;
    
    /**
     * @brief Wall thickness for visualization
     */
    double wall_thickness_;
    
    /**
     * @brief Cell size for maze grid
     */
    double cell_size_;
    
    /**
     * @brief Maze origin point (corner)
     */
    geometry_msgs::Point maze_origin_;
    
    /**
     * @brief Maze rotation angle in degrees
     */
    double maze_rotation_;
    
    /**
     * @brief Current marker ID counter
     */
    int marker_id_counter_;

    /**
     * @brief Creates a color with RGBA values
     * 
     * @param r Red component (0.0-1.0)
     * @param g Green component (0.0-1.0)
     * @param b Blue component (0.0-1.0)
     * @param a Alpha component (0.0-1.0)
     * @return std_msgs::ColorRGBA Color message
     */
    std_msgs::ColorRGBA createColor(double r, double g, double b, double a = 1.0);
    
    /**
     * @brief Transforms a point based on maze origin and rotation
     * 
     * @param local_x Local X coordinate
     * @param local_y Local Y coordinate
     * @param local_z Local Z coordinate
     * @return geometry_msgs::Point Transformed point
     */
    geometry_msgs::Point transformPoint(double local_x, double local_y, double local_z = 0.0);
    
    /**
     * @brief Creates a wall marker between two points
     * 
     * @param start Start point of wall
     * @param end End point of wall
     * @param color Wall color
     * @return visualization_msgs::Marker Wall marker
     */
    visualization_msgs::Marker createWallMarker(const geometry_msgs::Point& start, 
                                              const geometry_msgs::Point& end,
                                              const std_msgs::ColorRGBA& color);

public:
    /**
     * @brief Constructor for MazeVisualizer
     * 
     * @param frame_id Frame ID for markers (default: "world")
     */
    MazeVisualizer(const std::string& frame_id = "world");
    
    /**
     * @brief Destructor
     */
    ~MazeVisualizer();
    
    /**
     * @brief Sets the maze parameters for visualization
     * 
     * @param origin Maze corner/origin point
     * @param rotation Maze rotation in degrees
     * @param cell_size Size of each cell in meters
     */
    void setMazeParameters(const geometry_msgs::Point& origin, 
                          double rotation, 
                          double cell_size);
    
    /**
     * @brief Sets visualization parameters
     * 
     * @param wall_height Height of wall markers
     * @param wall_thickness Thickness of wall markers
     */
    void setVisualizationParameters(double wall_height = 0.05, 
                                   double wall_thickness = 0.005);
    
    /**
     * @brief Publishes maze wall visualization
     * 
     * @param maze 2D string vector representing the maze
     */
    void publishMazeWalls(const std::vector<std::string>& maze);
    
    /**
     * @brief Publishes solution path visualization
     * 
     * @param path Vector of poses representing the solution path
     * @param color Path color (optional)
     */
    void publishSolutionPath(const std::vector<geometry_msgs::Pose>& path, 
                           const std_msgs::ColorRGBA& color = std_msgs::ColorRGBA());
    
    /**
     * @brief Publishes robot trajectory visualization
     * 
     * @param trajectory Vector of poses representing robot trajectory
     * @param color Trajectory color (optional)
     */
    void publishRobotTrajectory(const std::vector<geometry_msgs::Pose>& trajectory,
                               const std_msgs::ColorRGBA& color = std_msgs::ColorRGBA());
    
    /**
     * @brief Publishes complete maze visualization (walls + paths)
     * 
     * @param maze 2D string vector representing the maze
     * @param solution_path Solution path poses (optional)
     * @param robot_trajectory Robot trajectory poses (optional)
     */
    void publishCompleteVisualization(const std::vector<std::string>& maze,
                                    const std::vector<geometry_msgs::Pose>& solution_path = {},
                                    const std::vector<geometry_msgs::Pose>& robot_trajectory = {});
    
    /**
     * @brief Clears all published markers
     */
    void clearAllMarkers();
    
    /**
     * @brief Updates visualization with new data
     * 
     * @param maze Updated maze data
     * @param solution_path Updated solution path
     */
    void updateVisualization(const std::vector<std::string>& maze,
                           const std::vector<geometry_msgs::Pose>& solution_path = {});
    
    /**
     * @brief Gets the current marker count
     * 
     * @return int Number of markers published
     */
    int getMarkerCount() const;
    
    /**
     * @brief Resets the marker ID counter
     */
    void resetMarkerCounter();

     /**
     * @brief Visualize a path as maze representation
     * @param maze_path Vector of poses representing the maze path
     */
    void publishPathAsMaze(const std::vector<geometry_msgs::Pose>& maze_path);
    
    /**
     * @brief Update visualization with new path data
     * @param maze_path Vector of poses representing the maze path
     */
    void updatePathVisualization(const std::vector<geometry_msgs::Pose>& maze_path);
};

#endif // MAZE_VISUALIZER_H