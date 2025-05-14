/**
 * @file maze_solver.h
 * @brief Header file for the maze_solver class
 * 
 * This header defines the maze_solver class for solving mazes and generating
 * waypoints for robot navigation. It includes functions for path planning,
 * coordinate transformation, and visualization.
 * 
 * @author Original author
 * @date May 2025
 */

#ifndef MAZE_SOVLER_H
#define MAZE_SOVLER_H

#include <vector>
#include <string>
#include <queue>
#include <utility>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/Pose.h>

/**
 * @class maze_solver
 * @brief Class for solving mazes and generating waypoints for robot navigation
 * 
 * Provides functionality to solve mazes using breadth-first search and
 * generate waypoints for robot navigation, with options for scaling,
 * rotation, and coordinate transformation.
 */
class maze_solver
{
public:
    /**
     * @brief Constructor for the maze_solver class
     * 
     * @param mazeStr String representation of the maze where:
     *                S = Start
     *                E = End
     *                # = Wall
     *                . = Open path
     */
    maze_solver(const std::vector<std::string>& mazeStr);

    /**
     * @brief Plans a path through the maze and generates waypoints
     * 
     * Main function that solves the maze and converts the solution path
     * into a series of waypoints for robot navigation.
     * 
     * @return std::vector<geometry_msgs::Pose> Vector of waypoints for robot navigation
     */
    std::vector<geometry_msgs::Pose> pathPlaner();

    /**
     * @brief Sets the scale factor for waypoint transformation
     * 
     * @param scale Scale factor for coordinate transformation
     */
    void scaleSet(const double& scale);

    /**
     * @brief Sets the world origin coordinates for waypoint transformation
     * 
     * @param world Pair of (x,y) coordinates representing the world origin
     */
    void worldSet(std::pair<double, double>& world);

    /**
     * @brief Sets the rotation angle for waypoint transformation
     * 
     * @param rotation Rotation angle in degrees (clockwise)
     */
    void rotationSet(const double& rotation);

    /**
     * @brief Sets the depth (z-coordinate) for waypoints
     * 
     * @param depth Z-coordinate value for all waypoints
     */
    void depthSet(const double& depth);

private:
    /**
     * @brief Solves the maze using Breadth-First Search algorithm
     * 
     * @return std::vector<std::pair<int, int>> Vector of coordinates representing the path through the maze
     */
    std::vector<std::pair<int, int>> solve();

    /**
     * @brief Generates waypoints for robot navigation from the maze solution path
     * 
     * @param path Vector of grid coordinates representing the maze solution path
     * @return std::vector<geometry_msgs::Pose> Vector of waypoints for robot navigation
     */
    std::vector<geometry_msgs::Pose> generateWaypoints(const std::vector<std::pair<int, int>>& path) const;

    /**
     * @brief Prints the maze solution for debugging
     * 
     * @param path Vector of coordinates representing the solution path
     */
    void printSolution(const std::vector<std::pair<int, int>>& path) const;

    /**
     * @brief Prints the generated waypoints for debugging
     * 
     * @param waypoints Vector of waypoints for robot navigation
     */
    void printWaypoints(const std::vector<geometry_msgs::Pose>& waypoints) const;

    /**
     * @brief 2D grid representation of the maze
     */
    std::vector<std::vector<char>> maze;
    
    /**
     * @brief Dimensions of the maze
     */
    int rows;
    int cols;
    
    /**
     * @brief Start and end positions in the maze
     */
    std::pair<int, int> start;
    std::pair<int, int> end;
    
    /**
     * @brief Direction vectors for all 4 possible moves (up, right, down, left)
     */
    const int dx[4] = {-1, 0, 1, 0};
    const int dy[4] = {0, 1, 0, -1};

    /**
     * @brief Scale factor for transforming maze coordinates to world coordinates
     * 
     * Waypoints during BFS computations are based on a 'maze grid = 1' unit system.
     * For the robot this distance will need to be adjusted to whatever the real world 
     * scale of the maze is (i.e., 1 unit may = 0.72cm, hence we could set the scale to 0.72)
     */
    double scale_;
    
    /**
     * @brief Rotation angle in degrees (clockwise)
     */
    double rotation_;
    
    /**
     * @brief Z coordinate of maze surface
     */
    double depth_;
    
    /**
     * @brief World coordinate offset for the robot
     * 
     * This should be the real world coordinates of the left top point of the maze grid.
     */
    std::pair<double, double> world_;
    
    /**
     * @brief Checks if a position in the maze is valid for traversal
     * 
     * @param x Row index in the maze grid
     * @param y Column index in the maze grid
     * @return bool True if the position is valid, false otherwise
     */
    bool isValid(int x, int y) const;
};

#endif // MAZE_SOLVER_H
