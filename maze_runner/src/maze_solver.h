#ifndef MAZE_SOVLER_H
#define MAZE_SOVLER_H

#include <vector>
#include <string>
#include <queue>
#include <utility>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/Pose.h>

class maze_solver
{
public:
    
    maze_solver(const std::vector<std::string>& mazeStr); // Constructor takes a string ascii representation of the maze
    // S = Start
    // E = End
    // # = Wall
    // . = Open path

    std::vector<geometry_msgs::Pose> pathPlaner();

    void scaleSet(const double& scale); //waypoint modifier setters

    void worldSet(std::pair<double, double>& world);

    void rotationSet(const double& rotation);

    void depthSet(const double& depth);

private:

    std::vector<std::pair<int, int>> solve(); // Solve the maze using Breadth-First Search and return the path as a vector of coordinates

    std::vector<geometry_msgs::Pose> generateWaypoints(const std::vector<std::pair<int, int>>& path) const; // Generate waypoints for robot navigation (only at corners)

    void printSolution(const std::vector<std::pair<int, int>>& path) const; // Print the maze with the solution path, for debugging

    void printWaypoints(const std::vector<geometry_msgs::Pose>& waypoints) const; // Print waypoints, for debugging

    std::vector<std::vector<char>> maze; // 2D grid representation of the maze
    
    int rows; // Dimensions of the maze
    int cols;
    
    std::pair<int, int> start; // Start and end positions
    std::pair<int, int> end;
    
    const int dx[4] = {-1, 0, 1, 0}; // Direction vectors for all 4 possible moves (up, right, down, left)
    const int dy[4] = {0, 1, 0, -1};

    // allows modification of the waypoint scale. Waypoints during BFS computations are based on a 'maze grid = 1' unit system. 
    // For the robot this distance will need to be ajusted to whatever the real world scale of the maze it. (ie, 1 unit may = 0.72cm, hence we
    // could set the scale to 0.72)
    double scale_; 
    double rotation_; // Rotation in degrees (clockwise rotation)
    double depth_; // x coordinate of maze surface
    std::pair<double, double>  world_; // world coord offset for the robot. This should be the real world coord of the left top point of the maze grid. 
    
    bool isValid(int x, int y) const; // function to check if a position is is in the maze not in a wall

};

#endif // MAZE_SOLVER_H
