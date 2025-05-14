#include "maze_solver.h"
#include <iostream>
#include <algorithm> 
#include <cmath>
#include <limits>

std::vector<geometry_msgs::Pose> maze_solver::pathPlaner(){
    // Solve the maze
    auto path = solve();

    // Print the ASCII solution (for debugging)
    printSolution(path);

    // Generate waypoints as geometry_msgs
    auto waypoints = generateWaypoints(path);

    // Print output to terminal to verify
    printWaypoints(waypoints);

    return waypoints;
}

maze_solver::maze_solver(const std::vector<std::string>& mazeStr) {
    // default settings for scale and world coord
    world_ = {0, 0};
    scale_ = 1;
    rotation_ = 0;
    depth_ = 0;

    // Get maze dimensions from the ascii input
    rows = mazeStr.size();
    cols = rows > 0 ? mazeStr[0].size() : 0;
    
    // Create an empty maze grid based on the string ascii representation size
    maze.resize(rows, std::vector<char>(cols));
    
    // Fills the empty maze grid with characters. Effectively converts the incoming string maze to a char maze. Also maps out the start and exit points
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            maze[i][j] = mazeStr[i][j];
            
            // Assuming 'S' represents the start
            if (maze[i][j] == 'S') {
                start = {i, j};
            }
            // Assuming 'E' represents an exit - collect all exits
            else if (maze[i][j] == 'E') {
                exits.push_back({i, j});
            }
        }
    }
    
    // If no exits were found, print an error
    if (exits.empty()) {
        std::cout << "Error: No exit points ('E') found in the maze!" << std::endl;
    }
    
    // closest_exit will be determined in the solve() method
}

bool maze_solver::isValid(int x, int y) const {
    // Check if position is within the maze bounds
    if (x < 0 || x >= rows || y < 0 || y >= cols) {
        return false;
    }
    
    // Check if the position is not a wall, otherwise return true
    return maze[x][y] != '#';
}

std::vector<std::pair<int, int>> maze_solver::solve() {
    if (exits.empty()) {
        return {}; // Return empty path if no exits
    }
    
    // Queue for BFS, stores cells to be explored next
    std::queue<std::pair<int, int>> q;
    
    // Array to keep track of visited cells, creates a row x col array of bool false entries
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    
    // Parent array to reconstruct the path, creates a row x col array of pairs set to (-1,-1).
    std::vector<std::vector<std::pair<int, int>>> parent(rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));
    
    // Distance array to track the distance from start to each cell
    std::vector<std::vector<int>> distance(rows, std::vector<int>(cols, std::numeric_limits<int>::max()));
    
    // Start BFS from the start position
    q.push(start);
    visited[start.first][start.second] = true; // Mark this cell as visited
    distance[start.first][start.second] = 0;   // Distance to start is 0
    
    int min_distance = std::numeric_limits<int>::max();
    bool found_any_exit = false;
    
    // BFS
    while (!q.empty()) {
        auto current = q.front(); // Set the current cell being processed as the one in the front of the queue
        q.pop(); // Remove that same value as we are currently processing it
        
        int x = current.first;
        int y = current.second;
        int current_distance = distance[x][y];
        
        // Check if this point is an exit
        for (const auto& exit : exits) {
            if (x == exit.first && y == exit.second) {
                found_any_exit = true;
                
                // If this exit is closer than any previously found, update the closest exit
                if (current_distance < min_distance) {
                    min_distance = current_distance;
                    closest_exit = exit;
                }
                
                // Don't break - we want to check all exits that we reach at this distance
            }
        }
        
        // If we've found an exit and we're now exploring cells that are further away than our closest exit,
        // we can stop the BFS since we've already found the closest exit
        if (found_any_exit && current_distance > min_distance) {
            break;
        }
        
        // Explore all 4 adjacent grids to the current one being processed
        for (int i = 0; i < 4; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];
            
            // If valid and not visited
            if (isValid(newX, newY) && !visited[newX][newY]) {
                q.push({newX, newY}); // Push that point into the queue to be processed next
                visited[newX][newY] = true; // Also mark this new point as visited to avoid revisiting later
                parent[newX][newY] = {x, y}; // Remember the parent of the new point so we can determine the path later
                distance[newX][newY] = current_distance + 1; // Update the distance
            }
        }
    }
    
    // When the while loop is finished, create a vector of grid coordinates for the solution path
    std::vector<std::pair<int, int>> path; 
    
    if (found_any_exit) {
        // Start from the closest exit and follow parent array to backtrack to the start
        auto current = closest_exit;
        
        // While we haven't reached the start
        while (current.first != -1 && current.second != -1) {
            path.push_back(current);
            current = parent[current.first][current.second]; // Set current as the parent of the current position
        }
        
        // Reverse the path to get from start to exit
        std::reverse(path.begin(), path.end());
    } else {
        std::cout << "No path found to any exit!" << std::endl;
    }
    
    return path;
}

std::vector<geometry_msgs::Pose> maze_solver::generateWaypoints(const std::vector<std::pair<int, int>>& path) const {
    if (path.empty() || path.size() == 1) {
        std::cout << "Invalid maze (no usable waypoints could be created)" << std::endl;
        return {};
    }
    
    std::vector<std::pair<int, int>> waypoints; 
    std::vector<geometry_msgs::Pose> converted_waypoints;

    waypoints.push_back(path[0]); // Include the start point
    
    int currentDirection = 0; // Direction of movement (0 = undefined, 1 = horizontal, 2 = vertical)
    
    for (int i = 1; i < path.size(); i++) {
        // Check to see if the path is currently moving horizontally or vertically
        int dx = path[i].first - path[i-1].first;
        int dy = path[i].second - path[i-1].second;
        
        int newDirection = 0;
        if (dx != 0) newDirection = 2; // Vertical movement
        else if (dy != 0) newDirection = 1; // Horizontal movement
        
        // For the first step, set the initial direction
        if (i == 1) {
            currentDirection = newDirection;
            continue;
        }
        
        // If direction changed, we found a corner, and thus a usable waypoint
        if (newDirection != 0 && newDirection != currentDirection) {
            waypoints.push_back(path[i-1]); // Add the previous point as a waypoint
            currentDirection = newDirection;
        }
    }
    
    if (waypoints.back() != path.back()) { // Include the exit point if it's not already included
        waypoints.push_back(path.back());
    }

    double angle_rad = rotation_ * M_PI / 180.0; // Convert degrees to radians

    geometry_msgs::Pose pose;
    pose.position.z = depth_; 

    pose.orientation.x = 1.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;

    for (auto& point : waypoints) { // Modify the waypoints by the real world scale and convert the int values to doubles
        double y = static_cast<double>(point.first);
        double x = static_cast<double>(point.second);
        // The negative is necessary to offset inverse nature of the maze.
        // Left top of the maze is (0, 0), so to stop going down one being y = 1, which would not work on the real robot, we flip the sign
        
        // Apply rotation
        double rotated_x = (x * cos(angle_rad) - y * sin(angle_rad));
        double rotated_y = (x * sin(angle_rad) + y * cos(angle_rad));

        double scaled_x = (rotated_x * scale_) + world_.first;
        double scaled_y = (rotated_y * scale_ * (-1)) + world_.second;
                
        pose.position.x = scaled_x;
        pose.position.y = scaled_y;
            
        converted_waypoints.push_back(pose);
    }

    return converted_waypoints;
}

void maze_solver::scaleSet(const double& scale){
    if (scale > 0){ // Otherwise default scale is used
        scale_ = scale;
    }
}

void maze_solver::worldSet(std::pair<double, double>& world){
    world_ = world;
}

void maze_solver::rotationSet(const double& rotation){
    rotation_ = rotation;
}

void maze_solver::depthSet(const double& depth){
    depth_ = depth;
}

void maze_solver::printSolution(const std::vector<std::pair<int, int>>& path) const {
    if (path.empty()) {
        std::cout << "No solution found!" << std::endl;
        return;
    }
    
    // Create a copy of the maze
    std::vector<std::vector<char>> solutionMaze = maze;
    
    // Mark the path with '*'
    for (const auto& pos : path) {
        // Don't overwrite start and exits
        if (solutionMaze[pos.first][pos.second] != 'S' && 
            solutionMaze[pos.first][pos.second] != 'E') {
            solutionMaze[pos.first][pos.second] = '*';
        }
    }
    
    // Print the solution
    std::cout << "Solution (path to closest exit):" << std::endl;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            std::cout << solutionMaze[i][j];
        }
        std::cout << std::endl;
    }
    
    // Print information about the chosen exit
    if (!exits.empty() && !path.empty()) {
        std::cout << "\nChosen exit at position: (" << closest_exit.first << ", " << closest_exit.second << ")" << std::endl;
        std::cout << "Path length to this exit: " << path.size() - 1 << " steps" << std::endl;
    }
}

void maze_solver::printWaypoints(const std::vector<geometry_msgs::Pose>& waypoints) const {
    std::cout << "\nWaypoints for robot navigation:" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    std::cout << "Total waypoints: " << waypoints.size() << std::endl;
    
    for (size_t i = 0; i < waypoints.size(); i++) {
        const auto& pose = waypoints[i];
        std::cout << "Position " << i+1 << ": (" << pose.position.x << "," << pose.position.y << "," << pose.position.z << ")" << std::endl;
        std::cout << "Orientation " << i+1 << ": (" << pose.orientation.x << "," << pose.orientation.y << "," << pose.orientation.z << ")" << std::endl;

        double yaw = atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                          1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z));
        std::cout << "  Yaw: " << yaw * 180.0 / M_PI << " degrees" << std::endl;
    }
}