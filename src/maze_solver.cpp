#include "maze_solver.h"
#include <iostream>
#include <algorithm> 

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
    // defualt settings for scale and world coord
    world_ = {0, 0};
    scale_ = 1;
    rotation_ = 0;
    depth_ = 0;

    // Get maze dimensions from the ascii input
    rows = mazeStr.size();
    cols = rows > 0 ? mazeStr[0].size() : 0;
    
    // Create an empty maze grid based on the string ascii representation size
    maze.resize(rows, std::vector<char>(cols));
    
    // Fills the empty maze grid with characters. Effectivly converts the incoming string maze to a char maze. Also maps out the start and end points
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            maze[i][j] = mazeStr[i][j];
            
            // Assuming 'S' represents the start
            if (maze[i][j] == 'S') {
                start = {i, j};
            }
            // Assuming 'E' represents the end
            else if (maze[i][j] == 'E') {
                end = {i, j};
            }
        }
    }
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
    // Queue for BFS, stores cells to be explored next
    std::queue<std::pair<int, int>> q;
    
    // array to keep track of visited cells, creates a row x col array of bool false entries
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    
    // Parent array to reconstruct the path, creates a row x col array of pairs set to (-1,-1). When we move to a new tile in the maze,
    // the corisponding point in the parent array is set to whatever the last coordinate was. This helps reconstuct our path later.
    std::vector<std::vector<std::pair<int, int>>> parent(rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));
    
    // Start BFS from the start position
    q.push(start);
    visited[start.first][start.second] = true; // Mark this cell as visited
    
    bool foundPath = false; // Path found = false until end is found 
    
    // BFS
    while (!q.empty() && !foundPath) {
        auto current = q.front(); // Set the current cell being procesed as the one in the front of the queue; the order of discovery
        q.pop(); // remove that same value was we are currently proccesing it
        
        int x = current.first;
        int y = current.second;
        
        // Check if this point is the end
        if (x == end.first && y == end.second) {
            foundPath = true;
            break; // break while loop if true
        }
        
        // Explore all 4 ajacent grids to the current one being proccesed by applying the dx dy modifiers to the current xy
        for (int i = 0; i < 4; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];
            
            // If valid and not visited
            if (isValid(newX, newY) && !visited[newX][newY]) {
                q.push({newX, newY}); // Push that point into the queue to be proccesed next
                visited[newX][newY] = true; // Also mark this new point as visited to avoid revisiting later
                parent[newX][newY] = {x, y}; // Remember the parent of the new point so we can determine the path later
            }
        }
    }
    
    // When the while loop is finished (either cause the end was found or all paths were explored)
    // create a vector or grid coordinates that will be all the cells required to solve the maze
    std::vector<std::pair<int, int>> path; 
    
    if (foundPath) { // if the end was found, fill this vector out, otherwise it will remain empty
        auto current = end; // Start from the end and follow parent array to backtrack to the start 
        
        while (current.first != -1 && current.second != -1) { // while parent is not (-1 -1), the start position
            path.push_back(current);
            current = parent[current.first][current.second]; // set current as the last currents perent and continue to backtrack
        }
        
        // One we reach the start again at (-1 -1), reverse the path to get from start to end
        std::reverse(path.begin(), path.end());
    }
    
    return path; //return the vector of solution coordinates
}

std::vector<geometry_msgs::Pose> maze_solver::generateWaypoints(const std::vector<std::pair<int, int>>& path) const {
    if (path.empty()) {
        return {};
    }
    
    std::vector<std::pair<int, int>> waypoints; 
    std::vector<geometry_msgs::Pose> converted_waypoints;

    waypoints.push_back(path[0]); // include the start point
    
    if (path.size() == 1) { // If the path has only one point, return just that (after conversion to double and applying scale mods)

        double angle_rad = rotation_ * M_PI / 180.0; 

        double x = static_cast<double>(waypoints[0].first);
        double y = static_cast<double>(waypoints[0].second);

        double rotated_x = (x * cos(angle_rad) - y * sin(angle_rad));
        double rotated_y = (x * sin(angle_rad) + y * cos(angle_rad));

        double scaled_x = (rotated_x * scale_ * (-1)) + world_.first;
        double scaled_y = (rotated_y * scale_) + world_.second;

        geometry_msgs::Pose pose;
        pose.position.x = scaled_x;
        pose.position.y = scaled_y;
        pose.position.z = depth_; 
        
        // Set orientation as identity quaternion (no rotation)
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        
        converted_waypoints.push_back(pose);
        return converted_waypoints;
    }
    
    int currentDirection = 0; // Direction of movement (0 = undefined, 1 = horizontal, 2 = vertical)
    
    for (int i = 1; i < path.size(); i++) {// Check to see if the path is currently moving horizontaly or verticaly
        int dx = path[i].first - path[i-1].first;
        int dy = path[i].second - path[i-1].second;
        
        int newDirection = 0;
        if (dx != 0) newDirection = 2; // Vertical movement
        else if (dy != 0) newDirection = 1; // Horizontal movement
        
        // for the first step, set the initial direction
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
    
    if (waypoints.back() != path.back()) { //include the end point if its not already included
        waypoints.push_back(path.back());
    }

    double angle_rad = rotation_ * M_PI / 180.0; // Convert degrees to radians

    geometry_msgs::Pose pose;
    pose.position.z = depth_; 

    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    for (auto& point : waypoints) { // modify the waypoints by the real world scale and convert the int values to doubles
        double y = static_cast<double>(point.first);
        double x = static_cast<double>(point.second);
        // The negative is nessasry to offset inverse nature of the maze.
        // left top of the maze is (0, 0), so to stop going down one being y = 1, which would not work on the real robot, we flip the sign
        
        // Apply rotation
        double rotated_x = (x * cos(angle_rad) - y * sin(angle_rad));
        double rotated_y = (x * sin(angle_rad) + y * cos(angle_rad));

        double scaled_x = (rotated_x * scale_ ) + world_.first;
        double scaled_y = (rotated_y * scale_* (-1)) + world_.second;
                
        pose.position.x = scaled_x;
        pose.position.y = scaled_y;
            
        converted_waypoints.push_back(pose);
    }

    return converted_waypoints;
}

void maze_solver::scaleSet(const double& scale){
    if (scale > 0){ // otherwise default scale is used
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
        // Don't overwrite start and end
        if (solutionMaze[pos.first][pos.second] != 'S' && 
            solutionMaze[pos.first][pos.second] != 'E') {
            solutionMaze[pos.first][pos.second] = '*';
        }
    }
    
    // Print the solution
    std::cout << "Solution:" << std::endl;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            std::cout << solutionMaze[i][j];
        }
        std::cout << std::endl;
    }
}

void maze_solver::printWaypoints(const std::vector<geometry_msgs::Pose>& waypoints) const {
    std::cout << "\nWaypoints for robot navigation:" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    std::cout << "Total waypoints: " << waypoints.size() << std::endl;
    
    for (size_t i = 0; i < waypoints.size(); i++) {
        const auto& pose = waypoints[i];
        std::cout << "Position " << i+1 << ": (" << pose.position.x << "," << pose.position.y << "," << pose.position.z << ")" << std::endl;
    }
}