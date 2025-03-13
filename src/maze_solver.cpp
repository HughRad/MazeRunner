#include "maze_solver.h"
#include <iostream>

maze_solver::maze_solver(const std::vector<std::string>& mazeStr) {
    // Initialize maze dimensions
    rows = mazeStr.size();
    cols = rows > 0 ? mazeStr[0].size() : 0;
    
    // Initialize the maze grid
    maze.resize(rows, std::vector<char>(cols));
    
    // Fill the maze and find start/end positions
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
    
    // Check if the position is not a wall (assuming '#' represents walls)
    return maze[x][y] != '#';
}

std::vector<std::pair<int, int>> maze_solver::solve() {
    // Queue for BFS
    std::queue<std::pair<int, int>> q;
    
    // Keep track of visited cells
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    
    // Parent array to reconstruct the path
    std::vector<std::vector<std::pair<int, int>>> parent(rows, std::vector<std::pair<int, int>>(cols, {-1, -1}));
    
    // Start BFS from the start position
    q.push(start);
    visited[start.first][start.second] = true;
    
    bool foundPath = false;
    
    // BFS
    while (!q.empty() && !foundPath) {
        auto current = q.front();
        q.pop();
        
        int x = current.first;
        int y = current.second;
        
        // If end is reached
        if (x == end.first && y == end.second) {
            foundPath = true;
            break;
        }
        
        // Try all four directions
        for (int i = 0; i < 4; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];
            
            // If valid and not visited
            if (isValid(newX, newY) && !visited[newX][newY]) {
                q.push({newX, newY});
                visited[newX][newY] = true;
                parent[newX][newY] = {x, y}; // Remember the parent
            }
        }
    }
    
    // Reconstruct the path if found
    std::vector<std::pair<int, int>> path;
    
    if (foundPath) {
        // Start from the end and follow parent pointers
        auto current = end;
        
        while (current.first != -1 && current.second != -1) {
            path.push_back(current);
            current = parent[current.first][current.second];
        }
        
        // Reverse the path to get from start to end
        std::reverse(path.begin(), path.end());
    }
    
    return path;
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