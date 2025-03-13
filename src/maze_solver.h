#ifndef MAZE_SOVLER_H
#define MAZE_SOVLER_H

#include <vector>
#include <string>
#include <queue>
#include <utility>
#include <algorithm>

class maze_solver
{
public:
    
    maze_solver(const std::vector<std::string>& mazeStr); // Constructor

    std::vector<std::pair<int, int>> solve(); // Solve the maze using BFS and return the path

    void printSolution(const std::vector<std::pair<int, int>>& path) const; // Print the maze with the solution path

private:

    std::vector<std::vector<char>> maze; // 2D grid representation of the maze
    
    int rows; // Dimensions of the maze
    int cols;
    
    std::pair<int, int> start; // Start and end positions
    std::pair<int, int> end;
    
    const int dx[4] = {-1, 0, 1, 0}; // Direction vectors for 4 possible moves (up, right, down, left)
    const int dy[4] = {0, 1, 0, -1};
    
    bool isValid(int x, int y) const; // Helper function to check if a position is valid

};

#endif // MAZE_SOLVER_H
