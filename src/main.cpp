#include "maze_solver.h"
#include <iostream>
#include <vector>
#include <string>

int main() {
    // Example maze
    // S = Start
    // E = End
    // # = Wall
    // . = Open path
    std::vector<std::string> maze = {
        "S#######",
        "..######",
        "#.######",
        "#.###...",
        "#.###.#.",
        "#.....#.",
        "#.#####.",
        "#.......",
        "####.###",
        "####.###",
        "####..E#",
        "########"
    };
    
    // Create a solver
    maze_solver solver(maze);
    
    // Solve the maze
    auto path = solver.solve();
    
    // Print the solution
    solver.printSolution(path);
    
    return 0;
}