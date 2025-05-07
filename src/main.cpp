#include "maze_solver.h"
#include <iostream>
#include <vector>
#include <string>

//..............HOW TO USE THIS EXECUTABLE

int main() {
    //get the maze layout as a string (should be given by Ryan)
    std::vector<std::string> maze = {
        "##########S#########",
        "#...#..............#",
        "###.#.############.#",
        "#...#.#..........#.#",
        "#.###.#.##########.#",
        "#.#...#.#........#.#",
        "#.#.###.#.######.#.#",
        "#...#...#.#....#.#.#",
        "###.#.###.#.##.#.#.#",
        "#...#.....#.#..#.#.#",
        "#.#######.#.#.##.#.#",
        "#.#.......#.#....#.#",
        "#.#.###.###.######.E",
        "#...#.....#......#.#",
        "#####.###.######.#.#",
        "#.....#...#....#.#.#",
        "#.#####.###.#.##.#.#",
        "#....#...#..#......#",
        "#.####.###.#######.#",
        "######E#############"
    };

    // Create a maze_solver object with the maze string as an input
    maze_solver solver(maze);

    //set waypoint parameters
    double scale = 1; //set as distance between maze grid points (manualy measured)
    std::pair<double, double>  world = {0, 0}; //set as the world coords of the mazes left top most point (should be given by nick)
    double rotation = 0; // Rotation of maze in degrees - clockwise rotation (should be given by nick)
    double depth = 0; // set as the drawing depth (should be given by nick)

    solver.scaleSet(scale);
    solver.worldSet(world);
    solver.rotationSet(rotation);
    solver.depthSet(depth);
    
    //solve the maze and output the required robot waypoints in order of execution
    std::vector<geometry_msgs::Pose> robotWaypoints = solver.pathPlaner();
    
    return 0;
}