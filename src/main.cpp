#include "image_processing.h"
#include <iostream>

int main() {
    ImageProcessor processor;
    std::vector<std::string> maze = processor.processMaze("../example_maze.JPG");

    // Print maze representation
    for (const auto &row : maze) {
        std::cout << row << std::endl;
    }

    return 0;
}