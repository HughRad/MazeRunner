#include "image_processing.h"
#include <iostream>

int main() {
    ImageProcessor processor;
    std::vector<std::string> maze = processor.processMaze("../images/example_maze_cropped.JPG");

    // Print maze representation
    for (const auto &row : maze) {
        std::cout << row << std::endl;
    }

    return 0;
}