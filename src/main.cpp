#include "image_processing.h"
#include <iostream>
#include <chrono>

int main() {
    ImageProcessor processor;
    
    // Process maze with debug info
    std::vector<std::string> maze = processor.processMaze("../images/test2.jpg");

    // Wait for key press to close windows
    cv::waitKey(0);

    return 0;
}