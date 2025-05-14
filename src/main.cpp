#include "image_processing.h"
#include <iostream>
#include <chrono>

int main() {
    ImageProcessor processor;
    ImageProcessor::DebugInfo debugInfo;
    
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();
    
    // Process maze with debug info
    std::vector<std::string> maze = processor.processMaze("../images/rotated_test2.jpg", &debugInfo);
    
    // End timer
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // Print maze representation
    for (const auto &row : maze) {
        std::cout << row << std::endl;
    }

    // Print processing time
    std::cout << "\nProcessing time: " << duration.count() << " milliseconds" << std::endl;

    // Wait for key press to close windows
    cv::waitKey(0);

    return 0;
}