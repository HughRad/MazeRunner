#include "pattern_detector.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pattern_detector_node");
    ros::NodeHandle nh;
    
    PatternDetector detector(nh);
    
    ros::spin();
    
    return 0;
}
