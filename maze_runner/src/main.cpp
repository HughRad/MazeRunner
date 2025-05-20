// #include "robot_control.cpp"

// // /**
// //  * @brief Main function for the drawing robot node
// //  * 
// //  * Initializes the ROS node, sets up the drawing robot, and executes the
// //  * maze solving and drawing process.
// //  * 
// //  * @param argc Number of command line arguments
// //  * @param argv Command line arguments
// //  * @return int Exit code
// //  */

// // int main(int argc, char** argv)
// // {
// //     ros::init(argc, argv, "maze_runner");
// //     ros::AsyncSpinner spinner(2); // Use 2 threads for better responsiveness
// //     spinner.start();

// //     ros::NodeHandle nh;
// //     bool loop = true;  // Change to false if you want the main logic to run only once

// //     while (ros::ok())
// //     {
// //         // Wait for the GUI to set /robot_ready to true
// //         ROS_INFO("Waiting for GUI to signal robot start via /robot_ready param...");
// //         ros::Rate rate(2);  // 2 Hz check
// //         bool ready = false;

// //         while (ros::ok())
// //         {
// //             nh.getParam("/robot_ready", ready);
// //             if (ready) break;
// //             rate.sleep();
// //         }

// //         // Reset flag so it must be pressed again if loop is true
// //         nh.setParam("/robot_ready", false);

// //         ROS_INFO("Starting UR3 Cartesian waypoint following program");

// //         try {
// //             DrawingRobot robot;
// //             ImageProcessor processor;

// //             ros::Duration(5.0).sleep(); // Ensure robot is connected

// //             std::vector<double> current_joints = robot.getCurrentJointPositions();
// //             std::string joints_str = "";
// //             for (double val : current_joints)
// //                 joints_str += std::to_string(val) + " ";
// //             ROS_INFO("Robot starting joint positions: [%s]", joints_str.c_str());

// //             std::vector<double> initial_joint_positions = {0.0, -1.452, 0.623, -0.748, -1.571, 0.0};
// //             bool init_success = robot.initializeToPosition(initial_joint_positions);

// //             if (!init_success) {
// //                 ROS_WARN("Initial movement failed. Attempting recovery...");
// //                 robot.recoverFromJointStateError();
// //                 init_success = robot.initializeToPosition(initial_joint_positions);
// //                 if (!init_success) {
// //                     ROS_ERROR("Failed to initialize robot after recovery attempt");
// //                     continue; // Skip this cycle and wait for next GUI trigger
// //                 }
// //             }

// //             ros::Duration(5.0).sleep();

// //             ros::ServiceClient aruco_client = nh.serviceClient<std_srvs::Empty>("aruco_tracker/start");

// //             ROS_INFO("Waiting for aruco_tracker/start service...");
// //             if (!ros::service::waitForService("aruco_tracker/start", ros::Duration(10.0))) {
// //                 ROS_ERROR("Service aruco_tracker/start not available after waiting");
// //                 continue;
// //             }

// //             std_srvs::Empty srv;
// //             if(aruco_client.call(srv)) {
// //                 ROS_INFO("Successfully called aruco_tracker/start service");
// //             } else {
// //                 ROS_ERROR("Failed to call aruco_tracker/start service");
// //                 continue;
// //             }

// //             robot.enableImageBasedControl();

// //             ros::Time start_time = ros::Time::now();
// //             const double timeout = 120.0;
// //             ROS_INFO("Waiting for snapshot from ArUco tracker...");

// //             while (!robot.image_processed_ && (ros::Time::now() - start_time).toSec() < timeout) {
// //                 ros::Duration(0.1).sleep();
// //                 ros::spinOnce();
// //             }

// //             if (!robot.image_processed_) {
// //                 ROS_ERROR("Snapshot not received in time. Aborting this run.");
// //                 continue;
// //             }

// //             ROS_INFO("Snapshot received successfully.");

// //             std::vector<std::string> maze = processor.processMaze(robot.received_image_);

// //             maze_solver solver(maze);
// //             geometry_msgs::Point maze_corner = robot.corner_waypoint_;
// //             double scale = 0.008;
// //             std::pair<double, double> world = {maze_corner.x, maze_corner.y};
// //             double rotation = 90 + robot.getMazeRotation();
// //             double depth = 0.158;

// //             solver.scaleSet(scale);
// //             solver.worldSet(world);
// //             solver.rotationSet(rotation);
// //             solver.depthSet(depth);

// //             std::vector<geometry_msgs::Pose> maze_path = solver.pathPlaner();

// //             ROS_INFO("Executing Cartesian path...");
// //             ros::Duration(1.0).sleep();
// //             bool path_success = robot.executeCartesianPath(maze_path);

// //             ROS_INFO("Returning to initial joint configuration...");
// //             bool return_success = robot.initializeToPosition(initial_joint_positions);
// //             if (!return_success) {
// //                 ROS_WARN("Return failed. Attempting recovery...");
// //                 robot.recoverFromJointStateError();
// //                 robot.initializeToPosition(initial_joint_positions);
// //             }

// //             ROS_INFO("Maze execution completed successfully.");
// //         }
// //         catch (const std::exception& e) {
// //             ROS_ERROR("Exception in main: %s", e.what());
// //         }
// //         catch (...) {
// //             ROS_ERROR("Unknown exception in main");
// //         }

// //         if (!loop) {
// //             break;  // Exit after one loop
// //         }

// //         ROS_INFO("Waiting for next start signal from GUI...");
// //     }

// //     ros::shutdown();
// //     return 0;
// // }