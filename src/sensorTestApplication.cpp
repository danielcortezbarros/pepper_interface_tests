#include "pepper_interface_tests/sensorTest.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <vector>

/* Main function */
int main(int argc, char **argv) {
    // Initialize the ROS2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sensorTest");

    std::vector<std::string> testNames = extractTests();
    std::string mode = extractMode();

    std::string path = getOutputFilePath();

    std::ofstream out_of;
    initializeOutputFile(out_of, path);

    if (mode == "parallel") {
        executeTestsInParallel(testNames, node);
    } else if (mode == "sequential") {
        executeTestsSequentially(testNames, node);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Invalid mode. Please check the mode in the configuration file.");
        promptAndExit(1);
    }

    finalizeOutputFile(out_of, path);

    // Shutdown the ROS2 node
    rclcpp::shutdown();
    return 0;
}




