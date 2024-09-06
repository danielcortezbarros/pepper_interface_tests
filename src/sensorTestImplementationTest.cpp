/* sensorTestImplementation.cpp
*
* Author: Yohannes Tadesse Haile and Mihirteab Taye Hordofa 
* Date: March 19, 2024
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*/

#include "pepper_interface_tests/sensorTest.h"
#include <rcutils/logging_macros.h>  // Include rcutils logging macros

// Global variables to handle the output file 
bool output;
std::ofstream outputFile;
int timeDuration = 10;
std::string outputFilePath;

// Global variables to handle the audio file 
std::ofstream outAudio;
int totalSamples = 0;
std::string currentChannel = "rearLeft";

// Global variables to handle the video file 
bool saveVideo = false;
cv::VideoWriter videoWriter;
bool isVideoWriterInitialized = false;

void backSonar(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("BackSonar");
    output = true;

    if (topicName.empty()) {
        RCUTILS_LOG_WARN("No valid topic found for BackSonar. Skipping this sensor test.");
        return;
    }

    RCUTILS_LOG_INFO("Subscribing to : %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto sub = node->create_subscription<sensor_msgs::msg::Range>(
        topicName, 1, backSonarMessageReceived);

    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void backSonarMessageReceived(const sensor_msgs::msg::Range::SharedPtr msg) {
    std::string path;

    RCUTILS_LOG_INFO("[MESSAGES] Printing back sonar data received.");
    RCUTILS_LOG_INFO("Frame id: %s", msg->header.frame_id.c_str());
    RCUTILS_LOG_INFO("Field of view: %f", msg->field_of_view);
    RCUTILS_LOG_INFO("Minimum range value: %f", msg->min_range);
    RCUTILS_LOG_INFO("Maximum range value: %f", msg->max_range);
    RCUTILS_LOG_INFO("Range value: %f", msg->range);

    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCUTILS_LOG_ERROR("Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- BACK SONAR ----\n\n";
        outputFile << "Frame id: " << msg->header.frame_id << "\n";
        outputFile << "Field of view: " << msg->field_of_view << "\n";
        outputFile << "Minimum range value: " << msg->min_range << "\n";
        outputFile << "Maximum range value: " << msg->max_range << "\n";
        outputFile << "Range value: " << msg->range << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        outputFile.close();
        output = false;
    }
}

std::string extractTopic(const std::string& key) {
    bool debug = false;
    
    std::string configFileName = "sensorTestConfiguration.ini";  // configuration filename
    std::string packagePath = ament_index_cpp::get_package_share_directory("pepper_interface_tests");  // get package path
    std::string configPathFile = packagePath + "/config/" + configFileName;

    std::string platformKey = "platform";
    std::string robotTopicKey = "robotTopics";
    std::string simulatorTopicKey = "simulatorTopics";

    std::string platformValue;
    std::string robotTopicValue;
    std::string simulatorTopicValue;
    std::string topic_value = "";

    if (debug) {
        RCUTILS_LOG_INFO("Config file is %s", configPathFile.c_str());
    }

    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()) {
        RCUTILS_LOG_ERROR("Unable to open the config file %s", configPathFile.c_str());
        promptAndExit(1);
    }

    std::string configLineRead;
    while (std::getline(configFile, configLineRead)) {
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        if (paramKey == platformKey) { platformValue = paramValue; }
        else if (paramKey == robotTopicKey) { robotTopicValue = paramValue; }
        else if (paramKey == simulatorTopicKey) { simulatorTopicValue = paramValue; }
    }
    configFile.close();

    std::string topicFileName;
    if (platformValue == "simulator") { topicFileName = simulatorTopicValue; }
    else if (platformValue == "robot") { topicFileName = robotTopicValue; }

    std::string topicPathFile = packagePath + "/data/" + topicFileName;
    
    if (debug) {
        RCUTILS_LOG_INFO("Topic file is %s", topicPathFile.c_str());
    }

    std::ifstream topicFile(topicPathFile.c_str());
    if (!topicFile.is_open()) {
        RCUTILS_LOG_ERROR("Unable to open the topic file %s", topicPathFile.c_str());
        promptAndExit(1);
    }

    std::string topicLineRead;
    while (std::getline(topicFile, topicLineRead)) {
        std::istringstream iss(topicLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        if (paramKey == key) {
            topic_value = paramValue;
            break;
        }
    }
    topicFile.close();

    return topic_value;
}



std::string getOutputFilePath() {
    std::string basePath;
    std::string fileName = "sensorTestOutput.dat";

    // Set the base path to where you'd like to store output files
    basePath = ament_index_cpp::get_package_share_directory("pepper_interface_tests");  // Replace with your actual output path

    // Append the subdirectory and filename to the base path
    return basePath + "/data/" + fileName;
}

void promptAndExit(int status) {
    std::cout << "Press any key to continue ... " << std::endl;
    std::cin.get();
    exit(status);
}

void finalizeOutputFile(std::ofstream& outputFile, const std::string& path) {
    outputFile.open(path, std::ofstream::app);
    if (!outputFile.is_open()) {
        RCUTILS_LOG_ERROR("Unable to open the output file: %s", path.c_str());
        return;
    }

    outputFile << "[END TIME] " << getCurrentTime() << "\n\n";
    outputFile.close();
}

std::string getCurrentTime() {
    char buffer[50];
    std::time_t now = std::time(0);
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %X", std::localtime(&now));
    return std::string(buffer);
}

std::vector<std::string> extractTests() {
    bool debug = false;

    std::string inputFileName = "sensorTestInput.ini";  // input filename
    std::string packagePath = ament_index_cpp::get_package_share_directory("pepper_interface_tests");  // get package path
    std::string inputPathFile = packagePath + "/config/" + inputFileName;

    std::vector<std::string> testName;
    if (debug) {
        RCUTILS_LOG_INFO("Input file is %s", inputPathFile.c_str());
    }

    std::ifstream inputFile(inputPathFile.c_str());
    if (!inputFile.is_open()) {
        RCUTILS_LOG_ERROR("Unable to open the input file %s", inputPathFile.c_str());
        throw std::runtime_error("Failed to open input file.");
    }

    std::string inpLineRead;
    while (std::getline(inputFile, inpLineRead)) {
        std::istringstream iss(inpLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;

        trim(paramValue); 
        std::transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);
        std::transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);

        if (paramValue == "true") {
            testName.push_back(paramKey);
        }
    }
    inputFile.close();

    return testName;
}

std::string extractMode() {
    bool debug = false;

    std::string configFileName = "sensorTestConfiguration.ini";  // configuration filename
    std::string packagePath = ament_index_cpp::get_package_share_directory("pepper_interface_tests");  // get package path
    std::string configPathFile = packagePath + "/config/" + configFileName;

    std::string modeKey = "mode";
    std::string modeValue;

    if (debug) {
        RCUTILS_LOG_INFO("Config file is %s", configPathFile.c_str());
    }

    std::ifstream configFile(configPathFile.c_str());
    if (!configFile.is_open()) {
        RCUTILS_LOG_ERROR("Unable to open the config file %s", configPathFile.c_str());
        throw std::runtime_error("Failed to open config file.");
    }

    std::string configLineRead;
    while (std::getline(configFile, configLineRead)) {
        std::istringstream iss(configLineRead);
        std::string paramKey, paramValue;
        iss >> paramKey;
        trim(paramKey);
        std::getline(iss, paramValue);
        iss >> paramValue;
        trim(paramValue);

        std::transform(paramKey.begin(), paramKey.end(), paramKey.begin(), ::tolower);
        std::transform(paramValue.begin(), paramValue.end(), paramValue.begin(), ::tolower);

        if (paramKey == modeKey) {
            modeValue = paramValue;
        }
    }
    configFile.close();

    if (modeValue.empty()) {
        RCUTILS_LOG_ERROR("Unable to find a valid mode.");
        throw std::runtime_error("Mode not found.");
    }

    return modeValue;
}

void initializeOutputFile(std::ofstream& outputFile, const std::string& path) {
    outputFile.open(path, std::ofstream::app);
    if (!outputFile.is_open()) {
        RCUTILS_LOG_ERROR("Unable to open the output file: %s", path.c_str());
        throw std::runtime_error("Failed to open output file.");
    }

    outputFile << "[TESTING] ############ SENSORS ############\n\n";
    outputFile << "[START TIME] " << getCurrentTime() << "\n";

    outputFile.close();
}

void executeTestsSequentially(const std::vector<std::string>& testNames, std::shared_ptr<rclcpp::Node> node) {
    // Map test names to their corresponding functions
    std::unordered_map<std::string, std::function<void(std::shared_ptr<rclcpp::Node>)>> testMap = {
        {"backsonar", backSonar},
        // {"frontsonar", frontSonar},
        // {"frontcamera", frontCamera},
        // {"bottomcamera", bottomCamera},
        // {"depthcamera", depthCamera},
        // {"laser", laserSensor},
        // {"jointstate", jointState},
        // {"odometry", odom},
        // {"imu", imu}
    };

    #ifdef PEPPER_ROBOT
    testMap["stereocamera"] = stereoCamera;
    testMap["microphone"] = microphone;
    testMap["speech"] = speech;
    #endif

    for (const auto& testName : testNames) {
        auto it = testMap.find(testName);
        if (it != testMap.end()) {
            // Call the function associated with testName
            it->second(node);
        } else {
            RCUTILS_LOG_WARN("Unknown test provided: %s. Proceeding to the next test...", testName.c_str());
        }
    }
}

void executeTestsInParallel(const std::vector<std::string>& testNames, std::shared_ptr<rclcpp::Node> node) {
    // Map test names to their corresponding functions
    std::unordered_map<std::string, std::function<void(std::shared_ptr<rclcpp::Node>)>> testMap = {
        {"backsonar", backSonar},
        // Add other test functions here...
    };

    std::vector<std::thread> threads;

    // Create a thread for each test found in the map
    for (const auto& testName : testNames) {
        auto it = testMap.find(testName);
        if (it != testMap.end()) {
            // Use std::ref to pass the node by reference
            threads.emplace_back(it->second, std::ref(node));
        } else {
            RCUTILS_LOG_WARN("There is no test function associated with the test name: %s. Proceeding to the next test...", testName.c_str());
        }
    }

    // Wait for all threads to finish execution
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}

