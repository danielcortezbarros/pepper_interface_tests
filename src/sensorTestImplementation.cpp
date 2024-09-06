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
        RCLCPP_WARN(node->get_logger(), "No valid topic found for BackSonar. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to : %s", topicName.c_str());
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

void frontSonar(std::shared_ptr<rclcpp::Node> node) {
    // Find the respective topic
    std::string topicName = extractTopic("FrontSonar");
    output = true;

    // Check if the topic name is empty
    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for FrontSonar. Skipping this sensor test.");
        return; // Exit the function early if no valid topic is found
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Create a subscriber to the front sonar topic and associate it with the callback function
    auto sub = node->create_subscription<sensor_msgs::msg::Range>(
        topicName, 1, frontSonarMessageReceived);

    // Listen for incoming messages and execute the callback function
    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void frontCamera(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("FrontCamera");
    output = true;

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for FrontCamera. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto sub = image_transport::create_subscription(node.get(), topicName, frontCameraMessageReceived, "raw");

    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    if (isVideoWriterInitialized) {
        videoWriter.release();
        isVideoWriterInitialized = false;
    }

    cv::destroyWindow("Front Camera");
}

void bottomCamera(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("BottomCamera");
    output = true;

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for BottomCamera. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto sub = image_transport::create_subscription(node.get(), topicName, bottomCameraMessageReceived, "raw");

    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    if (isVideoWriterInitialized) {
        videoWriter.release();
        isVideoWriterInitialized = false;
    }

    cv::destroyWindow("Bottom Camera");
}

void depthCamera(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("DepthCamera");
    output = true;

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for DepthCamera. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto sub = image_transport::create_subscription(node.get(), topicName, depthCameraMessageReceived, "raw");

    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    if (isVideoWriterInitialized) {
        videoWriter.release();
        isVideoWriterInitialized = false;
    }

    cv::destroyWindow("Depth Camera");
}

void laserSensor(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("Laser");
    output = true;

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for Laser. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
        topicName, 1, laserSensorMessageReceived);

    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void odom(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("Odometry");
    output = true;

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for Odometry. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
        topicName, 1, odomMessageReceived);

    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void imu(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("IMU");
    output = true;

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for IMU. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
        topicName, 1, imuMessageReceived);

    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

void jointState(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("JointState");
    output = true;

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for JointState. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        topicName, 1, jointStateMessageReceived);

    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

#ifdef PEPPER_ROBOT
void speech(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("Speech");

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for Speech. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Publishing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto pub = node->create_publisher<std_msgs::msg::String>(topicName, 1);
    rclcpp::sleep_for(std::chrono::seconds(1));  // Wait for the connection to establish

    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "This is the CSSR4Africa speaker test.";

    pub->publish(*msg);
    rclcpp::spin_some(node);  // Process incoming messages once. Not typically necessary for a publisher only.

    rclcpp::sleep_for(std::chrono::seconds(1));
}


void stereoCamera(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("StereoCamera");
    output = true;

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for StereoCamera. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto sub = image_transport::create_subscription(node.get(), topicName, stereoCameraMessageReceived, "raw");

    rclcpp::Rate rate(30);
    auto startTime = node->now();
    auto waitTime = rclcpp::Duration::from_seconds(timeDuration);
    auto endTime = startTime + waitTime;

    while (rclcpp::ok() && node->now() < endTime) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    if (isVideoWriterInitialized) {
        videoWriter.release();
        isVideoWriterInitialized = false;
    }

    cv::destroyWindow("Stereo Camera");
}

void microphone(std::shared_ptr<rclcpp::Node> node) {
    std::string topicName = extractTopic("Microphone");
    int sampleRate = 48000;

    if (topicName.empty()) {
        RCLCPP_WARN(node->get_logger(), "No valid topic found for Microphone. Skipping this sensor test.");
        return;
    }

    RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", topicName.c_str());
    rclcpp::sleep_for(std::chrono::seconds(1));

    outAudio.open(getOutputFilePath(), std::ios::binary);
    if (!outAudio.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Unable to open the output file microphoneOutput.wav");
        return;
    }

    auto sub = node->create_subscription<naoqi_driver::msg::AudioCustomMsg>(
        topicName, 1000, microphoneMessageReceived);

    for (int i = 0; i < 4; ++i) {  // Four channels to record
        RCLCPP_INFO(node->get_logger(), "Recording channel: %s", currentChannel.c_str());

        auto startTime = node->now();
        auto endTime = startTime + rclcpp::Duration::from_seconds(timeDuration);

        while (rclcpp::ok() && node->now() < endTime) {
            rclcpp::spin_some(node);
        }

        switchMicrophoneChannel();
    }

    outAudio.seekp(0, std::ios::beg);
    writeWavHeader(outAudio, sampleRate, totalSamples);

    outAudio.close();
    RCLCPP_INFO(node->get_logger(), "Microphone test finished");

    playAndDeleteFile();
}

void microphoneMessageReceived(const naoqi_driver::msg::AudioCustomMsg::SharedPtr msg) {
    if (currentChannel.empty() || !outAudio.is_open()) {
        return;
    }

    const std::vector<int16_t>* channelData = nullptr;

    if (currentChannel == "frontLeft") {
        channelData = &msg->front_left;
    } else if (currentChannel == "frontRight") {
        channelData = &msg->front_right;
    } else if (currentChannel == "rearLeft") {
        channelData = &msg->rear_left;
    } else if (currentChannel == "rearRight") {
        channelData = &msg->rear_right;
    }

    if (channelData) {
        for (const auto& sample : *channelData) {
            outAudio.write(reinterpret_cast<const char*>(&sample), sizeof(sample));
            totalSamples++;
        }
    }
}

void stereoCameraMessageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {
    std::string path;
    std::string videoPath;

    // Extract the path of the ROS package
    path = getOutputFilePath();  // Replace with a ROS2 equivalent function to get the path

    // Extract image attributes from the received message
    int imgWidth = msg->width;
    int imgHeight = msg->height;

    RCLCPP_INFO(rclcpp::get_logger("stereoCamera"), "Image received with width: %d, height: %d", imgWidth, imgHeight);

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";
        if (!isVideoWriterInitialized) {
            videoPath = path + "/stereoCameraOutput.mp4";
        }

        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("stereoCamera"), "Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- STEREO CAMERA ----\n\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        outputFile.close();

        output = false;
    }

    if (!isVideoWriterInitialized) {
        videoWriter.open(videoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 2, cv::Size(imgWidth, imgHeight), true);
        if (!videoWriter.isOpened()) {
            RCLCPP_ERROR(rclcpp::get_logger("stereoCamera"), "Failed to initialize video writer with path: %s", videoPath.c_str());
            return;
        }
        isVideoWriterInitialized = true;
    }

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    // Write frame to video
    videoWriter.write(img);

    cv::imshow("Stereo Camera", img);
    cv::waitKey(30);
}
#endif

void jointStateMessageReceived(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::string path;

    RCLCPP_INFO(rclcpp::get_logger("jointState"), "[MESSAGES] Printing joint state data received.");
    RCLCPP_INFO(rclcpp::get_logger("jointState"), "Header: %s", msg->header.frame_id.c_str());

    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::cout << "Name: " << msg->name[i] << "\n"
                  << "Position: " << (std::isnan(msg->position[i]) ? "NaN" : std::to_string(msg->position[i])) << "\n"
                  << "Velocity: " << (std::isnan(msg->velocity[i]) ? "NaN" : std::to_string(msg->velocity[i])) << "\n"
                  << "Effort: " << (std::isnan(msg->effort[i]) ? "NaN" : std::to_string(msg->effort[i])) << "\n\n";
    }

    // Assuming path management in ROS2
    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("jointState"), "Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- JOINT STATE ----\n\n";
        outputFile << "Header: " << msg->header.frame_id << "\n";
        for (size_t i = 0; i < msg->name.size(); ++i) {
            outputFile << "Name: " << msg->name[i] << "\n";
            outputFile << "Position: " << (std::isnan(msg->position[i]) ? "NaN" : std::to_string(msg->position[i])) << "\n";
            outputFile << "Velocity: " << (std::isnan(msg->velocity[i]) ? "NaN" : std::to_string(msg->velocity[i])) << "\n";
            outputFile << "Effort: " << (std::isnan(msg->effort[i]) ? "NaN" : std::to_string(msg->effort[i])) << "\n\n";
        }
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        outputFile.close();
        output = false;
    }
}

void odomMessageReceived(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::string path;

    RCLCPP_INFO(rclcpp::get_logger("odom"), "[MESSAGES] Printing odometry data received.");
    RCLCPP_INFO(rclcpp::get_logger("odom"), "Header: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(rclcpp::get_logger("odom"), "Child frame id: %s", msg->child_frame_id.c_str());
    RCLCPP_INFO(rclcpp::get_logger("odom"), "Pose: %f", msg->pose.pose.position.x);
    RCLCPP_INFO(rclcpp::get_logger("odom"), "Twist: %f", msg->twist.twist.linear.x);

    // Assuming path management in ROS2
    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("odom"), "Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- ODOMETRY ----\n\n";
        outputFile << "Header: " << msg->header.frame_id << "\n";
        outputFile << "Child frame id: " << msg->child_frame_id << "\n";
        outputFile << "Pose: " << msg->pose.pose.position.x << "\n";
        outputFile << "Twist: " << msg->twist.twist.linear.x << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        outputFile.close();
        output = false;
    }
}

void imuMessageReceived(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::string path;

    RCLCPP_INFO(rclcpp::get_logger("imu"), "[MESSAGES] Printing IMU data received.");
    RCLCPP_INFO(rclcpp::get_logger("imu"), "Header: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(rclcpp::get_logger("imu"), "Orientation: %f", msg->orientation.x);
    RCLCPP_INFO(rclcpp::get_logger("imu"), "Angular velocity: %f", msg->angular_velocity.x);
    RCLCPP_INFO(rclcpp::get_logger("imu"), "Linear acceleration: %f", msg->linear_acceleration.x);

    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("imu"), "Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- IMU ----\n\n";
        outputFile << "Header: " << msg->header.frame_id << "\n";
        outputFile << "Orientation: " << msg->orientation.x << "\n";
        outputFile << "Angular velocity: " << msg->angular_velocity.x << "\n";
        outputFile << "Linear acceleration: " << msg->linear_acceleration.x << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        outputFile.close();
        output = false;
    }
}

void backSonarMessageReceived(const sensor_msgs::msg::Range::SharedPtr msg) {
    std::string path;

    RCLCPP_INFO(rclcpp::get_logger("backSonar"), "[MESSAGES] Printing back sonar data received.");
    RCLCPP_INFO(rclcpp::get_logger("backSonar"), "Frame id: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(rclcpp::get_logger("backSonar"), "Field of view: %f", msg->field_of_view);
    RCLCPP_INFO(rclcpp::get_logger("backSonar"), "Minimum range value: %f", msg->min_range);
    RCLCPP_INFO(rclcpp::get_logger("backSonar"), "Maximum range value: %f", msg->max_range);
    RCLCPP_INFO(rclcpp::get_logger("backSonar"), "Range value: %f", msg->range);

    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("backSonar"), "Unable to open the output file %s", outputFilePath.c_str());
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

void frontSonarMessageReceived(const sensor_msgs::msg::Range::SharedPtr msg) {
    std::string path;

    RCLCPP_INFO(rclcpp::get_logger("frontSonar"), "[MESSAGES] Printing front sonar data received.");
    RCLCPP_INFO(rclcpp::get_logger("frontSonar"), "Frame id: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(rclcpp::get_logger("frontSonar"), "Field of view: %f", msg->field_of_view);
    RCLCPP_INFO(rclcpp::get_logger("frontSonar"), "Minimum range value: %f", msg->min_range);
    RCLCPP_INFO(rclcpp::get_logger("frontSonar"), "Maximum range value: %f", msg->max_range);
    RCLCPP_INFO(rclcpp::get_logger("frontSonar"), "Range value: %f", msg->range);

    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("frontSonar"), "Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- FRONT SONAR ----\n\n";
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

void frontCameraMessageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {
    std::string path;
    std::string videoPath;

    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    int imgWidth = msg->width;
    int imgHeight = msg->height;

    RCLCPP_INFO(rclcpp::get_logger("frontCamera"), "Image received with width: %d and height: %d", imgWidth, imgHeight);

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";

        if (!isVideoWriterInitialized) {
            videoPath = path + "/frontCameraOutput.mp4";
        }

        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("frontCamera"), "Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- FRONT CAMERA ----\n\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        outputFile.close();

        output = false;
    }

    if (!isVideoWriterInitialized) {
        videoWriter.open(videoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 2, cv::Size(imgWidth, imgHeight), true);

        if (!videoWriter.isOpened()) {
            RCLCPP_ERROR(rclcpp::get_logger("frontCamera"), "Failed to initialize video writer with path: %s", videoPath.c_str());
            return;
        }

        isVideoWriterInitialized = true;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat img = cv_ptr->image;

    // Write frame to video
    videoWriter.write(img);

    cv::imshow("Front Camera", img);
    cv::waitKey(30);
}

void bottomCameraMessageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {
    std::string path;
    std::string videoPath;

    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    int imgWidth = msg->width;
    int imgHeight = msg->height;

    RCLCPP_INFO(rclcpp::get_logger("bottomCamera"), "Image received with width: %d and height: %d", imgWidth, imgHeight);

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";

        if (!isVideoWriterInitialized) {
            videoPath = path + "/bottomCameraOutput.mp4";
        }

        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("bottomCamera"), "Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- BOTTOM CAMERA ----\n\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        outputFile.close();

        output = false;
    }

    if (!isVideoWriterInitialized) {
        videoWriter.open(videoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 2, cv::Size(imgWidth, imgHeight), true);

        if (!videoWriter.isOpened()) {
            RCLCPP_ERROR(rclcpp::get_logger("bottomCamera"), "Failed to initialize video writer with path: %s", videoPath.c_str());
            return;
        }

        isVideoWriterInitialized = true;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat img = cv_ptr->image;

    // Write frame to video
    videoWriter.write(img);

    cv::imshow("Bottom Camera", img);
    cv::waitKey(30);
}

void depthCameraMessageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg) {
    std::string path;
    std::string videoPath;

    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    int imgWidth = msg->width;
    int imgHeight = msg->height;

    RCLCPP_INFO(rclcpp::get_logger("depthCamera"), "Image received with width: %d and height: %d", imgWidth, imgHeight);

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";

        if (!isVideoWriterInitialized) {
            videoPath = path + "/depthCameraOutput.mp4";
        }

        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("depthCamera"), "Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- DEPTH CAMERA ----\n\n";
        outputFile << "Image Width: " << imgWidth << "\n";
        outputFile << "Image Height: " << imgHeight << "\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        outputFile.close();

        output = false;
    }

    if (!isVideoWriterInitialized) {
        videoWriter.open(videoPath, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 2, cv::Size(imgWidth, imgHeight), true);

        if (!videoWriter.isOpened()) {
            RCLCPP_ERROR(rclcpp::get_logger("depthCamera"), "Failed to initialize video writer with path: %s", videoPath.c_str());
            return;
        }

        isVideoWriterInitialized = true;
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

    cv::Mat img = cv_ptr->image;

    double min = 0;
    double max = 1000;

    cv::Mat img_scaled_8u;
    cv::Mat(img - min).convertTo(img_scaled_8u, CV_8UC1, 255.0 / (max - min));
    cv::Mat color_img;

    if (img_scaled_8u.type() == CV_8UC1) {
        cv::cvtColor(img_scaled_8u, color_img, cv::COLOR_GRAY2RGB);
    }

    // Write frame to video
    videoWriter.write(color_img);

    cv::imshow("Depth Camera", color_img);
    cv::waitKey(30);
}

void laserSensorMessageReceived(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::string path;

    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "[MESSAGES] Printing laser sensor data received.");
    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "Frame id: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "Start angle of the scan: %f", msg->angle_min);
    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "End angle of the scan: %f", msg->angle_max);
    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "Angular distance between measurements: %f", msg->angle_increment);
    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "Time between measurements: %f", msg->time_increment);
    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "Time between scans: %f", msg->scan_time);
    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "Minimum range value: %f", msg->range_min);
    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "Maximum range value: %f", msg->range_max);
    RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "Range data: (size: %zu)", msg->ranges.size());

    for (const auto& range : msg->ranges) {
        RCLCPP_INFO(rclcpp::get_logger("laserSensor"), "%f", range);
    }

    path = getOutputFilePath();  // Implement a similar function to get the path for ROS2

    if (output) {
        outputFilePath = path + "/sensorTestOutput.dat";
        outputFile.open(outputFilePath.c_str(), std::ofstream::app);
        if (!outputFile.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("laserSensor"), "Unable to open the output file %s", outputFilePath.c_str());
            return;
        }

        outputFile << "[TESTING] ---- LASER SENSOR ----\n\n";
        outputFile << "Frame id: " << msg->header.frame_id << "\n";
        outputFile << "Start angle of the scan: " << msg->angle_min << "\n";
        outputFile << "End angle of the scan: " << msg->angle_max << "\n";
        outputFile << "Angular distance between measurements: " << msg->angle_increment << "\n";
        outputFile << "Time between measurements: " << msg->time_increment << "\n";
        outputFile << "Time between scans: " << msg->scan_time << "\n";
        outputFile << "Minimum range value: " << msg->range_min << "\n";
        outputFile << "Maximum range value: " << msg->range_max << "\n";
        outputFile << "Range data: (size: " << msg->ranges.size() << ")\n";
        outputFile << "[END MESSAGES] Finished printing.\n\n";
        
        outputFile.close();
        output = false;
    }
}

void promptAndExit(int status) {
    std::cout << "Press any key to continue ... " << std::endl;
    std::cin.get();
    exit(status);
}

void promptAndContinue() {
    std::cout << "Press any key to proceed ..." << std::endl;
    std::cin.get();
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


void switchMicrophoneChannel() {
    if (currentChannel == "rearLeft") {
        currentChannel = "rearRight";
    } else if (currentChannel == "rearRight") {
        currentChannel = "frontLeft";
    } else if (currentChannel == "frontLeft") {
        currentChannel = "frontRight";
    } else {
        currentChannel = "";  // Finished recording all channels
    }
}

void writeWavHeader(std::ofstream& file, int sampleRate, int numSamples) {
    int byteRate = sampleRate * 2;  // 16-bit mono = 2 bytes per sample
    int dataSize = numSamples * 2;  // Total number of bytes in data
    int chunkSize = 36 + dataSize;

    file.write("RIFF", 4);  // ChunkID
    file.write(reinterpret_cast<const char*>(&chunkSize), 4);  // ChunkSize
    file.write("WAVE", 4);  // Format
    file.write("fmt ", 4);  // Subchunk1ID
    int subChunk1Size = 16;  // PCM header size
    file.write(reinterpret_cast<const char*>(&subChunk1Size), 4);  // Subchunk1Size
    short audioFormat = 1;  // PCM = 1
    file.write(reinterpret_cast<const char*>(&audioFormat), 2);  // AudioFormat
    short numChannels = 1;  // Mono = 1, Stereo = 2
    file.write(reinterpret_cast<const char*>(&numChannels), 2);  // NumChannels
    file.write(reinterpret_cast<const char*>(&sampleRate), 4);  // SampleRate
    file.write(reinterpret_cast<const char*>(&byteRate), 4);  // ByteRate
    short blockAlign = 2;  // NumChannels * BitsPerSample/8
    file.write(reinterpret_cast<const char*>(&blockAlign), 2);  // BlockAlign
    short bitsPerSample = 16;  // Bits per sample
    file.write(reinterpret_cast<const char*>(&bitsPerSample), 2);  // BitsPerSample
    file.write("data", 4);  // Subchunk2ID
    file.write(reinterpret_cast<const char*>(&dataSize), 4);  // Subchunk2Size
}


void playAndDeleteFile() {
    // Use True/False to delete the file after playing
    bool deleteFile = false;

    // Check if the file exists
    std::string fileName = getOutputFilePath() + "/microphoneOutput.wav";

    // Check if the file exists
    std::ifstream file(fileName);
    if (!file.good()) {
        RCLCPP_ERROR(rclcpp::get_logger("microphone"), "Error: File not found: %s", fileName.c_str());
        return;
    }

    // Play the audio file
    if (std::system(("aplay " + fileName).c_str()) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("microphone"), "Error playing file: %s", fileName.c_str());
        return; // Exit if playing failed
    }

    if (deleteFile) {
        if (std::remove(fileName.c_str()) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("microphone"), "Error deleting file: %s", fileName.c_str());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("microphone"), "File deleted successfully: %s", fileName.c_str());
        }
    }
}

void initializeOutputFile(std::ofstream& outputFile, const std::string& path) {
    outputFile.open(path, std::ofstream::app);
    if (!outputFile.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("sensorTest"), "Unable to open the output file: %s", path.c_str());
        throw std::runtime_error("Failed to open output file.");
    }

    outputFile << "[TESTING] ############ SENSORS ############\n\n";
    outputFile << "[START TIME] " << getCurrentTime() << "\n";

    outputFile.close();
}

void finalizeOutputFile(std::ofstream& outputFile, const std::string& path) {
    outputFile.open(path, std::ofstream::app);
    if (!outputFile.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("output"), "Unable to open the output file: %s", path.c_str());
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

std::string getOutputFilePath() {
    std::string basePath;
    std::string fileName = "sensorTestOutput.dat";

    // Set the base path to where you'd like to store output files
    basePath = ament_index_cpp::get_package_share_directory("pepper_interface_tests");  

    // Append the subdirectory and filename to the base path
    return basePath + "/data/" + fileName;
}

void executeTestsSequentially(const std::vector<std::string>& testNames, std::shared_ptr<rclcpp::Node> node) {
    // Map test names to their corresponding functions
    std::unordered_map<std::string, std::function<void(std::shared_ptr<rclcpp::Node>)>> testMap = {
        {"backsonar", backSonar},
        {"frontsonar", frontSonar},
        {"frontcamera", frontCamera},
        {"bottomcamera", bottomCamera},
        {"depthcamera", depthCamera},
        {"laser", laserSensor},
        {"jointstate", jointState},
        {"odometry", odom},
        {"imu", imu}
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
        {"frontsonar", frontSonar},
        {"frontcamera", frontCamera},
        {"bottomcamera", bottomCamera},
        {"depthcamera", depthCamera},
        {"laser", laserSensor},
        {"jointstate", jointState},
        {"odometry", odom},
        {"imu", imu}
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
















