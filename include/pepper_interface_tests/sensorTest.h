#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <fstream>
#include <string>
#include <unordered_map>
#include <boost/algorithm/string.hpp>
#include <ctime>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcutils/logging_macros.h>
#ifdef PEPPER_ROBOT
#include <naoqi_driver/AudioCustomMsg.h>


#endif // DEBUG

using namespace boost;
using namespace std;
using TestFunction = std::function<void(std::shared_ptr<rclcpp::Node>)>;
extern bool output;
extern int timeDuration;

#define ROS

void backSonar(std::shared_ptr<rclcpp::Node> node);
void frontSonar(std::shared_ptr<rclcpp::Node> node);
void frontCamera(std::shared_ptr<rclcpp::Node> node);
void bottomCamera(std::shared_ptr<rclcpp::Node> node);
void depthCamera(std::shared_ptr<rclcpp::Node> node);
void stereoCamera(std::shared_ptr<rclcpp::Node> node);
void laserSensor(std::shared_ptr<rclcpp::Node> node);
void jointState(std::shared_ptr<rclcpp::Node> node);
void odom(std::shared_ptr<rclcpp::Node> node);
void imu(std::shared_ptr<rclcpp::Node> node);
void speech(std::shared_ptr<rclcpp::Node> node);

/* Call back functions executed when a sensor data arrives */
void backSonarMessageReceived(const sensor_msgs::msg::Range::SharedPtr msg);
void frontSonarMessageReceived(const sensor_msgs::msg::Range::SharedPtr msg);
void frontCameraMessageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg);
void bottomCameraMessageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg);
void depthCameraMessageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg);
void stereoCameraMessageReceived(const std::shared_ptr<const sensor_msgs::msg::Image>& msg);
void laserSensorMessageReceived(const sensor_msgs::msg::LaserScan::SharedPtr msg);
void jointStateMessageReceived(const sensor_msgs::msg::JointState::SharedPtr msg);
void odomMessageReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
void imuMessageReceived(const sensor_msgs::msg::Imu::SharedPtr msg);

#ifdef PEPPER_ROBOT
void microphone(std::shared_ptr<rclcpp::Node> node);
void microphoneMessageReceived(const naoqi_driver::msg::AudioCustomMsg::SharedPtr msg);
#endif // DEBUG

std::vector<string> extractTests();
std::string extractTopic(const std::string& key);   
std::string extractMode();
void writeWavHeader(std::ofstream &file, int sampleRate, int numSamples);
void playAndDeleteFile();
void initializeOutputFile(std::ofstream& out_of, const std::string& path);
std::string getOutputFilePath();
std::string getCurrentTime();
void finalizeOutputFile(std::ofstream& out_of, const std::string& path);
void executeTestsSequentially(const std::vector<std::string>& testNames, std::shared_ptr<rclcpp::Node> node);
void executeTestsInParallel(const std::vector<std::string>& testNames, std::shared_ptr<rclcpp::Node> node);
void switchMicrophoneChannel();

void promptAndExit(int err);
void promptAndContinue();

#endif // SENSOR_TEST_H
