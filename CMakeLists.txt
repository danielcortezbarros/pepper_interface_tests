cmake_minimum_required(VERSION 3.8)
project(pepper_interface_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Manually add the correct include directory
include_directories(include)
include_directories("/opt/ros/humble/include/ament_index_cpp/ament_index_cpp")
include_directories(${OpenCV_INCLUDE_DIRS})

# Add your source files (from the src directory)
add_executable(sensor_test_node src/sensorTestImplementation.cpp src/sensorTestApplication.cpp)

# Link dependencies to the executable
ament_target_dependencies(sensor_test_node rclcpp std_msgs sensor_msgs nav_msgs image_transport cv_bridge ament_index_cpp)
target_link_libraries(sensor_test_node ${OpenCV_LIBS} ament_index_cpp::ament_index_cpp)

# Install the executable
install(TARGETS sensor_test_node
  DESTINATION lib/${PROJECT_NAME})

# Install the config and data directories to the share/${PROJECT_NAME} directory
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# Install the Python-based launch file to the share directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_lint_auto_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
