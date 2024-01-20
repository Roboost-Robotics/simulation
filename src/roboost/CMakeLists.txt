cmake_minimum_required(VERSION 3.5)
project(roboost)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include Cpp "include" directory
include_directories(include)

add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher rclcpp image_transport cv_bridge sensor_msgs OpenCV)

add_executable(image_stream src/image_stream.cpp)
ament_target_dependencies(image_stream rclcpp image_transport cv_bridge sensor_msgs OpenCV)

# Install Cpp executables
install(TARGETS
  image_publisher
  image_stream
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/odom_to_base_node.py
  scripts/camera_publisher_node.py
  scripts/audio_player_node.py
  scripts/timescale_connector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch robot_descriptions config meshes audio
        DESTINATION share/${PROJECT_NAME})

# Install package.xml
install(FILES package.xml
        DESTINATION share/${PROJECT_NAME})

# Install resource index
install(FILES resource/${PROJECT_NAME}
        DESTINATION share/ament_index/resource_index/packages)

ament_package()