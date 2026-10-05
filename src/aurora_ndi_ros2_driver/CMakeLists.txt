cmake_minimum_required(VERSION 3.8)
project(aurora_ndi_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate custom message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuroraData.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Include directories
include_directories(include)

# Create modular core libraries
add_library(aurora_serial
  src/core/aurora_serial_interface.cpp
)
target_include_directories(aurora_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(aurora_parser
  src/core/aurora_message_parser.cpp
)
target_include_directories(aurora_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(aurora_protocol
  src/core/aurora_protocol.cpp
)
target_include_directories(aurora_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aurora_protocol
  aurora_serial
  aurora_parser
)

# Create aurora_driver library (facade)
add_library(aurora_driver
  src/core/ndi_aurora_ros2.cpp
)
target_include_directories(aurora_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Link modular components to aurora_driver
target_link_libraries(aurora_driver
  aurora_serial
  aurora_parser
  aurora_protocol
  pthread
)

# Create aurora_utils library
add_library(aurora_utils
  src/utils/aurora_utils.cpp
)
target_include_directories(aurora_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Create aurora_data_filter library
add_library(aurora_data_filter
  src/filters/aurora_data_filter.cpp
)
target_include_directories(aurora_data_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aurora_data_filter
  aurora_utils
)

# Create the main executable
add_executable(aurora_publisher_node
  src/nodes/driver/aurora_publisher_node.cpp
)

# Link libraries to the executable
target_link_libraries(aurora_publisher_node
  aurora_driver
  aurora_data_filter
  aurora_utils
)

# Add ament dependencies to the executable
ament_target_dependencies(aurora_publisher_node
  rclcpp
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Link custom message interfaces to the executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(aurora_publisher_node "${cpp_typesupport_target}")

# Create noise estimator executable
add_executable(noise_estimator_node
  src/nodes/analysis/noise_estimator_node.cpp
)

# Add ament dependencies to noise estimator
ament_target_dependencies(noise_estimator_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Link custom message interfaces to noise estimator
target_link_libraries(noise_estimator_node "${cpp_typesupport_target}")

# Create kalman filter executable
add_executable(kalman_filter_node
  src/filters/kalman_filter_node.cpp
)

# Add ament dependencies to kalman filter
ament_target_dependencies(kalman_filter_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Link Eigen3 and custom message interfaces to kalman filter
target_link_libraries(kalman_filter_node
  "${cpp_typesupport_target}"
  Eigen3::Eigen
)

# Create lowpass filter executable
add_executable(lowpass_filter_node
  src/filters/lowpass_filter_node.cpp
)

# Add ament dependencies to lowpass filter
ament_target_dependencies(lowpass_filter_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Link Eigen3 and custom message interfaces to lowpass filter
target_link_libraries(lowpass_filter_node
  "${cpp_typesupport_target}"
  Eigen3::Eigen
)

# Create delay analyzer executable
add_executable(delay_analyzer_node
  src/nodes/analysis/delay_analyzer_node.cpp
)

# Add ament dependencies to delay analyzer
ament_target_dependencies(delay_analyzer_node
  rclcpp
  geometry_msgs
  std_msgs
)

# Link custom message interfaces to delay analyzer
target_link_libraries(delay_analyzer_node
  "${cpp_typesupport_target}"
)

# Install targets
install(TARGETS
  aurora_publisher_node
  noise_estimator_node
  kalman_filter_node
  lowpass_filter_node
  delay_analyzer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install libraries (in case other packages need them)
install(TARGETS
  aurora_driver
  aurora_utils
  aurora_serial
  aurora_parser
  aurora_protocol
  aurora_data_filter
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()