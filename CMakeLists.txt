cmake_minimum_required(VERSION 3.8)
project(laser_line_extraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineSegment.msg"
  "msg/LineSegmentList.msg"
  DEPENDENCIES std_msgs
)

# Create library for line extraction
add_library(line_extraction_lib
  src/line.cpp
  src/line_extraction.cpp
)
target_include_directories(line_extraction_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(line_extraction_lib PUBLIC cxx_std_17)

# Create library for ROS wrapper
add_library(line_extraction_ros_lib
  src/line_extraction_ros.cpp
)
target_include_directories(line_extraction_ros_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(line_extraction_ros_lib PUBLIC cxx_std_17)
target_link_libraries(line_extraction_ros_lib PUBLIC line_extraction_lib)
ament_target_dependencies(line_extraction_ros_lib PUBLIC
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(line_extraction_ros_lib PUBLIC ${cpp_typesupport_target})

# Create executable node
add_executable(line_extraction_node src/line_extraction_node.cpp)
target_link_libraries(line_extraction_node PRIVATE line_extraction_ros_lib)
ament_target_dependencies(line_extraction_node PUBLIC rclcpp)

# Install targets
install(TARGETS
  line_extraction_lib
  line_extraction_ros_lib
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS line_extraction_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(line_extraction_lib line_extraction_ros_lib)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
