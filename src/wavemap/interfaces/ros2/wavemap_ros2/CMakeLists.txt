cmake_minimum_required(VERSION 3.14)
project(wavemap_ros2)

# Resolve real path to handle symlinked package directories
get_filename_component(WAVEMAP_REAL_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}" REALPATH)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(wavemap REQUIRED)
find_package(wavemap_msgs REQUIRED)
find_package(wavemap_ros2_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)

# Check for optional Livox ROS2 driver
find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver2_FOUND)
  message(STATUS "Building with Livox ROS2 driver support")
  add_definitions(-DLIVOX_AVAILABLE)
else()
  message(STATUS "Livox ROS2 driver not found, building without Livox support")
endif()

# Library sources
set(WAVEMAP_ROS2_SOURCES
    src/ros_server.cc
    src/utils/ros_logging_level.cc
    src/utils/tf_transformer.cc
    src/inputs/ros_input_base.cc
    src/inputs/ros_input_factory.cc
    src/inputs/depth_image_topic_input.cc
    src/inputs/pointcloud_topic_input.cc
    src/map_operations/crop_map_operation.cc
    src/map_operations/publish_map_operation.cc
    src/map_operations/publish_pointcloud_operation.cc
    src/map_operations/map_ros_operation_factory.cc
    src/utils/rosbag_processor.cc)

add_library(${PROJECT_NAME}_lib ${WAVEMAP_ROS2_SOURCES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_lib PUBLIC
    rclcpp cv_bridge image_transport tf2_ros tf2_eigen
    std_srvs sensor_msgs visualization_msgs
    rosbag2_cpp rosbag2_storage tf2_msgs rosgraph_msgs
    wavemap wavemap_msgs wavemap_ros2_conversions)
if(livox_ros_driver2_FOUND)
  ament_target_dependencies(${PROJECT_NAME}_lib PUBLIC livox_ros_driver2)
endif()

# Executables
add_executable(ros_server app/ros_server.cc)
target_link_libraries(ros_server ${PROJECT_NAME}_lib)
ament_target_dependencies(ros_server rclcpp)

add_executable(rosbag_processor app/rosbag_processor.cc)
target_link_libraries(rosbag_processor ${PROJECT_NAME}_lib)
ament_target_dependencies(rosbag_processor rclcpp)

# Install
install(TARGETS ${PROJECT_NAME}_lib
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(TARGETS ros_server rosbag_processor
    DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
    DESTINATION include)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config)

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp tf2_ros wavemap wavemap_msgs wavemap_ros2_conversions)
ament_export_include_directories(include)

ament_package()
