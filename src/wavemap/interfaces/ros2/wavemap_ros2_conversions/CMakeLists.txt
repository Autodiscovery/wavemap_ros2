cmake_minimum_required(VERSION 3.14)
project(wavemap_ros2_conversions)

# Resolve real path to handle symlinked package directories
get_filename_component(WAVEMAP_REAL_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}" REALPATH)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(wavemap REQUIRED)
find_package(wavemap_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Libraries
add_library(${PROJECT_NAME}
    src/config_conversions.cc
    src/map_msg_conversions.cc
    src/time_conversions.cc)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp tf2_eigen geometry_msgs wavemap wavemap_msgs)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
      test/src/test_map_msg_conversions.cc)
  target_include_directories(test_${PROJECT_NAME} PRIVATE
      ${WAVEMAP_REAL_SOURCE_DIR}/../../../library/cpp/test/include)
  target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME})
  ament_target_dependencies(test_${PROJECT_NAME}
      rclcpp wavemap wavemap_msgs)
endif()

# Install
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/
    DESTINATION include)

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp tf2_eigen geometry_msgs wavemap wavemap_msgs)
ament_export_include_directories(include)

ament_package()
