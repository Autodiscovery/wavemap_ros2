cmake_minimum_required(VERSION 3.14)
project(wavemap)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(glog REQUIRED)

# Resolve real path to handle symlinked package directories
get_filename_component(WAVEMAP_REAL_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
set(WAVEMAP_LIBRARY_DIR "${WAVEMAP_REAL_SOURCE_DIR}/../../../library/cpp")

# Load the library
set(GENERATE_WAVEMAP_INSTALL_RULES OFF)
add_subdirectory(${WAVEMAP_LIBRARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME})

# Install targets
install(TARGETS wavemap_core wavemap_io wavemap_pipeline
    EXPORT wavemap-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

# Install headers
install(DIRECTORY
    ${WAVEMAP_LIBRARY_DIR}/include/wavemap/
    DESTINATION include/wavemap
    FILES_MATCHING PATTERN "*.h")

# Install 3rd party headers bundled with wavemap
install(DIRECTORY
    ${WAVEMAP_LIBRARY_DIR}/include/wavemap/3rd_party/
    DESTINATION include/wavemap/3rd_party
    FILES_MATCHING PATTERN "*.h")

# Export targets and dependencies
ament_export_targets(wavemap-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Boost glog)
ament_export_include_directories(include)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
