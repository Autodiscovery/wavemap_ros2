cmake_minimum_required(VERSION 3.14)
project(wavemap_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HashedBlock.msg"
  "msg/HashedBlocks.msg"
  "msg/HashedWaveletOctree.msg"
  "msg/HashedWaveletOctreeBlock.msg"
  "msg/Index3D.msg"
  "msg/Map.msg"
  "msg/OctreeNode.msg"
  "msg/WaveletOctree.msg"
  "msg/WaveletOctreeNode.msg"
  "srv/FilePath.srv"
  DEPENDENCIES std_msgs
)

ament_package()
