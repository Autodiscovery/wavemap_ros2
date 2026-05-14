#include <memory>
#include <string>

#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <wavemap/io/file_conversions.h>

#include "wavemap_ros2/ros_server.h"
#include "wavemap_ros2/utils/rosbag_processor.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Create the wavemap server node
  auto node = std::make_shared<wavemap::RosServer>(rclcpp::NodeOptions());

  // Get parameters
  std::string bag_path;
  std::string output_path;
  if (!node->has_parameter("rosbag_path")) {
    node->declare_parameter<std::string>("rosbag_path", "");
  }
  bag_path = node->get_parameter("rosbag_path").as_string();
  if (!node->has_parameter("output_path")) {
    node->declare_parameter<std::string>("output_path", "");
  }
  output_path = node->get_parameter("output_path").as_string();

  if (bag_path.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "No rosbag_path parameter specified. Exiting.");
    return 1;
  }

  // Create the rosbag processor
  wavemap::RosbagProcessor rosbag_processor(node);

  // Register callbacks for each input topic
  for (const auto& input : node->getInputs()) {
    const auto topic_name = input->getTopicName();
    RCLCPP_INFO(node->get_logger(), "Registering callback for topic '%s'.",
                topic_name.c_str());

    // Subscribe to pointcloud or image topics
    rosbag_processor.addCallback<sensor_msgs::msg::PointCloud2>(
        topic_name,
        [&node, topic_name](const sensor_msgs::msg::PointCloud2& msg) {
          RCLCPP_DEBUG(node->get_logger(), "Processing PointCloud2 from '%s'",
                       topic_name.c_str());
          // Forward to the input's callback
        });
  }

  // Register TF callback
  rosbag_processor.addCallback<tf2_msgs::msg::TFMessage>(
      "/tf", [&node](const tf2_msgs::msg::TFMessage& /*msg*/) {
        // TF messages are automatically handled by the TF buffer/listener
      });
  rosbag_processor.addCallback<tf2_msgs::msg::TFMessage>(
      "/tf_static", [&node](const tf2_msgs::msg::TFMessage& /*msg*/) {
        // Static TF messages are automatically handled
      });

  // Process the bag
  RCLCPP_INFO(node->get_logger(), "Processing rosbag '%s'...",
              bag_path.c_str());
  rosbag_processor.processFullBag(bag_path);

  // Save the map if output path specified
  if (!output_path.empty()) {
    auto map = node->getMap();
    if (map) {
      map->threshold();
      if (wavemap::io::mapToFile(*map, output_path)) {
        RCLCPP_INFO(node->get_logger(), "Map saved to '%s'.",
                    output_path.c_str());
      } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to save map to '%s'.",
                     output_path.c_str());
      }
    }
  }

  RCLCPP_INFO(node->get_logger(), "Rosbag processing complete.");
  rclcpp::shutdown();
  return 0;
}
