#include "wavemap_ros2/utils/rosbag_processor.h"

#include <memory>
#include <string>

namespace wavemap {
bool RosbagProcessor::processFullBag(const std::string& bag_path) {
  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open bag '%s': %s",
                 bag_path.c_str(), e.what());
    return false;
  }

  while (reader.has_next() && rclcpp::ok()) {
    auto serialized_msg = reader.read_next();
    auto serialized_ros_msg =
        std::make_shared<rclcpp::SerializedMessage>(*serialized_msg->serialized_data);

    for (const auto& cb : callbacks_) {
      if (serialized_msg->topic_name == cb.topic_name) {
        cb.deserialize_and_call(serialized_ros_msg);
        break;
      }
    }

    rclcpp::spin_some(node_);
  }

  return true;
}
}  // namespace wavemap
