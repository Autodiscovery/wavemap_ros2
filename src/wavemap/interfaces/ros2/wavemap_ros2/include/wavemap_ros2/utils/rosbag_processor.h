#ifndef WAVEMAP_ROS2_UTILS_ROSBAG_PROCESSOR_H_
#define WAVEMAP_ROS2_UTILS_ROSBAG_PROCESSOR_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rclcpp/serialization.hpp>

namespace wavemap {
class RosbagProcessor {
 public:
  explicit RosbagProcessor(rclcpp::Node::SharedPtr node)
      : node_(node) {}

  template <typename MessageT>
  void addCallback(const std::string& topic_name,
                   std::function<void(const MessageT&)> callback);

  bool processFullBag(const std::string& bag_path);

 private:
  rclcpp::Node::SharedPtr node_;

  struct TopicCallback {
    std::string topic_name;
    std::string type_name;
    std::function<void(const std::shared_ptr<rclcpp::SerializedMessage>&)>
        deserialize_and_call;
  };
  std::vector<TopicCallback> callbacks_;
};

template <typename MessageT>
void RosbagProcessor::addCallback(
    const std::string& topic_name,
    std::function<void(const MessageT&)> callback) {
  TopicCallback tc;
  tc.topic_name = topic_name;
  tc.type_name = rosidl_generator_traits::name<MessageT>();
  tc.deserialize_and_call =
      [callback](
          const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg) {
        MessageT msg;
        rclcpp::Serialization<MessageT> serializer;
        serializer.deserialize_message(serialized_msg.get(), &msg);
        callback(msg);
      };
  callbacks_.push_back(std::move(tc));
}
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_UTILS_ROSBAG_PROCESSOR_H_
