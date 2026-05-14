#ifndef WAVEMAP_ROS2_INPUTS_DEPTH_IMAGE_TOPIC_INPUT_H_
#define WAVEMAP_ROS2_INPUTS_DEPTH_IMAGE_TOPIC_INPUT_H_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <wavemap/core/config/string_list.h>
#include <wavemap/core/utils/time/stopwatch.h>

#include "wavemap_ros2/inputs/ros_input_base.h"

namespace wavemap {
struct DepthImageTopicInputConfig
    : public ConfigBase<DepthImageTopicInputConfig, 8, StringList> {
  std::string topic_name;
  int topic_queue_length = 10;

  StringList measurement_integrator_names;

  Seconds<FloatingPoint> processing_retry_period = 0.05f;
  Seconds<FloatingPoint> max_wait_for_pose = 1.f;

  std::string sensor_frame_id;
  Seconds<FloatingPoint> time_offset = 0.f;
  int depth_scale_factor = 1;

  static MemberMap memberMap;

  operator RosInputBaseConfig() const {  // NOLINT
    return {topic_name, topic_queue_length, measurement_integrator_names,
            processing_retry_period};
  }

  bool isValid(bool verbose) const override;
};

class DepthImageTopicInput : public RosInputBase {
 public:
  DepthImageTopicInput(const DepthImageTopicInputConfig& config,
                       std::shared_ptr<Pipeline> pipeline,
                       std::shared_ptr<TfTransformer> transformer,
                       std::string world_frame,
                       rclcpp::Node::SharedPtr node);

 private:
  const DepthImageTopicInputConfig config_;

  Stopwatch integration_timer_;

  image_transport::Subscriber depth_image_sub_;

  struct StampedImage {
    rclcpp::Time stamp;
    Image<> image;
    std::string frame_id;
  };
  std::queue<StampedImage> depth_image_queue_;

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_INPUTS_DEPTH_IMAGE_TOPIC_INPUT_H_
