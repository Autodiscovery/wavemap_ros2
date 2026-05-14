#include "wavemap_ros2/inputs/ros_input_base.h"

#include <memory>
#include <string>
#include <utility>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(RosInputBaseConfig,
                      (topic_name)
                      (topic_queue_length)
                      (measurement_integrator_names)
                      (processing_retry_period));

bool RosInputBaseConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, "", verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_FALSE(measurement_integrator_names.empty(), verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);

  return all_valid;
}

RosInputBase::RosInputBase(const RosInputBaseConfig& config,
                           std::shared_ptr<Pipeline> pipeline,
                           std::shared_ptr<TfTransformer> transformer,
                           std::string world_frame,
                           rclcpp::Node::SharedPtr node)
    : config_(config.checkValid()),
      pipeline_(std::move(pipeline)),
      transformer_(std::move(transformer)),
      world_frame_(std::move(world_frame)),
      node_(node) {
  // Start the queue processing retry timer
  queue_processing_retry_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(config_.processing_retry_period),
      [this]() { processQueue(); });
}

void RosInputBase::processQueue() {
  // In ROS2, input callbacks are handled directly by the subscription
  // This method is kept for future queue-based processing if needed
}
}  // namespace wavemap
