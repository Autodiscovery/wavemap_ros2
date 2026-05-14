#include "wavemap_ros2/map_operations/publish_map_operation.h"

#include <memory>
#include <string>
#include <utility>

#include <wavemap/core/utils/profile/profiler_interface.h>
#include <wavemap_msgs/msg/map.hpp>
#include <wavemap_ros2_conversions/map_msg_conversions.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PublishMapOperationConfig,
                      (once_every)
                      (max_num_blocks_per_msg)
                      (topic));

bool PublishMapOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &= IS_PARAM_GT(max_num_blocks_per_msg, 0, verbose);
  all_valid &= IS_PARAM_NE(topic, "", verbose);

  return all_valid;
}

PublishMapOperation::PublishMapOperation(
    const PublishMapOperationConfig& config, MapBase::Ptr occupancy_map,
    std::shared_ptr<ThreadPool> thread_pool, std::string world_frame,
    rclcpp::Node::SharedPtr node)
    : MapOperationBase(occupancy_map),
      config_(config.checkValid()),
      occupancy_map_(std::move(occupancy_map)),
      thread_pool_(std::move(thread_pool)),
      world_frame_(std::move(world_frame)),
      node_(node),
      last_run_timestamp_(0, 0, RCL_ROS_TIME) {
  map_pub_ = node_->create_publisher<wavemap_msgs::msg::Map>(
      config_.topic, 10);
}

bool PublishMapOperation::shouldRun(const rclcpp::Time& current_time) {
  return config_.once_every < (current_time - last_run_timestamp_).seconds();
}

void PublishMapOperation::run(bool force_run) {
  const auto current_time = node_->now();
  if (force_run || shouldRun(current_time)) {
    ProfilerZoneScoped;
    // If the map is empty, there's no work to do
    if (occupancy_map_->empty()) {
      return;
    }

    occupancy_map_->threshold();
    wavemap_msgs::msg::Map map_msg;
    if (convert::mapToRosMsg(*occupancy_map_, world_frame_, current_time,
                             map_msg)) {
      map_pub_->publish(map_msg);
    }

    last_run_timestamp_ = current_time;
  }
}
}  // namespace wavemap
