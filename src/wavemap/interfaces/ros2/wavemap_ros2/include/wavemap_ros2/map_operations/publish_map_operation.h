#ifndef WAVEMAP_ROS2_MAP_OPERATIONS_PUBLISH_MAP_OPERATION_H_
#define WAVEMAP_ROS2_MAP_OPERATIONS_PUBLISH_MAP_OPERATION_H_

#include <memory>
#include <string>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/thread_pool.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>
#include <wavemap_msgs/msg/map.hpp>

namespace wavemap {
struct PublishMapOperationConfig
    : public ConfigBase<PublishMapOperationConfig, 3> {
  std::string topic = "map";
  Seconds<FloatingPoint> once_every = 2.f;
  int max_num_blocks_per_msg = 1000;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class PublishMapOperation : public MapOperationBase {
 public:
  PublishMapOperation(const PublishMapOperationConfig& config,
                      MapBase::Ptr occupancy_map,
                      std::shared_ptr<ThreadPool> thread_pool,
                      std::string world_frame,
                      rclcpp::Node::SharedPtr node);

  bool shouldRun(const rclcpp::Time& current_time);
  void run(bool force_publish = false) override;

 private:
  const PublishMapOperationConfig config_;
  MapBase::Ptr occupancy_map_;
  std::shared_ptr<ThreadPool> thread_pool_;
  const std::string world_frame_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<wavemap_msgs::msg::Map>::SharedPtr map_pub_;

  rclcpp::Time last_run_timestamp_;
  std::unordered_set<Index3D, Index3DHash> previously_published_blocks_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_MAP_OPERATIONS_PUBLISH_MAP_OPERATION_H_
