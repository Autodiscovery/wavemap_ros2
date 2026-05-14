#ifndef WAVEMAP_ROS2_MAP_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_
#define WAVEMAP_ROS2_MAP_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

namespace wavemap {
struct PublishPointcloudOperationConfig
    : public ConfigBase<PublishPointcloudOperationConfig, 3> {
  std::string topic = "obstacle_pointcloud";
  Seconds<FloatingPoint> once_every = 1.f;
  FloatingPoint occupancy_threshold_log_odds = 1e-3f;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class PublishPointcloudOperation : public MapOperationBase {
 public:
  PublishPointcloudOperation(const PublishPointcloudOperationConfig& config,
                             MapBase::Ptr occupancy_map,
                             std::string world_frame,
                             rclcpp::Node::SharedPtr node);

  void run(bool force_publish = false) override;

 private:
  const PublishPointcloudOperationConfig config_;
  MapBase::Ptr occupancy_map_;
  const std::string world_frame_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  rclcpp::Time last_run_timestamp_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_MAP_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_
