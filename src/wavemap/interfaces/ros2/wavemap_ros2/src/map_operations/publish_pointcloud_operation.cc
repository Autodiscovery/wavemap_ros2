#include "wavemap_ros2/map_operations/publish_pointcloud_operation.h"

#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <wavemap/core/indexing/index_conversions.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PublishPointcloudOperationConfig,
                      (topic)
                      (once_every)
                      (occupancy_threshold_log_odds));

bool PublishPointcloudOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic, "", verbose);
  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);

  return all_valid;
}

PublishPointcloudOperation::PublishPointcloudOperation(
    const PublishPointcloudOperationConfig& config, MapBase::Ptr occupancy_map,
    std::string world_frame, rclcpp::Node::SharedPtr node)
    : MapOperationBase(occupancy_map),
      config_(config.checkValid()),
      occupancy_map_(std::move(occupancy_map)),
      world_frame_(std::move(world_frame)),
      node_(node),
      last_run_timestamp_(0, 0, RCL_ROS_TIME) {
  pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          config_.topic, 10);
}

void PublishPointcloudOperation::run(bool force_publish) {
  const auto current_time = node_->now();
  if (!force_publish) {
    const auto time_since_last_run = current_time - last_run_timestamp_;
    if (time_since_last_run.seconds() < config_.once_every) {
      return;
    }
  }
  last_run_timestamp_ = current_time;

  if (!occupancy_map_ || pointcloud_pub_->get_subscription_count() == 0) {
    return;
  }

  // Collect all occupied points
  std::vector<Point3D> occupied_points;
  occupancy_map_->forEachLeaf(
      [&](const OctreeIndex& node_index, FloatingPoint log_odds) {
        if (log_odds > config_.occupancy_threshold_log_odds) {
           const auto center = convert::nodeIndexToCenterPoint(
              node_index, occupancy_map_->getMinCellWidth());
          occupied_points.emplace_back(center);
        }
      });

  // Build PointCloud2 message directly
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = current_time;
  cloud_msg.header.frame_id = world_frame_;
  cloud_msg.height = 1;
  cloud_msg.width = occupied_points.size();
  cloud_msg.is_dense = true;
  cloud_msg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(occupied_points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

  for (const auto& point : occupied_points) {
    *iter_x = point.x();
    *iter_y = point.y();
    *iter_z = point.z();
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  pointcloud_pub_->publish(cloud_msg);
}
}  // namespace wavemap
