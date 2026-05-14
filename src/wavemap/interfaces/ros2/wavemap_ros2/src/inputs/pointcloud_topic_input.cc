#include "wavemap_ros2/inputs/pointcloud_topic_input.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "wavemap_ros2_conversions/time_conversions.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PointcloudTopicInputConfig,
                      (topic_name)
                      (topic_type)
                      (topic_queue_length)
                      (measurement_integrator_names)
                      (processing_retry_period)
                      (max_wait_for_pose)
                      (sensor_frame_id)
                      (time_offset)
                      (undistort_motion)
                      (num_undistortion_interpolation_intervals_per_cloud)
                      (projected_range_image_topic_name)
                      (undistorted_pointcloud_topic_name));

bool PointcloudTopicInputConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, "", verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_FALSE(measurement_integrator_names.empty(), verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GT(max_wait_for_pose, 0.f, verbose);

  return all_valid;
}

PointcloudTopicInput::PointcloudTopicInput(
    const PointcloudTopicInputConfig& config,
    std::shared_ptr<Pipeline> pipeline,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    rclcpp::Node::SharedPtr node)
    : RosInputBase(config, std::move(pipeline), std::move(transformer),
                   std::move(world_frame), node),
      config_(config.checkValid()) {
  type_ = RosInputType::kPointcloudTopic;

  // Create the subscriber based on topic type
  switch (config_.topic_type) {
    case PointcloudTopicType::kPointCloud2:
    case PointcloudTopicType::kOuster: {
      auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
          config_.topic_name,
          rclcpp::QoS(config_.topic_queue_length).best_effort(),
          [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            callback(msg);
          });
      pointcloud_sub_ = sub;
      break;
    }
#ifdef LIVOX_AVAILABLE
    case PointcloudTopicType::kLivox: {
      auto sub =
          node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
              config_.topic_name,
              rclcpp::QoS(config_.topic_queue_length).best_effort(),
              [this](
                  const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
                callback(msg);
              });
      pointcloud_sub_ = sub;
      break;
    }
#endif
    default:
      RCLCPP_ERROR(rclcpp::get_logger("wavemap"),
                   "Unknown pointcloud topic type.");
      break;
  }

  // Setup optional debug publishers
  if (!config_.undistorted_pointcloud_topic_name.empty()) {
    undistorted_pointcloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.undistorted_pointcloud_topic_name, 10);
  }
  if (!config_.projected_range_image_topic_name.empty()) {
    image_transport::ImageTransport it(node_);
    projected_range_image_pub_ =
        it.advertise(config_.projected_range_image_topic_name, 1);
  }
}

void PointcloudTopicInput::callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg) {
  const std::string frame_id = config_.sensor_frame_id.empty()
                                    ? pointcloud_msg->header.frame_id
                                    : config_.sensor_frame_id;
  const auto stamp = rclcpp::Time(pointcloud_msg->header.stamp) +
                     rclcpp::Duration::from_seconds(config_.time_offset);

  // Extract points from PointCloud2
  Pointcloud<> pointcloud;
  int num_points = pointcloud_msg->width * pointcloud_msg->height;
  if (num_points == 0) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("wavemap"),
        "Skipping empty pointcloud with timestamp " << stamp.seconds());
    return;
  }
  pointcloud.resize(num_points);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pointcloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pointcloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pointcloud_msg, "z");

  int valid_idx = 0;
  for (int i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      pointcloud[valid_idx] = Point3D(x, y, z);
      ++valid_idx;
    }
  }
  pointcloud.resize(valid_idx);

  // Wait for the pose
  if (!transformer_->waitForTransform(world_frame_, frame_id, stamp)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("wavemap"),
        "Waited " << config_.max_wait_for_pose
            << "s, but still could not get the pose for pointcloud at time "
            << stamp.seconds() << ". Dropping it.");
    return;
  }

  // Get the pose
  const auto T_W_C = transformer_->lookupTransform(world_frame_, frame_id,
                                                     stamp);
  if (!T_W_C) {
    return;
  }

  // Create posed pointcloud and integrate
  integration_timer_.start();
  PosedPointcloud<> posed_pointcloud(T_W_C.value(), std::move(pointcloud.data()));
  pipeline_->runPipeline(
      std::vector<std::string>(config_.measurement_integrator_names.value.begin(),
                               config_.measurement_integrator_names.value.end()),
      posed_pointcloud);
  integration_timer_.stop();

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("wavemap"),
      "Inserting pointcloud with "
          << posed_pointcloud.size() << " points took "
          << integration_timer_.getLastEpisodeDuration() << "s.");
}

#ifdef LIVOX_AVAILABLE
void PointcloudTopicInput::callback(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr pointcloud_msg) {
  const std::string frame_id = config_.sensor_frame_id.empty()
                                    ? pointcloud_msg->header.frame_id
                                    : config_.sensor_frame_id;
  const auto stamp = rclcpp::Time(pointcloud_msg->header.stamp) +
                     rclcpp::Duration::from_seconds(config_.time_offset);

  // Extract points from Livox CustomMsg
  Pointcloud<> pointcloud;
  const auto& points = pointcloud_msg->points;
  if (points.empty()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("wavemap"),
        "Skipping empty Livox pointcloud with timestamp " << stamp.seconds());
    return;
  }
  pointcloud.resize(points.size());

  int valid_idx = 0;
  for (const auto& point : points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      pointcloud[valid_idx] = Point3D(point.x, point.y, point.z);
      ++valid_idx;
    }
  }
  pointcloud.resize(valid_idx);

  // Wait for the pose
  if (!transformer_->waitForTransform(world_frame_, frame_id, stamp)) {
    return;
  }

  const auto T_W_C = transformer_->lookupTransform(world_frame_, frame_id,
                                                     stamp);
  if (!T_W_C) {
    return;
  }

  // Create posed pointcloud and integrate
  integration_timer_.start();
  PosedPointcloud<> posed_pointcloud(T_W_C.value(), std::move(pointcloud.data()));
  pipeline_->runPipeline(
      std::vector<std::string>(config_.measurement_integrator_names.value.begin(),
                               config_.measurement_integrator_names.value.end()),
      posed_pointcloud);
  integration_timer_.stop();
}
#endif

bool PointcloudTopicInput::hasField(
    const sensor_msgs::msg::PointCloud2& msg, const std::string& field_name) {
  for (const auto& field : msg.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

void PointcloudTopicInput::publishProjectedRangeImageIfEnabled(
    const rclcpp::Time& /*stamp*/,
    const PosedPointcloud<>& /*posed_pointcloud*/) {
  // TODO: Implement if needed
}

void PointcloudTopicInput::publishUndistortedPointcloudIfEnabled(
    const rclcpp::Time& /*stamp*/,
    const PosedPointcloud<>& /*undistorted_pointcloud*/) {
  // TODO: Implement if needed
}
}  // namespace wavemap
