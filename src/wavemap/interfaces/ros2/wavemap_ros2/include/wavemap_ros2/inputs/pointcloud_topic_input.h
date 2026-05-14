#ifndef WAVEMAP_ROS2_INPUTS_POINTCLOUD_TOPIC_INPUT_H_
#define WAVEMAP_ROS2_INPUTS_POINTCLOUD_TOPIC_INPUT_H_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <wavemap/core/config/string_list.h>
#include <wavemap/core/utils/time/stopwatch.h>

#include "wavemap_ros2/inputs/ros_input_base.h"

#ifdef LIVOX_AVAILABLE
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace wavemap {
struct PointcloudTopicType : public TypeSelector<PointcloudTopicType> {
  using TypeSelector<PointcloudTopicType>::TypeSelector;

  enum Id : TypeId { kPointCloud2, kOuster, kLivox };

  static constexpr std::array names = {"PointCloud2", "ouster", "livox"};
};

struct PointcloudTopicInputConfig
    : public ConfigBase<PointcloudTopicInputConfig, 12, PointcloudTopicType,
                        StringList> {
  std::string topic_name;
  PointcloudTopicType topic_type = PointcloudTopicType::kPointCloud2;
  int topic_queue_length = 10;

  StringList measurement_integrator_names;

  Seconds<FloatingPoint> processing_retry_period = 0.05f;
  Seconds<FloatingPoint> max_wait_for_pose = 1.f;

  std::string sensor_frame_id;
  Seconds<FloatingPoint> time_offset = 0.f;
  bool undistort_motion = false;
  int num_undistortion_interpolation_intervals_per_cloud = 100;

  std::string projected_range_image_topic_name;
  std::string undistorted_pointcloud_topic_name;

  static MemberMap memberMap;

  operator RosInputBaseConfig() const {  // NOLINT
    return {topic_name, topic_queue_length, measurement_integrator_names,
            processing_retry_period};
  }

  bool isValid(bool verbose) const override;
};

class PointcloudTopicInput : public RosInputBase {
 public:
  PointcloudTopicInput(const PointcloudTopicInputConfig& config,
                       std::shared_ptr<Pipeline> pipeline,
                       std::shared_ptr<TfTransformer> transformer,
                       std::string world_frame,
                       rclcpp::Node::SharedPtr node);

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg);
#ifdef LIVOX_AVAILABLE
  void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr pointcloud_msg);
#endif

 private:
  const PointcloudTopicInputConfig config_;

  Stopwatch integration_timer_;

  rclcpp::SubscriptionBase::SharedPtr pointcloud_sub_;

  static bool hasField(const sensor_msgs::msg::PointCloud2& msg,
                       const std::string& field_name);

  void publishProjectedRangeImageIfEnabled(
      const rclcpp::Time& stamp, const PosedPointcloud<>& posed_pointcloud);
  image_transport::Publisher projected_range_image_pub_;

  void publishUndistortedPointcloudIfEnabled(
      const rclcpp::Time& stamp,
      const PosedPointcloud<>& undistorted_pointcloud);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      undistorted_pointcloud_pub_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_INPUTS_POINTCLOUD_TOPIC_INPUT_H_
