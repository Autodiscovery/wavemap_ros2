#ifndef WAVEMAP_ROS2_UTILS_TF_TRANSFORMER_H_
#define WAVEMAP_ROS2_UTILS_TF_TRANSFORMER_H_

#include <map>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <wavemap/core/common.h>

namespace wavemap {
class TfTransformer {
 public:
  explicit TfTransformer(rclcpp::Node::SharedPtr node,
                         FloatingPoint tf_buffer_cache_time = 10.f)
      : node_(node),
        tf_buffer_(node->get_clock()),
        tf_listener_(tf_buffer_, node) {}

  // Check whether a transform is available
  bool isTransformAvailable(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const rclcpp::Time& frame_timestamp) const;

  // Waits for a transform to become available
  bool waitForTransform(const std::string& to_frame_id,
                        const std::string& from_frame_id,
                        const rclcpp::Time& frame_timestamp);

  // Lookup transforms
  std::optional<Transformation3D> lookupTransform(
      const std::string& to_frame_id, const std::string& from_frame_id,
      const rclcpp::Time& frame_timestamp);
  std::optional<Transformation3D> lookupLatestTransform(
      const std::string& to_frame_id, const std::string& from_frame_id);

  // Strip leading slashes if needed to avoid TF errors
  static std::string sanitizeFrameId(const std::string& string);

 private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Transform lookup timers
  static constexpr std::chrono::duration<double> transform_lookup_retry_period_{
      0.02};
  static constexpr std::chrono::duration<double> transform_lookup_max_time_{
      0.25};

  bool waitForTransformImpl(const std::string& to_frame_id,
                            const std::string& from_frame_id,
                            const rclcpp::Time& frame_timestamp) const;
  std::optional<Transformation3D> lookupTransformImpl(
      const std::string& to_frame_id, const std::string& from_frame_id,
      const rclcpp::Time& frame_timestamp);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_UTILS_TF_TRANSFORMER_H_
