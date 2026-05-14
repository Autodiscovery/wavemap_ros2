#ifndef WAVEMAP_ROS2_INPUTS_ROS_INPUT_BASE_H_
#define WAVEMAP_ROS2_INPUTS_ROS_INPUT_BASE_H_

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/config/string_list.h>
#include <wavemap/pipeline/pipeline.h>

#include "wavemap_ros2/utils/tf_transformer.h"

namespace wavemap {
struct RosInputType : public TypeSelector<RosInputType> {
  using TypeSelector<RosInputType>::TypeSelector;

  enum Id : TypeId { kPointcloudTopic, kDepthImageTopic };

  static constexpr std::array names = {"pointcloud", "depth_image"};
};

struct RosInputBaseConfig
    : public ConfigBase<RosInputBaseConfig, 4, StringList> {
  std::string topic_name;
  int topic_queue_length = 10;
  StringList measurement_integrator_names;
  Seconds<FloatingPoint> processing_retry_period = 0.1f;

  RosInputBaseConfig() = default;
  RosInputBaseConfig(std::string topic_name, int topic_queue_length,
                     StringList measurement_integrator_names,
                     Seconds<FloatingPoint> processing_retry_period)
      : topic_name(std::move(topic_name)),
        topic_queue_length(topic_queue_length),
        measurement_integrator_names(std::move(measurement_integrator_names)),
        processing_retry_period(processing_retry_period) {}

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class RosInputBase {
 public:
  RosInputBase(const RosInputBaseConfig& config,
               std::shared_ptr<Pipeline> pipeline,
               std::shared_ptr<TfTransformer> transformer,
               std::string world_frame,
               rclcpp::Node::SharedPtr node);
  virtual ~RosInputBase() = default;

  RosInputType getType() const { return type_; }
  std::string getTopicName() const { return config_.topic_name; }

 protected:
  const RosInputBaseConfig config_;
  std::shared_ptr<Pipeline> pipeline_;
  std::shared_ptr<TfTransformer> transformer_;
  const std::string world_frame_;
  rclcpp::Node::SharedPtr node_;

  void processQueue();

  RosInputType type_ = RosInputType::kPointcloudTopic;
  rclcpp::TimerBase::SharedPtr queue_processing_retry_timer_;

  // Measurement queue
  struct StampedMeasurement {
    rclcpp::Time stamp;
    Pointcloud<> pointcloud;
    Transformation3D T_W_C;
    std::vector<std::string> integrator_names;
  };
  std::queue<StampedMeasurement> measurement_queue_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_INPUTS_ROS_INPUT_BASE_H_
