#ifndef WAVEMAP_ROS2_MAP_OPERATIONS_CROP_MAP_OPERATION_H_
#define WAVEMAP_ROS2_MAP_OPERATIONS_CROP_MAP_OPERATION_H_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

#include "wavemap_ros2/utils/tf_transformer.h"

namespace wavemap {
struct CropMapOperationConfig
    : public ConfigBase<CropMapOperationConfig, 4> {
  FloatingPoint radius = 0.f;
  Seconds<FloatingPoint> once_every = 10.f;
  std::string body_frame = "body";
  std::string sensor_frame = "";

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class CropMapOperation : public MapOperationBase {
 public:
  CropMapOperation(const CropMapOperationConfig& config,
                   MapBase::Ptr occupancy_map,
                   std::shared_ptr<TfTransformer> transformer,
                   std::string world_frame);

  void run(bool force_run = false) override;

 private:
  const CropMapOperationConfig config_;
  MapBase::Ptr occupancy_map_;
  std::shared_ptr<TfTransformer> transformer_;
  const std::string world_frame_;

  rclcpp::Time last_run_timestamp_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_MAP_OPERATIONS_CROP_MAP_OPERATION_H_
