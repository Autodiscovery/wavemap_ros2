#include "wavemap_ros2/map_operations/crop_map_operation.h"

#include <string>
#include <utility>

#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(CropMapOperationConfig,
                      (radius)
                      (once_every)
                      (body_frame)
                      (sensor_frame));

bool CropMapOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_GT(radius, 0.f, verbose);
  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &= IS_PARAM_NE(body_frame, "", verbose);

  return all_valid;
}

CropMapOperation::CropMapOperation(const CropMapOperationConfig& config,
                                   MapBase::Ptr occupancy_map,
                                   std::shared_ptr<TfTransformer> transformer,
                                   std::string world_frame)
    : MapOperationBase(occupancy_map),
      config_(config.checkValid()),
      occupancy_map_(std::move(occupancy_map)),
      transformer_(std::move(transformer)),
      world_frame_(std::move(world_frame)),
      last_run_timestamp_(0, 0, RCL_ROS_TIME) {}

void CropMapOperation::run(bool force_run) {
  const auto current_time = rclcpp::Clock(RCL_ROS_TIME).now();
  if (!force_run) {
    const auto time_since_last_run = current_time - last_run_timestamp_;
    if (time_since_last_run.seconds() < config_.once_every) {
      return;
    }
  }
  last_run_timestamp_ = current_time;

  // Get the current robot pose
  const std::string frame_id =
      config_.sensor_frame.empty() ? config_.body_frame : config_.sensor_frame;
  const auto T_W_B = transformer_->lookupLatestTransform(
      world_frame_, frame_id);
  if (!T_W_B) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("wavemap"),
        "Could not look up center point for map cropping. "
        << "TF lookup of frame \"" << frame_id << "\" w.r.t. \""
        << world_frame_ << "\" failed.");
    return;
  }

  // Crop the map
  const Point3D t_W_B = T_W_B->getPosition();
  auto crop_fn = [&t_W_B, this](const Index3D& block_index,
                                 const auto& /*block*/) {
    const auto block_center = convert::indexToCenterPoint(
        block_index,
        occupancy_map_->getMinCellWidth() *
            std::exp2(occupancy_map_->getTreeHeight()));
    const auto distance = (block_center - t_W_B).norm();
    return config_.radius < distance;
  };

  if (auto* hashed_wavelet =
          dynamic_cast<HashedWaveletOctree*>(occupancy_map_.get())) {
    hashed_wavelet->eraseBlockIf(crop_fn);
  } else if (auto* hashed_chunked =
                 dynamic_cast<HashedChunkedWaveletOctree*>(
                     occupancy_map_.get())) {
    hashed_chunked->eraseBlockIf(crop_fn);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("wavemap"),
                "Crop operation not supported for this map type.");
  }
}
}  // namespace wavemap
