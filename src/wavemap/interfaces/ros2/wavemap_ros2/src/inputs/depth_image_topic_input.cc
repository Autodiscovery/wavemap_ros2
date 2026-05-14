#include "wavemap_ros2/inputs/depth_image_topic_input.h"

#include <memory>
#include <string>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "wavemap_ros2_conversions/time_conversions.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(DepthImageTopicInputConfig,
                      (topic_name)
                      (topic_queue_length)
                      (measurement_integrator_names)
                      (processing_retry_period)
                      (max_wait_for_pose)
                      (sensor_frame_id)
                      (time_offset)
                      (depth_scale_factor));

bool DepthImageTopicInputConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, "", verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_FALSE(measurement_integrator_names.empty(), verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GT(max_wait_for_pose, 0.f, verbose);
  all_valid &= IS_PARAM_GT(depth_scale_factor, 0, verbose);

  return all_valid;
}

DepthImageTopicInput::DepthImageTopicInput(
    const DepthImageTopicInputConfig& config,
    std::shared_ptr<Pipeline> pipeline,
    std::shared_ptr<TfTransformer> transformer, std::string world_frame,
    rclcpp::Node::SharedPtr node)
    : RosInputBase(config, std::move(pipeline), std::move(transformer),
                   std::move(world_frame), node),
      config_(config.checkValid()) {
  type_ = RosInputType::kDepthImageTopic;

  // Subscribe to depth image topic
  image_transport::ImageTransport it(node_);
  depth_image_sub_ = it.subscribe(
      config_.topic_name, config_.topic_queue_length,
      &DepthImageTopicInput::callback, this);
}

void DepthImageTopicInput::callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_image_msg) {
  // Convert depth image to wavemap Image
  const std::string frame_id = config_.sensor_frame_id.empty()
                                    ? depth_image_msg->header.frame_id
                                    : config_.sensor_frame_id;
  const auto stamp = rclcpp::Time(depth_image_msg->header.stamp) +
                     rclcpp::Duration::from_seconds(config_.time_offset);

  // Wait for pose
  if (!transformer_->waitForTransform(world_frame_, frame_id, stamp)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("wavemap"),
        "Waited " << config_.max_wait_for_pose
            << "s, but still could not get the pose for depth image at time "
            << stamp.seconds() << ". Dropping it.");
    return;
  }

  // Get the pose
  const auto T_W_C = transformer_->lookupTransform(world_frame_, frame_id,
                                                     stamp);
  if (!T_W_C) {
    return;
  }

  // Convert to cv::Mat
  cv_bridge::CvImageConstPtr cv_image;
  try {
    cv_image = cv_bridge::toCvShare(depth_image_msg);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("wavemap"),
                 "cv_bridge exception: %s", e.what());
    return;
  }

  // Convert to floating point
  Image<> image(cv_image->image.rows, cv_image->image.cols);
  const auto& cv_mat = cv_image->image;
  if (cv_mat.type() == CV_32FC1) {
    for (int row = 0; row < cv_mat.rows; ++row) {
      for (int col = 0; col < cv_mat.cols; ++col) {
        image.at(Index2D(row, col)) = cv_mat.at<float>(row, col);
      }
    }
  } else if (cv_mat.type() == CV_16UC1) {
    const FloatingPoint scale = 1.f / static_cast<FloatingPoint>(
                                          config_.depth_scale_factor);
    for (int row = 0; row < cv_mat.rows; ++row) {
      for (int col = 0; col < cv_mat.cols; ++col) {
        image.at(Index2D(row, col)) = static_cast<FloatingPoint>(
                              cv_mat.at<uint16_t>(row, col)) *
                          scale;
      }
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("wavemap"),
                 "Unsupported depth image type: %d", cv_mat.type());
    return;
  }

  // Create posed image and integrate
  integration_timer_.start();
  PosedImage<> posed_image(T_W_C.value(), image.getNumRows(),
                           image.getNumColumns());
  posed_image.getData() = image.getData();
  pipeline_->runPipeline(
      std::vector<std::string>(config_.measurement_integrator_names.value.begin(),
                               config_.measurement_integrator_names.value.end()),
      posed_image);
  integration_timer_.stop();

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("wavemap"),
      "Inserting depth image with "
          << posed_image.size() << " pixels took "
          << integration_timer_.getLastEpisodeDuration() << "s.");
}
}  // namespace wavemap
