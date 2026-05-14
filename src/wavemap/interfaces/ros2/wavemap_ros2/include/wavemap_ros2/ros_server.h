#ifndef WAVEMAP_ROS2_ROS_SERVER_H_
#define WAVEMAP_ROS2_ROS_SERVER_H_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/pipeline/pipeline.h>
#include <wavemap_msgs/srv/file_path.hpp>

#include "wavemap_ros2/inputs/ros_input_base.h"
#include "wavemap_ros2/utils/ros_logging_level.h"
#include "wavemap_ros2/utils/tf_transformer.h"

namespace wavemap {
class RosServer : public rclcpp::Node {
 public:
  explicit RosServer(const rclcpp::NodeOptions& options);

  // Access the map
  MapBase::Ptr getMap() { return occupancy_map_; }
  MapBase::ConstPtr getMap() const { return occupancy_map_; }

  // Access the pipeline
  Pipeline& getPipeline() { return *pipeline_; }
  const Pipeline& getPipeline() const { return *pipeline_; }

  // Access the inputs
  const std::vector<std::unique_ptr<RosInputBase>>& getInputs() const {
    return inputs_;
  }

 private:
  // Config
  std::string world_frame_ = "odom";
  int num_threads_ = -1;
  RosLoggingLevel logging_level_ = RosLoggingLevel::kInfo;
  std::string config_file_;

  // Map and pipeline
  MapBase::Ptr occupancy_map_;
  std::shared_ptr<Pipeline> pipeline_;
  std::shared_ptr<TfTransformer> transformer_;

  // Inputs
  std::vector<std::unique_ptr<RosInputBase>> inputs_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_map_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr republish_whole_map_srv_;
  rclcpp::Service<wavemap_msgs::srv::FilePath>::SharedPtr save_map_srv_;
  rclcpp::Service<wavemap_msgs::srv::FilePath>::SharedPtr load_map_srv_;

  // Service callbacks
  void resetMapCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void saveMapCallback(
      const std::shared_ptr<wavemap_msgs::srv::FilePath::Request> request,
      std::shared_ptr<wavemap_msgs::srv::FilePath::Response> response);
  void loadMapCallback(
      const std::shared_ptr<wavemap_msgs::srv::FilePath::Request> request,
      std::shared_ptr<wavemap_msgs::srv::FilePath::Response> response);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_ROS_SERVER_H_
