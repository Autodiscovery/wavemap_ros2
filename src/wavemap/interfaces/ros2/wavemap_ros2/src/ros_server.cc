#include "wavemap_ros2/ros_server.h"

#include <memory>
#include <string>
#include <utility>

#include <glog/logging.h>
#include <wavemap/core/map/map_factory.h>
#include <wavemap/core/utils/profile/profiler_interface.h>
#include <wavemap/io/file_conversions.h>

#include "wavemap_ros2/inputs/ros_input_factory.h"
#include "wavemap_ros2/map_operations/map_ros_operation_factory.h"
#include "wavemap_ros2_conversions/config_conversions.h"

namespace wavemap {
RosServer::RosServer(const rclcpp::NodeOptions& options)
    : Node("wavemap", options) {
  // Declare and get config_file parameter
  config_file_ = declare_parameter<std::string>("config_file", "");
  if (config_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "No config_file parameter specified!");
    return;
  }

  // Load the config
  auto node_shared = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {});
  const auto params = param::convert::yamlFileToParamMap(config_file_);

  // Read general settings
  if (auto general_it = params.find("general"); general_it != params.end()) {
    const auto general_val = param::Value(general_it->second);
    if (auto wf = general_val.getChildAs<std::string>("world_frame"); wf) {
      world_frame_ = *wf;
    }
    if (auto nt = general_val.getChildAs<int>("num_threads"); nt) {
      num_threads_ = *nt;
    }
    if (auto ll = general_val.getChildAs<std::string>("logging_level"); ll) {
      if (auto level = RosLoggingLevel(*ll); level.isValid()) {
        logging_level_ = level;
      }
    }
  }

  // Apply logging level
  logging_level_.applyToGlog();
  logging_level_.applyToRos2Logger(get_logger().get_name());

  // Setup the TF transformer
  transformer_ = std::make_shared<TfTransformer>(node_shared);

  // Create the map and pipeline
  if (auto map_it = params.find("map"); map_it != params.end()) {
    occupancy_map_ = MapFactory::create(map_it->second);
  }
  if (!occupancy_map_) {
    RCLCPP_ERROR(get_logger(), "Could not create map. Shutting down.");
    return;
  }
  auto thread_pool =
      std::make_shared<ThreadPool>(num_threads_ < 0 ? 0 : num_threads_);
  pipeline_ = std::make_shared<Pipeline>(occupancy_map_, thread_pool);

  // Setup measurement integrators
  if (auto it = params.find("measurement_integrators"); it != params.end()) {
    if (auto integrators_map = it->second.as<param::Map>(); integrators_map) {
      for (const auto& [integrator_name, integrator_params] :
           *integrators_map) {
        pipeline_->addIntegrator(integrator_name, integrator_params);
      }
    }
  }

  // Setup inputs
  if (auto it = params.find("inputs"); it != params.end()) {
    if (auto inputs = it->second.as<param::Array>(); inputs) {
      for (const auto& input_params : *inputs) {
        auto input = RosInputFactory::create(
            input_params, pipeline_, transformer_, world_frame_, node_shared);
        if (input) {
          inputs_.emplace_back(std::move(input));
        }
      }
    }
  }

  // Setup map operations
  if (auto it = params.find("map_operations"); it != params.end()) {
    if (auto ops = it->second.as<param::Array>(); ops) {
      for (const auto& op_params : *ops) {
        auto operation = MapRosOperationFactory::create(
            op_params, occupancy_map_, thread_pool, transformer_, world_frame_,
            node_shared);
        if (operation) {
          pipeline_->addOperation(std::move(operation));
        }
      }
    }
  }

  // Advertise services
  reset_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "reset_map",
      std::bind(&RosServer::resetMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  save_map_srv_ = create_service<wavemap_msgs::srv::FilePath>(
      "save_map",
      std::bind(&RosServer::saveMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  load_map_srv_ = create_service<wavemap_msgs::srv::FilePath>(
      "load_map",
      std::bind(&RosServer::loadMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Wavemap ROS2 server initialized.");
}

void RosServer::resetMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (occupancy_map_) {
    occupancy_map_->clear();
    response->success = true;
    response->message = "Map was reset.";
    RCLCPP_INFO(get_logger(), "Map was reset.");
  } else {
    response->success = false;
    response->message = "Could not reset the map (not initialized).";
    RCLCPP_WARN(get_logger(), "Could not reset the map (not initialized).");
  }
}

void RosServer::saveMapCallback(
    const std::shared_ptr<wavemap_msgs::srv::FilePath::Request> request,
    std::shared_ptr<wavemap_msgs::srv::FilePath::Response> response) {
  if (!occupancy_map_) {
    response->success = false;
    return;
  }
  occupancy_map_->threshold();
  response->success = io::mapToFile(*occupancy_map_, request->file_path);
  if (response->success) {
    RCLCPP_INFO(get_logger(), "Map saved to '%s'.", request->file_path.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Failed to save map to '%s'.",
                request->file_path.c_str());
  }
}

void RosServer::loadMapCallback(
    const std::shared_ptr<wavemap_msgs::srv::FilePath::Request> request,
    std::shared_ptr<wavemap_msgs::srv::FilePath::Response> response) {
  response->success = io::fileToMap(request->file_path, occupancy_map_);
  if (response->success) {
    RCLCPP_INFO(get_logger(), "Map loaded from '%s'.",
                request->file_path.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Failed to load map from '%s'.",
                request->file_path.c_str());
  }
}
}  // namespace wavemap
