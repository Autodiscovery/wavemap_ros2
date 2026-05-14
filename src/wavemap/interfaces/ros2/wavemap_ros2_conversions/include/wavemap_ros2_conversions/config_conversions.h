#ifndef WAVEMAP_ROS2_CONVERSIONS_CONFIG_CONVERSIONS_H_
#define WAVEMAP_ROS2_CONVERSIONS_CONFIG_CONVERSIONS_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <wavemap/core/config/config_base.h>

namespace wavemap::param::convert {
// Load wavemap config from a YAML file
param::Map yamlFileToParamMap(const std::string& file_path);
param::Array yamlFileToParamArray(const std::string& file_path,
                                  const std::string& ns);
param::Value yamlFileToParamValue(const std::string& file_path,
                                  const std::string& ns);

// Load wavemap config from a ROS2 node's parameters
param::Map toParamMap(const rclcpp::Node::SharedPtr& node,
                      const std::string& ns);
param::Array toParamArray(const rclcpp::Node::SharedPtr& node,
                          const std::string& ns);
param::Value toParamValue(const rclcpp::Node::SharedPtr& node,
                          const std::string& ns);

// Convert rclcpp::Parameter to wavemap param types
param::Value rclParameterToParamValue(const rclcpp::Parameter& parameter);
}  // namespace wavemap::param::convert

#endif  // WAVEMAP_ROS2_CONVERSIONS_CONFIG_CONVERSIONS_H_
