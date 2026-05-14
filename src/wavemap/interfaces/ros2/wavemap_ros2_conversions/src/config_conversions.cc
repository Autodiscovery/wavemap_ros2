#include "wavemap_ros2_conversions/config_conversions.h"

#include <fstream>
#include <string>

#include <yaml-cpp/yaml.h>

namespace wavemap::param::convert {
namespace {
// Forward declarations for recursive conversion
param::Value yamlNodeToParamValue(const YAML::Node& node);
param::Map yamlNodeToParamMap(const YAML::Node& node);
param::Array yamlNodeToParamArray(const YAML::Node& node);

param::Value yamlNodeToParamValue(const YAML::Node& node) {
  switch (node.Type()) {
    case YAML::NodeType::Scalar: {
      // Try to interpret as bool first
      try {
        return param::Value(node.as<bool>());
      } catch (...) {}
      // Try int
      try {
        return param::Value(node.as<int>());
      } catch (...) {}
      // Try double
      try {
        return param::Value(node.as<double>());
      } catch (...) {}
      // Fall back to string
      return param::Value(node.as<std::string>());
    }
    case YAML::NodeType::Sequence:
      return param::Value(yamlNodeToParamArray(node));
    case YAML::NodeType::Map:
      return param::Value(yamlNodeToParamMap(node));
    case YAML::NodeType::Null:
    case YAML::NodeType::Undefined:
    default:
      RCLCPP_ERROR(rclcpp::get_logger("wavemap"),
                   "Encountered null/undefined YAML node.");
      return param::Value(param::Array{});
  }
}

param::Map yamlNodeToParamMap(const YAML::Node& node) {
  param::Map param_map;
  for (const auto& kv : node) {
    param_map.emplace(kv.first.as<std::string>(),
                      yamlNodeToParamValue(kv.second));
  }
  return param_map;
}

param::Array yamlNodeToParamArray(const YAML::Node& node) {
  param::Array array;
  array.reserve(node.size());
  for (const auto& element : node) {
    array.emplace_back(yamlNodeToParamValue(element));
  }
  return array;
}
}  // namespace

param::Map yamlFileToParamMap(const std::string& file_path) {
  try {
    YAML::Node root = YAML::LoadFile(file_path);
    return yamlNodeToParamMap(root);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("wavemap"),
                 "Failed to load YAML file '%s': %s",
                 file_path.c_str(), e.what());
    return {};
  }
}

param::Array yamlFileToParamArray(const std::string& file_path,
                                  const std::string& ns) {
  try {
    YAML::Node root = YAML::LoadFile(file_path);
    if (root[ns]) {
      return yamlNodeToParamArray(root[ns]);
    }
    RCLCPP_WARN(rclcpp::get_logger("wavemap"),
                "Namespace '%s' not found in YAML file '%s'",
                ns.c_str(), file_path.c_str());
    return {};
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("wavemap"),
                 "Failed to load YAML file '%s': %s",
                 file_path.c_str(), e.what());
    return {};
  }
}

param::Value yamlFileToParamValue(const std::string& file_path,
                                  const std::string& ns) {
  try {
    YAML::Node root = YAML::LoadFile(file_path);
    if (root[ns]) {
      return yamlNodeToParamValue(root[ns]);
    }
    RCLCPP_WARN(rclcpp::get_logger("wavemap"),
                "Namespace '%s' not found in YAML file '%s'",
                ns.c_str(), file_path.c_str());
    return param::Value{param::Map{}};
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("wavemap"),
                 "Failed to load YAML file '%s': %s",
                 file_path.c_str(), e.what());
    return param::Value{param::Map{}};
  }
}

param::Value rclParameterToParamValue(const rclcpp::Parameter& parameter) {
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return param::Value(parameter.as_bool());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return param::Value(static_cast<int>(parameter.as_int()));
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return param::Value(parameter.as_double());
    case rclcpp::ParameterType::PARAMETER_STRING:
      return param::Value(parameter.as_string());
    default:
      RCLCPP_ERROR(rclcpp::get_logger("wavemap"),
                   "Unsupported ROS2 parameter type for '%s'.",
                   parameter.get_name().c_str());
      return param::Value(param::Array{});
  }
}

param::Map toParamMap(const rclcpp::Node::SharedPtr& node,
                      const std::string& ns) {
  // Try to get the config_file parameter first
  std::string config_file;
  if (node->has_parameter("config_file")) {
    config_file = node->get_parameter("config_file").as_string();
  } else {
    config_file =
        node->declare_parameter<std::string>("config_file", "");
  }

  if (!config_file.empty()) {
    try {
      YAML::Node root = YAML::LoadFile(config_file);
      if (root[ns]) {
        return yamlNodeToParamMap(root[ns]);
      }
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(node->get_logger(),
                   "Failed to load config file '%s': %s",
                   config_file.c_str(), e.what());
    }
  }

  RCLCPP_WARN(node->get_logger(),
              "Could not load params under namespace '%s'", ns.c_str());
  return {};
}

param::Array toParamArray(const rclcpp::Node::SharedPtr& node,
                          const std::string& ns) {
  std::string config_file;
  if (node->has_parameter("config_file")) {
    config_file = node->get_parameter("config_file").as_string();
  } else {
    config_file =
        node->declare_parameter<std::string>("config_file", "");
  }

  if (!config_file.empty()) {
    try {
      YAML::Node root = YAML::LoadFile(config_file);
      if (root[ns]) {
        return yamlNodeToParamArray(root[ns]);
      }
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(node->get_logger(),
                   "Failed to load config file '%s': %s",
                   config_file.c_str(), e.what());
    }
  }

  RCLCPP_WARN(node->get_logger(),
              "Could not load params under namespace '%s'", ns.c_str());
  return {};
}

param::Value toParamValue(const rclcpp::Node::SharedPtr& node,
                          const std::string& ns) {
  std::string config_file;
  if (node->has_parameter("config_file")) {
    config_file = node->get_parameter("config_file").as_string();
  } else {
    config_file =
        node->declare_parameter<std::string>("config_file", "");
  }

  if (!config_file.empty()) {
    try {
      YAML::Node root = YAML::LoadFile(config_file);
      if (root[ns]) {
        return yamlNodeToParamValue(root[ns]);
      }
    } catch (const YAML::Exception& e) {
      RCLCPP_ERROR(node->get_logger(),
                   "Failed to load config file '%s': %s",
                   config_file.c_str(), e.what());
    }
  }

  RCLCPP_WARN(node->get_logger(),
              "Could not load params under namespace '%s'", ns.c_str());
  return param::Value{param::Map{}};
}
}  // namespace wavemap::param::convert
