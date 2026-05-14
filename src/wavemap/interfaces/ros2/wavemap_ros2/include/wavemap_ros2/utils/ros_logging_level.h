#ifndef WAVEMAP_ROS2_UTILS_ROS_LOGGING_LEVEL_H_
#define WAVEMAP_ROS2_UTILS_ROS_LOGGING_LEVEL_H_

#include <string>

#include <rcutils/logging.h>
#include <wavemap/core/config/type_selector.h>
#include <wavemap/core/utils/logging_level.h>

namespace wavemap {
struct RosLoggingLevel : public TypeSelector<RosLoggingLevel> {
  using TypeSelector<RosLoggingLevel>::TypeSelector;

  enum Id : TypeId { kDebug, kInfo, kWarning, kError, kFatal };

  static constexpr std::array names = {"debug", "info", "warning", "error",
                                       "fatal"};
  static constexpr std::array ros_levels = {
      RCUTILS_LOG_SEVERITY_DEBUG, RCUTILS_LOG_SEVERITY_INFO,
      RCUTILS_LOG_SEVERITY_WARN, RCUTILS_LOG_SEVERITY_ERROR,
      RCUTILS_LOG_SEVERITY_FATAL};

  // Conversion to general LoggingLevel (from the C++ Library)
  operator LoggingLevel() const;  // NOLINT

  // Apply the logger level to a given output
  void applyToGlog() const;
  bool applyToRos2Logger(const std::string& name = "") const;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS2_UTILS_ROS_LOGGING_LEVEL_H_
