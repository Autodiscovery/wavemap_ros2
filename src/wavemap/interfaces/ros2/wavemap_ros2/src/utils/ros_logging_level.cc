#include "wavemap_ros2/utils/ros_logging_level.h"

#include <string>

namespace wavemap {
RosLoggingLevel::operator LoggingLevel() const {
  if (id_ == Id::kDebug) {
    return LoggingLevel::kInfo;
  } else if (Id::kInfo < id_ && id_ <= Id::kFatal) {
    return id_ - 1;
  } else {
    return LoggingLevel::kInvalidTypeId;
  }
}

void RosLoggingLevel::applyToGlog() const {
  operator LoggingLevel().applyToGlog();
}

bool RosLoggingLevel::applyToRos2Logger(const std::string& name) const {
  auto ret = rcutils_logging_set_logger_level(
      name.c_str(), ros_levels[toTypeId()]);
  return ret == RCUTILS_RET_OK;
}
}  // namespace wavemap
