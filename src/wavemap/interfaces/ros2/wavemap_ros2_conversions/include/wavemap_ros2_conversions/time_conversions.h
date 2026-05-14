#ifndef WAVEMAP_ROS2_CONVERSIONS_TIME_CONVERSIONS_H_
#define WAVEMAP_ROS2_CONVERSIONS_TIME_CONVERSIONS_H_

#include <rclcpp/rclcpp.hpp>

namespace wavemap::convert {
inline rclcpp::Time nanoSecondsToRosTime(uint64_t nsec) {
  return rclcpp::Time(static_cast<int64_t>(nsec));
}

inline double nanoSecondsToSeconds(uint64_t nsec) {
  constexpr double kNsecToSec = 1e-9;
  return static_cast<double>(nsec) * kNsecToSec;
}

inline uint64_t rosTimeToNanoSeconds(const rclcpp::Time& time) {
  return static_cast<uint64_t>(time.nanoseconds());
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS2_CONVERSIONS_TIME_CONVERSIONS_H_
