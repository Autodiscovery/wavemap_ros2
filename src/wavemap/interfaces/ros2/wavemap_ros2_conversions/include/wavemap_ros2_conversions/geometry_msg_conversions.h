#ifndef WAVEMAP_ROS2_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS2_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_

#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <wavemap/core/common.h>

namespace wavemap::convert {
inline Point3D pointMsgToPoint3D(const geometry_msgs::msg::Point& msg) {
  Eigen::Vector3d point_double;
  tf2::fromMsg(msg, point_double);
  return point_double.cast<FloatingPoint>();
}

inline geometry_msgs::msg::Point point3DToPointMsg(const Point3D& point) {
  return tf2::toMsg(Eigen::Vector3d(point.cast<double>()));
}

inline Point3D point32MsgToPoint3D(const geometry_msgs::msg::Point32& msg) {
  return {msg.x, msg.y, msg.z};
}

inline geometry_msgs::msg::Point32 point3DToPoint32Msg(const Point3D& point) {
  geometry_msgs::msg::Point32 msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

inline Vector3D vector3MsgToVector3D(const geometry_msgs::msg::Vector3& msg) {
  Eigen::Vector3d vector_double;
  tf2::fromMsg(msg, vector_double);
  return vector_double.cast<FloatingPoint>();
}

inline geometry_msgs::msg::Vector3 vector3DToVector3Msg(
    const Vector3D& vector) {
  geometry_msgs::msg::Vector3 msg;
  msg.x = vector.x();
  msg.y = vector.y();
  msg.z = vector.z();
  return msg;
}

inline Rotation3D quaternionMsgToRotation3D(
    const geometry_msgs::msg::Quaternion& msg) {
  Eigen::Quaterniond rotation_double;
  tf2::fromMsg(msg, rotation_double);
  return Rotation3D{rotation_double.cast<FloatingPoint>()};
}

inline Transformation3D transformMsgToTransformation3D(
    const geometry_msgs::msg::Transform& msg) {
  return Transformation3D{quaternionMsgToRotation3D(msg.rotation),
                          vector3MsgToVector3D(msg.translation)};
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS2_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
