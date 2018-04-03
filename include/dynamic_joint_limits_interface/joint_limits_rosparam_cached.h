#ifndef DYNAMIC_JOINT_LIMITS_INTERFACE_JOINT_LIMITS_ROSPARAM_CACHED_H
#define DYNAMIC_JOINT_LIMITS_INTERFACE_JOINT_LIMITS_ROSPARAM_CACHED_H

#include <string>

#include <joint_limits_interface/joint_limits.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace dynamic_joint_limits_interface {

// this is almost equivarent to joint_limits_interface::getJointLimits()
// but uses getParamCached() instead of getParam() to avoid blocking access to the parameter server
inline bool getJointLimitsCached(const std::string &joint_name, const ros::NodeHandle &nh,
                                 joint_limits_interface::JointLimits &limits) {
  // Node handle scoped where the joint limits are defined
  ros::NodeHandle limits_nh;
  try {
    // never check parameter exists by hasParam() dislike joint_limits_interface::getJointLimits()
    // because we want to start caching parameters in background anyway
    limits_nh = ros::NodeHandle(nh, "joint_limits/" + joint_name);
  } catch (const ros::InvalidNameException &ex) {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  // Position limits
  bool has_position_limits = false;
  if (limits_nh.getParamCached("has_position_limits", has_position_limits)) {
    if (!has_position_limits) {
      limits.has_position_limits = false;
    }
    double min_pos, max_pos;
    if (has_position_limits && limits_nh.getParamCached("min_position", min_pos) &&
        limits_nh.getParamCached("max_position", max_pos)) {
      limits.has_position_limits = true;
      limits.min_position = min_pos;
      limits.max_position = max_pos;
    }

    bool angle_wraparound;
    if (!has_position_limits && limits_nh.getParamCached("angle_wraparound", angle_wraparound)) {
      limits.angle_wraparound = angle_wraparound;
    }
  }

  // Velocity limits
  bool has_velocity_limits = false;
  if (limits_nh.getParamCached("has_velocity_limits", has_velocity_limits)) {
    if (!has_velocity_limits) {
      limits.has_velocity_limits = false;
    }
    double max_vel;
    if (has_velocity_limits && limits_nh.getParamCached("max_velocity", max_vel)) {
      limits.has_velocity_limits = true;
      limits.max_velocity = max_vel;
    }
  }

  // Acceleration limits
  bool has_acceleration_limits = false;
  if (limits_nh.getParamCached("has_acceleration_limits", has_acceleration_limits)) {
    if (!has_acceleration_limits) {
      limits.has_acceleration_limits = false;
    }
    double max_acc;
    if (has_acceleration_limits && limits_nh.getParamCached("max_acceleration", max_acc)) {
      limits.has_acceleration_limits = true;
      limits.max_acceleration = max_acc;
    }
  }

  // Jerk limits
  bool has_jerk_limits = false;
  if (limits_nh.getParamCached("has_jerk_limits", has_jerk_limits)) {
    if (!has_jerk_limits) {
      limits.has_jerk_limits = false;
    }
    double max_jerk;
    if (has_jerk_limits && limits_nh.getParamCached("max_jerk", max_jerk)) {
      limits.has_jerk_limits = true;
      limits.max_jerk = max_jerk;
    }
  }

  // Effort limits
  bool has_effort_limits = false;
  if (limits_nh.getParamCached("has_effort_limits", has_effort_limits)) {
    if (!has_effort_limits) {
      limits.has_effort_limits = false;
    }
    double max_effort;
    if (has_effort_limits && limits_nh.getParamCached("max_effort", max_effort)) {
      limits.has_effort_limits = true;
      limits.max_effort = max_effort;
    }
  }

  return true;
}

} // namespace dynamic_joint_limits_interface

#endif