#ifndef DYNAMIC_JOINT_LIMITS_INTERFACE_JOINT_LIMITS_ROSPARAM_CACHED_H
#define DYNAMIC_JOINT_LIMITS_INTERFACE_JOINT_LIMITS_ROSPARAM_CACHED_H

#include <string>

#include <dynamic_joint_limits_interface/joint_limits.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace dynamic_joint_limits_interface {

// helper macro to try loading a cached param
#define DJLI_GET_CACHE(name, value) const bool value##_loaded(limits_nh.getParamCached(name, value))

// Note about getParamCached()
//   - first call: start subscribing the parameter with access to the param server
//   - second call or futher: read local cache without access to the param server
// => to never access the param server except the first call of following functions,
//    all calls of getParamCached() are made outside of if-statements.

// this is almost equivarent to joint_limits_interface::getJointLimits()
// but uses getParamCached() instead of getParam() to avoid blocking access to the parameter server
inline bool getJointLimitsCached(const std::string &joint_name, const ros::NodeHandle &nh,
                                 JointLimits &limits) {
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
  bool has_pos_limits;
  DJLI_GET_CACHE("has_position_limits", has_pos_limits);
  double min_pos, max_pos;
  DJLI_GET_CACHE("min_position", min_pos);
  DJLI_GET_CACHE("max_position", max_pos);
  bool angle_wraparound;
  DJLI_GET_CACHE("angle_wraparound", angle_wraparound);
  if (has_pos_limits_loaded) {
    if (!has_pos_limits) {
      limits.has_position_limits = false;
    }
    if (has_pos_limits && min_pos_loaded && max_pos_loaded) {
      limits.has_position_limits = true;
      limits.min_position = min_pos;
      limits.max_position = max_pos;
    }
    if (!has_pos_limits && angle_wraparound_loaded) {
      limits.angle_wraparound = angle_wraparound;
    }
  }

  // Velocity limits
  bool has_vel_limits;
  DJLI_GET_CACHE("has_velocity_limits", has_vel_limits);
  double max_vel;
  DJLI_GET_CACHE("max_velocity", max_vel);
  if (has_vel_limits_loaded) {
    if (!has_vel_limits) {
      limits.has_velocity_limits = false;
    }
    if (has_vel_limits && max_vel_loaded) {
      limits.has_velocity_limits = true;
      limits.max_velocity = max_vel;
    }
  }

  // Acceleration limits
  bool has_acc_limits;
  DJLI_GET_CACHE("has_acceleration_limits", has_acc_limits);
  double max_acc;
  DJLI_GET_CACHE("max_acceleration", max_acc);
  if (has_acc_limits_loaded) {
    if (!has_acc_limits) {
      limits.has_acceleration_limits = false;
    }
    if (has_acc_limits && max_acc_loaded) {
      limits.has_acceleration_limits = true;
      limits.max_acceleration = max_acc;
    }
  }

  // Jerk limits
  bool has_jerk_limits;
  DJLI_GET_CACHE("has_jerk_limits", has_jerk_limits);
  double max_jerk;
  DJLI_GET_CACHE("max_jerk", max_jerk);
  if (has_jerk_limits_loaded) {
    if (!has_jerk_limits) {
      limits.has_jerk_limits = false;
    }
    if (has_jerk_limits && max_jerk_loaded) {
      limits.has_jerk_limits = true;
      limits.max_jerk = max_jerk;
    }
  }

  // Effort limits
  bool has_eff_limits;
  DJLI_GET_CACHE("has_effort_limits", has_eff_limits);
  double max_eff;
  DJLI_GET_CACHE("max_effort", max_eff);
  if (has_eff_limits_loaded) {
    if (!has_eff_limits) {
      limits.has_effort_limits = false;
    }
    if (has_eff_limits && max_eff_loaded) {
      limits.has_effort_limits = true;
      limits.max_effort = max_eff;
    }
  }

  return true;
}

// this is almost equivarent to joint_limits_interface::getSoftJointLimits()
// but uses getParamCached() instead of getParam() to avoid blocking access to the parameter server
inline bool getSoftJointLimitsCached(const std::string &joint_name, const ros::NodeHandle &nh,
                                     SoftJointLimits &soft_limits) {
  // Node handle scoped where the soft joint limits are defined
  ros::NodeHandle limits_nh;
  try {
    // never check parameter exists by hasParam() dislike
    // joint_limits_interface::getSoftJointLimits()
    // because we want to start caching parameters in background anyway
    limits_nh = ros::NodeHandle(nh, "joint_limits/" + joint_name);
  } catch (const ros::InvalidNameException &ex) {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  // Override soft limits if complete specification is found
  bool has_soft_limits;
  DJLI_GET_CACHE("has_soft_limits", has_soft_limits);
  double k_position, k_velocity;
  DJLI_GET_CACHE("k_position", k_position);
  DJLI_GET_CACHE("k_velocity", k_velocity);
  double soft_upper_limit, soft_lower_limit;
  DJLI_GET_CACHE("soft_upper_limit", soft_upper_limit);
  DJLI_GET_CACHE("soft_lower_limit", soft_lower_limit);
  if (has_soft_limits_loaded) {
    if (!has_soft_limits) {
      soft_limits.has_soft_limits = false;
    }
    if (has_soft_limits && k_position_loaded && k_velocity_loaded && soft_upper_limit_loaded &&
        soft_lower_limit_loaded) {
      soft_limits.has_soft_limits = true;
      soft_limits.k_position = k_position;
      soft_limits.k_velocity = k_velocity;
      soft_limits.max_position = soft_upper_limit;
      soft_limits.min_position = soft_lower_limit;
    }
  }

  return true;
}

} // namespace dynamic_joint_limits_interface

#endif