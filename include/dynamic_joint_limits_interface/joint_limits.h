#ifndef DYNAMIC_JOINT_LIMITS_INTERFACE_JOINT_LIMITS_H
#define DYNAMIC_JOINT_LIMITS_INTERFACE_JOINT_LIMITS_H

#include <joint_limits_interface/joint_limits.h>

namespace dynamic_joint_limits_interface {

// equivarent of joint_limits_interface::JointLimits
struct JointLimits : public joint_limits_interface::JointLimits {};

// add has_soft_limits to allow "no limits" state like JointLimits
struct SoftJointLimits : public joint_limits_interface::SoftJointLimits {
  SoftJointLimits() : has_soft_limits(false) {}

  bool has_soft_limits;
};

} // namespace dynamic_joint_limits_interface

#endif