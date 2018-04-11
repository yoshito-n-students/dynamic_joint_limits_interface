#ifndef DYNAMIC_JOINT_LIMITS_INTERFACE_JOINT_LIMITS_INTERFACE_H
#define DYNAMIC_JOINT_LIMITS_INTERFACE_JOINT_LIMITS_INTERFACE_H

#include <cmath>
#include <limits>
#include <string>

#include <dynamic_joint_limits_interface/joint_limits.h>
#include <dynamic_joint_limits_interface/joint_limits_rosparam_cached.h>
#include <hardware_interface/internal/resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <boost/algorithm/clamp.hpp>
#include <boost/foreach.hpp>

namespace dynamic_joint_limits_interface {

// helper to deal with range of saturation
struct Range {
  Range(const double _min, const double _max) : min(_min), max(_max) {}
  // initializers
  static Range entire() {
    return Range(-std::numeric_limits< double >::max(), std::numeric_limits< double >::max());
  }
  static Range positive() { return Range(0., std::numeric_limits< double >::max()); }
  static Range negative() { return Range(-std::numeric_limits< double >::max(), 0.); }
  // saturation
  double clamp(const double val) const { return boost::algorithm::clamp(val, min, max); }
  Range clamp(const Range &range) const { return Range(clamp(range.min), clamp(range.max)); }

  double min, max;
};

// they are almost equivarents to joint_limits_interface::XXXJointYyyHandle
// but can update limits after construction by updateLimits()

class PositionJointSaturationHandle {
public:
  PositionJointSaturationHandle(const hardware_interface::JointHandle &jh,
                                const JointLimits &limits = JointLimits())
      : jh_(jh), limits_(limits), prev_cmd_(std::numeric_limits< double >::quiet_NaN()) {
    // never throw dislike joint_limits_interface::PositionJointSaturationHandle
    // according to the initial limits because it can be updated by updateLimits()
  }

  virtual ~PositionJointSaturationHandle() {}

  std::string getName() const { return jh_.getName(); }

  void enforceLimits(const ros::Duration &period) {
    if (std::isnan(prev_cmd_)) {
      prev_cmd_ = jh_.getPosition();
    }

    Range pos_range(Range::entire());
    if (limits_.has_velocity_limits) {
      const double delta_pos(limits_.max_velocity * period.toSec());
      pos_range = Range(prev_cmd_ - delta_pos, prev_cmd_ + delta_pos).clamp(pos_range);
    }
    if (limits_.has_position_limits) {
      pos_range = Range(limits_.min_position, limits_.max_position).clamp(pos_range);
    }

    const double cmd(pos_range.clamp(jh_.getCommand()));
    jh_.setCommand(cmd);
    prev_cmd_ = cmd;
  }

  void updateLimits(ros::NodeHandle &nh) { getJointLimitsCached(getName(), nh, limits_); }

  void reset() { prev_cmd_ = std::numeric_limits< double >::quiet_NaN(); }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
  double prev_cmd_;
};

class PositionJointSoftLimitsHandle {
public:
  PositionJointSoftLimitsHandle(const hardware_interface::JointHandle &jh,
                                const JointLimits &limits = JointLimits(),
                                const SoftJointLimits &soft_limits = SoftJointLimits())
      : jh_(jh), limits_(limits), soft_limits_(soft_limits),
        prev_cmd_(std::numeric_limits< double >::quiet_NaN()) {
    // never throw dislike joint_limits_interface::PositionJointSoftLimitsHandle
    // according to the initial limits because it can be updated by updateLimits()
  }

  std::string getName() const { return jh_.getName(); }

  void enforceLimits(const ros::Duration &period) {
    if (std::isnan(prev_cmd_)) {
      prev_cmd_ = jh_.getPosition();
    }

    // Velocity bounds
    Range vel_range(Range::entire());
    if (soft_limits_.has_soft_limits) {
      vel_range = Range(-soft_limits_.k_position * (prev_cmd_ - soft_limits_.min_position),
                        -soft_limits_.k_position * (prev_cmd_ - soft_limits_.max_position))
                      .clamp(vel_range);
    }
    if (limits_.has_velocity_limits) {
      vel_range = Range(-limits_.max_velocity, limits_.max_velocity).clamp(vel_range);
    }

    // Position bounds
    const double dt(period.toSec());
    Range pos_range(prev_cmd_ + vel_range.min * dt, prev_cmd_ + vel_range.max * dt);
    if (limits_.has_position_limits) {
      pos_range = Range(limits_.min_position, limits_.max_position).clamp(pos_range);
    }

    // Saturate position command according to bounds
    const double cmd(pos_range.clamp(jh_.getCommand()));
    jh_.setCommand(cmd);
    prev_cmd_ = cmd;
  }

  void updateLimits(ros::NodeHandle &nh) {
    getJointLimitsCached(getName(), nh, limits_);
    getSoftJointLimitsCached(getName(), nh, soft_limits_);
  }

  void reset() { prev_cmd_ = std::numeric_limits< double >::quiet_NaN(); }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
  SoftJointLimits soft_limits_;
  double prev_cmd_;
};

class VelocityJointSaturationHandle {
public:
  VelocityJointSaturationHandle(const hardware_interface::JointHandle &jh,
                                const JointLimits &limits = JointLimits())
      : jh_(jh), limits_(limits) {
    // never throw dislike joint_limits_interface::VelocityJointSaturationHandle
    // according to the initial limits because it can be updated by updateLimits()
  }

  virtual ~VelocityJointSaturationHandle() {}

  std::string getName() const { return jh_.getName(); }

  void enforceLimits(const ros::Duration &period) {
    Range vel_range(Range::entire());
    if (limits_.has_acceleration_limits) {
      const double vel(jh_.getVelocity());
      const double delta_vel(limits_.max_acceleration * period.toSec());
      vel_range = Range(vel - delta_vel, vel + delta_vel).clamp(vel_range);
    }
    if (limits_.has_velocity_limits) {
      vel_range = Range(-limits_.max_velocity, limits_.max_velocity).clamp(vel_range);
    }

    jh_.setCommand(vel_range.clamp(jh_.getCommand()));
  }

  void updateLimits(ros::NodeHandle &nh) { getJointLimitsCached(getName(), nh, limits_); }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
};

class VelocityJointSoftLimitsHandle {
public:
  VelocityJointSoftLimitsHandle(const hardware_interface::JointHandle &jh,
                                const JointLimits &limits = JointLimits(),
                                const SoftJointLimits &soft_limits = SoftJointLimits())
      : jh_(jh), limits_(limits), soft_limits_(soft_limits_) {}

  std::string getName() const { return jh_.getName(); }

  void enforceLimits(const ros::Duration &period) {
    Range vel_range(Range::entire());
    if (soft_limits_.has_soft_limits) {
      const double pos(jh_.getPosition());
      vel_range = Range(-soft_limits_.k_position * (pos - soft_limits_.min_position),
                        -soft_limits_.k_position * (pos - soft_limits_.max_position))
                      .clamp(vel_range);
    }
    if (limits_.has_acceleration_limits) {
      const double vel(jh_.getVelocity());
      const double delta_vel(limits_.max_acceleration * period.toSec());
      vel_range = Range(vel - delta_vel, vel + delta_vel).clamp(vel_range);
    }
    if (limits_.has_velocity_limits) {
      vel_range = Range(-limits_.max_velocity, limits_.max_velocity).clamp(vel_range);
    }

    jh_.setCommand(vel_range.clamp(jh_.getCommand()));
  }

  void updateLimits(ros::NodeHandle &nh) {
    getJointLimitsCached(getName(), nh, limits_);
    getSoftJointLimitsCached(getName(), nh, soft_limits_);
  }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
  SoftJointLimits soft_limits_;
};

class EffortJointSaturationHandle {
public:
  EffortJointSaturationHandle(const hardware_interface::JointHandle &jh,
                              const JointLimits &limits = JointLimits())
      : jh_(jh), limits_(limits) {
    // never throw dislike joint_limits_interface::EffortJointSaturationHandle
    // according to the initial limits because it can be updated by updateLimits()
  }

  virtual ~EffortJointSaturationHandle() {}

  std::string getName() const { return jh_.getName(); }

  void enforceLimits(const ros::Duration & /* period */) {
    Range eff_range(Range::entire());
    if (limits_.has_effort_limits) {
      eff_range = Range(-limits_.max_effort, limits_.max_effort).clamp(eff_range);
    }
    if (limits_.has_position_limits) {
      const double pos(jh_.getPosition());
      if (pos < limits_.min_position) {
        eff_range = Range::positive().clamp(eff_range);
      } else if (pos > limits_.max_position) {
        eff_range = Range::negative().clamp(eff_range);
      }
    }
    if (limits_.has_velocity_limits) {
      const double vel(jh_.getVelocity());
      if (vel < -limits_.max_velocity) {
        eff_range = Range::positive().clamp(eff_range);
      } else if (vel > limits_.max_velocity) {
        eff_range = Range::negative().clamp(eff_range);
      }
    }

    jh_.setCommand(eff_range.clamp(jh_.getCommand()));
  }

  void updateLimits(ros::NodeHandle &nh) { getJointLimitsCached(getName(), nh, limits_); }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
};

class EffortJointSoftLimitsHandle {
public:
  EffortJointSoftLimitsHandle(const hardware_interface::JointHandle &jh,
                              const JointLimits &limits = JointLimits(),
                              const SoftJointLimits &soft_limits = SoftJointLimits())
      : jh_(jh), limits_(limits), soft_limits_(soft_limits) {}

  std::string getName() const { return jh_.getName(); }

  void enforceLimits(const ros::Duration & /*period*/) {
    // Velocity bounds
    Range vel_range(Range::entire());
    if (soft_limits_.has_soft_limits) {
      const double pos(jh_.getPosition());
      vel_range = Range(-soft_limits_.k_position * (pos - soft_limits_.min_position),
                        -soft_limits_.k_position * (pos - soft_limits_.max_position))
                      .clamp(vel_range);
    }
    // consider acc limits like VelocityJointSoftLimitsHandle??
    if (limits_.has_velocity_limits) {
      vel_range = Range(-limits_.max_velocity, limits_.max_velocity).clamp(vel_range);
    }

    // Effort bounds depend on the velocity and effort bounds
    Range eff_range(Range::entire());
    if (soft_limits_.has_soft_limits) {
      const double vel(jh_.getVelocity());
      eff_range = Range(-soft_limits_.k_velocity * (vel - vel_range.min),
                        -soft_limits_.k_velocity * (vel - vel_range.max))
                      .clamp(eff_range);
    }
    if (limits_.has_effort_limits) {
      eff_range = Range(-limits_.max_effort, limits_.max_effort).clamp(eff_range);
    }
    // consider pos & vel limits like EffortJointSaturationHandle??

    // Saturate effort command according to bounds
    jh_.setCommand(eff_range.clamp(jh_.getCommand()));
  }

  void updateLimits(ros::NodeHandle &nh) {
    getJointLimitsCached(getName(), nh, limits_);
    getSoftJointLimitsCached(getName(), nh, soft_limits_);
  }

private:
  hardware_interface::JointHandle jh_;
  JointLimits limits_;
  SoftJointLimits soft_limits_;
};

// this is almost an equivarent to joint_limits_interface::JointLimitsInterface
// but has updateLimits() which updates limits in managed handles

template < class LimitsHandle >
class JointLimitsInterface : public hardware_interface::ResourceManager< LimitsHandle > {
protected:
  typedef typename hardware_interface::ResourceManager< LimitsHandle > Base;

public:
  virtual ~JointLimitsInterface() {}

  void enforceLimits(const ros::Duration &period) {
    BOOST_FOREACH (typename Base::ResourceMap::value_type &resource_pair, Base::resource_map_) {
      resource_pair.second.enforceLimits(period);
    }
  }

  void updateLimits(ros::NodeHandle &nh) {
    BOOST_FOREACH (typename Base::ResourceMap::value_type &resource_pair, Base::resource_map_) {
      resource_pair.second.updateLimits(nh);
    }
  }
};

// they are almost equivarents to joint_limits_interface::XxxJointYyyInterface
// but inherits DynamicXxxJointYyyInterface

class PositionJointSaturationInterface
    : public JointLimitsInterface< PositionJointSaturationHandle > {
public:
  virtual ~PositionJointSaturationInterface() {}

  void reset() {
    BOOST_FOREACH (ResourceMap::value_type &resource_pair, resource_map_) {
      resource_pair.second.reset();
    }
  }

  bool reset(const std::string &name) {
    const ResourceMap::iterator handle(resource_map_.find(name));
    if (handle == resource_map_.end()) {
      return false;
    }
    handle->second.reset();
    return true;
  }
};

class PositionJointSoftLimitsInterface
    : public JointLimitsInterface< PositionJointSoftLimitsHandle > {
public:
  virtual ~PositionJointSoftLimitsInterface() {}

  void reset() {
    BOOST_FOREACH (ResourceMap::value_type &resource_pair, resource_map_) {
      resource_pair.second.reset();
    }
  }

  bool reset(const std::string &name) {
    const ResourceMap::iterator handle(resource_map_.find(name));
    if (handle == resource_map_.end()) {
      return false;
    }
    handle->second.reset();
    return true;
  }
};

class VelocityJointSaturationInterface
    : public JointLimitsInterface< VelocityJointSaturationHandle > {
public:
  virtual ~VelocityJointSaturationInterface() {}
};

class VelocityJointSoftLimitsInterface
    : public JointLimitsInterface< VelocityJointSoftLimitsHandle > {
public:
  virtual ~VelocityJointSoftLimitsInterface() {}
};

class EffortJointSaturationInterface : public JointLimitsInterface< EffortJointSaturationHandle > {
public:
  virtual ~EffortJointSaturationInterface() {}
};

class EffortJointSoftLimitsInterface : public JointLimitsInterface< EffortJointSoftLimitsHandle > {
public:
  virtual ~EffortJointSoftLimitsInterface() {}
};

} // namespace dynamic_joint_limits_interface

#endif