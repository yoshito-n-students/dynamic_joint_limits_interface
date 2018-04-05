#ifndef DYNAMIC_JOINT_LIMITS_INTERFACE_DYNAMIC_JOINT_LIMITS_INTERFACE_H
#define DYNAMIC_JOINT_LIMITS_INTERFACE_DYNAMIC_JOINT_LIMITS_INTERFACE_H

#include <limits>
#include <string>

#include <dynamic_joint_limits_interface/joint_limits_rosparam_cached.h>
#include <hardware_interface/internal/resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <ros/duration.h>
#include <ros/node_handle.h>

#include <boost/algorithm/clamp.hpp>
#include <boost/foreach.hpp>

namespace dynamic_joint_limits_interface {

// they are almost equivarents to joint_limits_interface::XXXJointSaturationHandle
// but can update limits after construction by updateLimits()

class DynamicPositionJointSaturationHandle {
public:
  DynamicPositionJointSaturationHandle(
      const hardware_interface::JointHandle &jh,
      const joint_limits_interface::JointLimits &limits = joint_limits_interface::JointLimits())
      : jh_(jh), limits_(limits), prev_cmd_(std::numeric_limits< double >::quiet_NaN()) {
    // never throw dislike joint_limits_interface::PositionJointSaturationHandle
    // according to the initial limits because it can be updated by updateLimits()
  }

  virtual ~DynamicPositionJointSaturationHandle() {}

  std::string getName() const { return jh_.getName(); }

  void enforceLimits(const ros::Duration &period) {
    if (std::isnan(prev_cmd_)) {
      prev_cmd_ = jh_.getPosition();
    }

    double min_pos(std::numeric_limits< double >::quiet_NaN());
    double max_pos(std::numeric_limits< double >::quiet_NaN());
    if (limits_.has_position_limits && limits_.has_velocity_limits) {
      // position & velocity limits available
      const double delta_pos(limits_.max_velocity * period.toSec());
      min_pos = std::max(prev_cmd_ - delta_pos, limits_.min_position);
      max_pos = std::min(prev_cmd_ + delta_pos, limits_.max_position);
    } else if (limits_.has_position_limits && !limits_.has_velocity_limits) {
      // position limits only
      min_pos = limits_.min_position;
      max_pos = limits_.max_position;
    } else if (!limits_.has_position_limits && limits_.has_velocity_limits) {
      // velocity limits only
      const double delta_pos(limits_.max_velocity * period.toSec());
      min_pos = prev_cmd_ - delta_pos;
      max_pos = prev_cmd_ + delta_pos;
    }

    double cmd;
    if (!std::isnan(min_pos) && !std::isnan(max_pos)) {
      cmd = boost::algorithm::clamp(jh_.getCommand(), min_pos, max_pos);
    } else {
      cmd = jh_.getCommand();
    }
    jh_.setCommand(cmd);
    prev_cmd_ = cmd;
  }

  void updateLimits(ros::NodeHandle &nh) { getJointLimitsCached(getName(), nh, limits_); }

  void reset() { prev_cmd_ = std::numeric_limits< double >::quiet_NaN(); }

private:
  hardware_interface::JointHandle jh_;
  joint_limits_interface::JointLimits limits_;
  double prev_cmd_;
};

class DynamicVelocityJointSaturationHandle {
public:
  DynamicVelocityJointSaturationHandle(
      const hardware_interface::JointHandle &jh,
      const joint_limits_interface::JointLimits &limits = joint_limits_interface::JointLimits())
      : jh_(jh), limits_(limits) {
    // never throw dislike joint_limits_interface::VelocityJointSaturationHandle
    // according to the initial limits because it can be updated by updateLimits()
  }

  virtual ~DynamicVelocityJointSaturationHandle() {}

  std::string getName() const { return jh_.getName(); }

  void enforceLimits(const ros::Duration &period) {
    double min_vel(std::numeric_limits< double >::quiet_NaN());
    double max_vel(std::numeric_limits< double >::quiet_NaN());
    if (limits_.has_velocity_limits && limits_.has_acceleration_limits) {
      // velocity&acceleration limits available
      const double vel(jh_.getVelocity());
      const double delta_vel(limits_.max_acceleration * period.toSec());
      min_vel = std::max(vel - delta_vel, -limits_.max_velocity);
      max_vel = std::min(vel + delta_vel, limits_.max_velocity);
    } else if (limits_.has_velocity_limits && !limits_.has_acceleration_limits) {
      // velocity limits only
      min_vel = -limits_.max_velocity;
      max_vel = limits_.max_velocity;
    } else if (!limits_.has_velocity_limits && limits_.has_acceleration_limits) {
      // acceleration limits only
      const double vel(jh_.getVelocity());
      const double delta_vel(limits_.max_acceleration * period.toSec());
      min_vel = vel - delta_vel;
      max_vel = vel + delta_vel;
    }

    double cmd;
    if (!isnan(min_vel) && !isnan(max_vel)) {
      cmd = boost::algorithm::clamp(jh_.getCommand(), min_vel, max_vel);
    } else {
      cmd = jh_.getCommand();
    }
    jh_.setCommand(cmd);
  }

  void updateLimits(ros::NodeHandle &nh) { getJointLimitsCached(getName(), nh, limits_); }

private:
  hardware_interface::JointHandle jh_;
  joint_limits_interface::JointLimits limits_;
};

class DynamicEffortJointSaturationHandle {
public:
  DynamicEffortJointSaturationHandle(
      const hardware_interface::JointHandle &jh,
      const joint_limits_interface::JointLimits &limits = joint_limits_interface::JointLimits())
      : jh_(jh), limits_(limits) {
    // never throw dislike joint_limits_interface::EffortJointSaturationHandle
    // according to the initial limits because it can be updated by updateLimits()
  }

  virtual ~DynamicEffortJointSaturationHandle() {}

  std::string getName() const { return jh_.getName(); }

  void enforceLimits(const ros::Duration & /* period */) {
    double min_eff(std::numeric_limits< double >::quiet_NaN());
    double max_eff(std::numeric_limits< double >::quiet_NaN());
    // use effort limits if available
    if (limits_.has_effort_limits) {
      min_eff = -limits_.max_effort;
      max_eff = limits_.max_effort;
    }
    // clip effort limits if position is out of range
    if (limits_.has_position_limits) {
      const double pos(jh_.getPosition());
      if (pos < limits_.min_position) {
        min_eff = 0;
      } else if (pos > limits_.max_position) {
        max_eff = 0;
      }
    }
    // clip effort limits if velocity is out of range
    if (limits_.has_velocity_limits) {
      const double vel(jh_.getVelocity());
      if (vel < -limits_.max_velocity) {
        min_eff = 0;
      } else if (vel > limits_.max_velocity) {
        max_eff = 0;
      }
    }

    double cmd;
    if (!std::isnan(min_eff) && !std::isnan(max_eff)) {
      // lower & upper limits available
      cmd = boost::algorithm::clamp(jh_.getCommand(), min_eff, max_eff);
    } else if (!std::isnan(min_eff) && std::isnan(max_eff)) {
      // lower limit only
      cmd = std::max(jh_.getCommand(), min_eff);
    } else if (std::isnan(min_eff) && !std::isnan(max_eff)) {
      // upper limit only
      cmd = std::min(jh_.getCommand(), max_eff);
    } else {
      // no limits
      cmd = jh_.getCommand();
    }
    jh_.setCommand(cmd);
  }

  void updateLimits(ros::NodeHandle &nh) { getJointLimitsCached(getName(), nh, limits_); }

private:
  hardware_interface::JointHandle jh_;
  joint_limits_interface::JointLimits limits_;
};

// this is almost an equivarent to joint_limits_interface::JointLimitsInterface
// but has updateLimits() which updates limits in managed handles

template < class LimitsHandle >
class DynamicJointLimitsInterface : public hardware_interface::ResourceManager< LimitsHandle > {
private:
  typedef typename hardware_interface::ResourceManager< LimitsHandle > Base;

public:
  virtual ~DynamicJointLimitsInterface() {}

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

// they are almost equivarents to joint_limits_interface::XXXJointSaturationInterface
// but inherits DynamicXXXJointSaturationInterface

class DynamicPositionJointSaturationInterface
    : public DynamicJointLimitsInterface< DynamicPositionJointSaturationHandle > {
public:
  virtual ~DynamicPositionJointSaturationInterface() {}

  void reset() {
    BOOST_FOREACH (ResourceMap::value_type &resource_pair, resource_map_) {
      resource_pair.second.reset();
    }
  }

  bool reset(const std::string &name) {
    ResourceMap::iterator handle(resource_map_.find(name));
    if (handle == resource_map_.end()) {
      return false;
    }
    handle->second.reset();
    return true;
  }
};

class DynamicVelocityJointSaturationInterface
    : public DynamicJointLimitsInterface< DynamicVelocityJointSaturationHandle > {
public:
  virtual ~DynamicVelocityJointSaturationInterface() {}
};

class DynamicEffortJointSaturationInterface
    : public DynamicJointLimitsInterface< DynamicEffortJointSaturationHandle > {
public:
  virtual ~DynamicEffortJointSaturationInterface() {}
};

} // namespace dynamic_joint_limits_interface

#endif