#include <cmath>
#include <string>

#include <controller_manager/controller_manager.h>
#include <dynamic_joint_limits_interface/joint_limits_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/time.h>

namespace djli = dynamic_joint_limits_interface;
namespace hi = hardware_interface;

//
// fake joints in fake hardware
//

class FakeJointBase {
public:
  FakeJointBase(const std::string &name) : name_(name), pos_(0.), vel_(0.), eff_(0.), cmd_(0.) {}
  virtual ~FakeJointBase() {}

  template < class CommandInterface, class LimitsInterface >
  void registerHandles(hi::JointStateInterface &state_iface, CommandInterface &cmd_iface,
                       LimitsInterface &lim_iface) {
    const hi::JointStateHandle state_handle(name_, &pos_, &vel_, &eff_);
    state_iface.registerHandle(state_handle);
    const hi::JointHandle cmd_handle(state_handle, &cmd_);
    cmd_iface.registerHandle(cmd_handle);
    const typename LimitsInterface::Handle lim_handle(cmd_handle);
    lim_iface.registerHandle(lim_handle);
  }

  virtual void update(const ros::Duration &period) = 0;

protected:
  const std::string name_;
  double pos_, vel_, eff_, cmd_;
};

// position-controlled
class PositionFakeJoint : public FakeJointBase {
public:
  PositionFakeJoint(const std::string &name) : FakeJointBase(name) {}
  virtual ~PositionFakeJoint() {}

  virtual void update(const ros::Duration &period) {
    const double prev_pos(pos_);
    if (!std::isnan(cmd_)) {
      pos_ = cmd_;
    }
    vel_ = (cmd_ - pos_) / period.toSec();
    // TODO: update eff
  }
};

// velocity-controlled
class VelocityFakeJoint : public FakeJointBase {
public:
  VelocityFakeJoint(const std::string &name) : FakeJointBase(name) {}
  virtual ~VelocityFakeJoint() {}

  virtual void update(const ros::Duration &period) {
    pos_ += vel_ * period.toSec();
    if (!std::isnan(cmd_)) {
      vel_ = cmd_;
    }
    // TODO: update eff
  }
};

// effort-controlled
class EffortFakeJoint : public FakeJointBase {
public:
  EffortFakeJoint(const std::string &name) : FakeJointBase(name) {}
  virtual ~EffortFakeJoint() {}

  virtual void update(const ros::Duration &period) {
    const double dt(period.toSec());
    pos_ += vel_ * dt;
    const double M(1.), D(1.); // assuming mass & damper system
    vel_ += (eff_ - D * vel_) / M * dt; // assuming effort is torque
    if (!std::isnan(cmd_)) {
      eff_ = cmd_;
    }
  }
};

//
// fake hardware containing fake joints (3 types) and supporting dynamic joint limits (2 types)
//

class FakeHW : public hardware_interface::RobotHW {
public:
  FakeHW()
      : pos_joint_("pos_joint"), pos_soft_joint_("pos_soft_joint"), vel_joint_("vel_joint"),
        vel_soft_joint_("vel_soft_joint"), eff_joint_("eff_joint"),
        eff_soft_joint_("eff_soft_joint") {}

  virtual ~FakeHW() {}

  virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh) {
    root_nh_ = root_nh;

    // register state/command interfaces to allow controllers to access joints' state/commands
    registerInterface(&state_iface_);
    registerInterface(&pos_iface_);
    registerInterface(&vel_iface_);
    registerInterface(&eff_iface_);

    // register joints' state/command/limits handles to corresponding interfaces
    pos_joint_.registerHandles(state_iface_, pos_iface_, pos_sat_iface_);
    pos_soft_joint_.registerHandles(state_iface_, pos_iface_, pos_soft_lim_iface_);
    vel_joint_.registerHandles(state_iface_, vel_iface_, vel_sat_iface_);
    vel_soft_joint_.registerHandles(state_iface_, vel_iface_, vel_soft_lim_iface_);
    eff_joint_.registerHandles(state_iface_, eff_iface_, eff_sat_iface_);
    eff_soft_joint_.registerHandles(state_iface_, eff_iface_, eff_soft_lim_iface_);

    // do first update of limits which will perform blocking access to the paremeter server
    pos_sat_iface_.updateLimits(root_nh_);
    pos_soft_lim_iface_.updateLimits(root_nh_);
    vel_sat_iface_.updateLimits(root_nh_);
    vel_soft_lim_iface_.updateLimits(root_nh_);
    eff_sat_iface_.updateLimits(root_nh_);
    eff_soft_lim_iface_.updateLimits(root_nh_);

    return true;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {}

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // update joint limits and states
    updateJoint(pos_sat_iface_, pos_joint_, period);
    updateJoint(pos_soft_lim_iface_, pos_soft_joint_, period);
    updateJoint(vel_sat_iface_, vel_joint_, period);
    updateJoint(vel_soft_lim_iface_, vel_soft_joint_, period);
    updateJoint(eff_sat_iface_, eff_joint_, period);
    updateJoint(eff_soft_lim_iface_, eff_soft_joint_, period);
  }

private:
  template < class LimitsInterface >
  void updateJoint(LimitsInterface &lim_iface, FakeJointBase &joint, const ros::Duration &period) {
    // update limits with cached parameters subscribed in background
    lim_iface.updateLimits(root_nh_);
    // apply limits to commands
    lim_iface.enforceLimits(period);
    // update joint states according to commands
    joint.update(period);
  }

private:
  ros::NodeHandle root_nh_;

  hi::JointStateInterface state_iface_;

  hi::PositionJointInterface pos_iface_;
  hi::VelocityJointInterface vel_iface_;
  hi::EffortJointInterface eff_iface_;

  djli::PositionJointSaturationInterface pos_sat_iface_;
  djli::PositionJointSoftLimitsInterface pos_soft_lim_iface_;
  djli::VelocityJointSaturationInterface vel_sat_iface_;
  djli::VelocityJointSoftLimitsInterface vel_soft_lim_iface_;
  djli::EffortJointSaturationInterface eff_sat_iface_;
  djli::EffortJointSoftLimitsInterface eff_soft_lim_iface_;

  PositionFakeJoint pos_joint_, pos_soft_joint_;
  VelocityFakeJoint vel_joint_, vel_soft_joint_;
  EffortFakeJoint eff_joint_, eff_soft_joint_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "fake_hardware");
  ros::NodeHandle nh, pnh("~");

  FakeHW fake_hw;
  if (!fake_hw.init(nh, pnh)) {
    ROS_FATAL("Failed to init hardware");
    return 1;
  }

  controller_manager::ControllerManager controllers(&fake_hw);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(20.);
  ros::Time last(ros::Time::now());
  while (ros::ok()) {
    const ros::Time now(ros::Time::now());
    const ros::Duration period(now - last);
    fake_hw.read(now, period);
    controllers.update(now, period);
    fake_hw.write(now, period);
    last = now;
    rate.sleep();
  }

  return 0;
}