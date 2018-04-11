#include <cmath>

#include <controller_manager/controller_manager.h>
#include <dynamic_joint_limits_interface/joint_limits_interface.h>
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

class FakeHW : public hardware_interface::RobotHW {
public:
  FakeHW() : pos_(0.), vel_(0.), eff_(0.), pos_cmd_(0.), vel_cmd_(0.), eff_cmd_(0.) {}

  virtual ~FakeHW() {}

  virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh) {
    root_nh_ = root_nh;

    // register command interfaces to accept commands from controllers
    registerInterface(&state_iface_);
    registerInterface(&pos_iface_);
    registerInterface(&vel_iface_);
    registerInterface(&eff_iface_);

    // register command handles so that controllers can find the joint
    const hi::JointStateHandle state_handle("joint", &pos_, &vel_, &eff_);
    state_iface_.registerHandle(state_handle);
    const hi::JointHandle pos_handle(state_handle, &pos_cmd_);
    pos_iface_.registerHandle(pos_handle);
    const hi::JointHandle vel_handle(state_handle, &vel_cmd_);
    vel_iface_.registerHandle(vel_handle);
    const hi::JointHandle eff_handle(state_handle, &eff_cmd_);
    eff_iface_.registerHandle(eff_handle);

    // register limits handles to enable limits
    pos_sat_iface_.registerHandle(djli::PositionJointSaturationHandle(pos_handle));
    vel_sat_iface_.registerHandle(djli::VelocityJointSaturationHandle(vel_handle));
    eff_sat_iface_.registerHandle(djli::EffortJointSaturationHandle(eff_handle));

    // do first update which will perform blocking access to the paremeter server
    pos_sat_iface_.updateLimits(root_nh_);
    vel_sat_iface_.updateLimits(root_nh_);
    eff_sat_iface_.updateLimits(root_nh_);

    return true;
  }

  virtual void read(const ros::Time &time, const ros::Duration &period) {
    // TODO: couple pos & vel & eff
    if (!std::isnan(pos_cmd_)) {
      pos_ = pos_cmd_;
    }
    if (!std::isnan(vel_cmd_)) {
      vel_ = vel_cmd_;
    }
    if (!std::isnan(eff_cmd_)) {
      eff_ = eff_cmd_;
    }
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    // update limits with cached parameters subscribed in background
    pos_sat_iface_.updateLimits(root_nh_);
    vel_sat_iface_.updateLimits(root_nh_);
    eff_sat_iface_.updateLimits(root_nh_);

    // apply limits to commands
    pos_sat_iface_.enforceLimits(period);
    vel_sat_iface_.enforceLimits(period);
    eff_sat_iface_.enforceLimits(period);
  }

private:
  ros::NodeHandle root_nh_;

  hi::JointStateInterface state_iface_;

  hi::PositionJointInterface pos_iface_;
  hi::VelocityJointInterface vel_iface_;
  hi::EffortJointInterface eff_iface_;

  djli::PositionJointSaturationInterface pos_sat_iface_;
  djli::VelocityJointSaturationInterface vel_sat_iface_;
  djli::EffortJointSaturationInterface eff_sat_iface_;

  double pos_, vel_, eff_;
  double pos_cmd_, vel_cmd_, eff_cmd_;
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