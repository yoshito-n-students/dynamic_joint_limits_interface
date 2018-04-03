#include <dynamic_joint_limits_interface/dynamic_joint_limits_interface.h>
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
  FakeHW(const ros::NodeHandle &limits_nh) : limits_nh_(limits_nh), pos_(0.), vel_(0.), eff_(0.) {}

  virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh) {
    // register command interfaces to accept commands from controllers
    registerInterface(&pos_iface_);
    registerInterface(&vel_iface_);
    registerInterface(&eff_iface_);

    // register command handles so that controllers can find the joint
    const hi::JointStateHandle state_handle("joint", &pos_, &vel_, &eff_);
    const hi::JointHandle pos_handle(state_handle, &pos_cmd_);
    pos_iface_.registerHandle(pos_handle);
    const hi::JointHandle vel_handle(state_handle, &vel_cmd_);
    vel_iface_.registerHandle(vel_handle);
    const hi::JointHandle eff_handle(state_handle, &eff_cmd_);
    eff_iface_.registerHandle(eff_handle);

    // register limits handles to enable limits
    pos_sat_iface_.registerHandle(djli::DynamicPositionJointSaturationHandle(pos_handle));
    vel_sat_iface_.registerHandle(djli::DynamicVelocityJointSaturationHandle(vel_handle));
    eff_sat_iface_.registerHandle(djli::DynamicEffortJointSaturationHandle(eff_handle));

    return true;
  }

  virtual void write(const ros::Time &time, const ros::Duration &period) {
    pos_sat_iface_.updateLimits(limits_nh_);
    vel_sat_iface_.updateLimits(limits_nh_);
    eff_sat_iface_.updateLimits(limits_nh_);

    pos_sat_iface_.enforceLimits(period);
    vel_sat_iface_.enforceLimits(period);
    eff_sat_iface_.enforceLimits(period);

    ROS_INFO_STREAM("pos_cmd: " << pos_cmd_);
    ROS_INFO_STREAM("vel_cmd: " << vel_cmd_);
    ROS_INFO_STREAM("eff_cmd: " << eff_cmd_);
  }

private:
  ros::NodeHandle limits_nh_;

  hi::PositionJointInterface pos_iface_;
  hi::VelocityJointInterface vel_iface_;
  hi::EffortJointInterface eff_iface_;

  djli::DynamicPositionJointSaturationInterface pos_sat_iface_;
  djli::DynamicVelocityJointSaturationInterface vel_sat_iface_;
  djli::DynamicEffortJointSaturationInterface eff_sat_iface_;

  double pos_, vel_, eff_;
  double pos_cmd_, vel_cmd_, eff_cmd_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "fake_hardware");
  ros::NodeHandle nh;

  FakeHW fake_hw(nh);

  ros::Rate rate(20.);
  ros::Time last(ros::Time::now());
  while (ros::ok()) {
    const ros::Time now(ros::Time::now());
    const ros::Duration period(now - last);
    fake_hw.write(now, period);
    last = now;
    rate.sleep();
  }

  return 0;
}