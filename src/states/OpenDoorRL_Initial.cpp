#include "OpenDoorRL_Initial.h"

#include "../OpenDoorRL.h"

void OpenDoorRL_Initial::configure(const mc_rtc::Configuration & config)
{
}

void OpenDoorRL_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  //sync init pose
  sva::PTransformd default_pose;
  {
    const auto & da = ctl.robot().module().default_attitude();
    default_pose.translation() << da[4], da[5], da[6];
    default_pose.rotation() = Eigen::Quaterniond{da[0], da[1], da[2], da[3]}.toRotationMatrix();
  }
  if(ctl.robot().posW() == default_pose)
  {
    mc_rtc::Configuration init_pose_cfg(static_cast<std::string>(ctl.config()("ETC_DIR")) + "/initial_pose.yaml");
    ctl.robot().posW(init_pose_cfg("initial_pose"));
    mc_rtc::log::success("Check if posW is in sync");
  }
}

bool OpenDoorRL_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  output("OK");
  return true;
}

void OpenDoorRL_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
}

EXPORT_SINGLE_STATE("OpenDoorRL_Initial", OpenDoorRL_Initial)
