#include "OpenDoorRL_Initial.h"

#include "../OpenDoorRL.h"

void OpenDoorRL_Initial::configure(const mc_rtc::Configuration & config)
{
}

void OpenDoorRL_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
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
