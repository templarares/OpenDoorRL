#include "Teleport.h"

#include "../OpenDoorRL.h"

void Teleport::configure(const mc_rtc::Configuration & config)
{
}

void Teleport::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  ctl.robot().posW(ctl.robot().posW()*sva::PTransformd(sva::RotZ(0.4)));
}

bool Teleport::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  return true;
}

void Teleport::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
}

EXPORT_SINGLE_STATE("Teleport", Teleport)