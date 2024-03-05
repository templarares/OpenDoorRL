#include "Rewind.h"

#include "../OpenDoorRL.h"

void Rewind::configure(const mc_rtc::Configuration & config)
{
}

void Rewind::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  auto pt = ctl.getPostureTask("bit_humanoid");
  //remove entry "Root" from the default stance to prevent error
  auto default_stance=ctl.robot().module().stance();
  default_stance.erase("Root");
  pt->target(default_stance);
}

bool Rewind::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  auto pt = ctl.getPostureTask("bit_humanoid");
  if (pt->speed().norm()<0.001 || pt->eval().norm()<0.05)
  {
    //q()[0] is Root node, which is empty for the fixed BITDoor robot
    if (ctl.realRobots().robots()[1].q()[1][0]>0.3)
    {
        mc_rtc::log::success("door opened at {}",ctl.realRobots().robots()[1].q()[1][0]);
        output("DoorOpened");
    }
    else
    {
        output("OK");
    }    
    return true;
  }
  return false;
}

void Rewind::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
}

EXPORT_SINGLE_STATE("OpenDoorRLFSM::Rewind", Rewind)