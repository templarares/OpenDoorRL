#include "Teleport.h"

#include "../OpenDoorRL.h"

void Teleport::configure(const mc_rtc::Configuration & config)
{
  if (config.has("dx"))
  {
    dx=config("dx");
    // mc_rtc::log::success("teleport dx is {}",dx);
  }
  if (config.has("dy"))
  {
    dy=config("dy");
  }
  if (config.has("dyaw"))
  {
    dyaw=config("dyaw");
  }
  if (config.has("steps"))
  {
    steps=config("steps");
  }
}

void Teleport::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  
  
  
}

bool Teleport::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  if (counter<steps*100)
  {
    if (counter%100==0)
    {
      sva::PTransformd deviation=sva::PTransformd(sva::RotZ(this->dyaw/steps), Eigen::Vector3d(this->dx/steps,this->dy/steps, 0));
      ctl.robot().posW(ctl.robot().posW()*deviation);
    }
    

    counter++;
    return false;
  }
  else
  {
    output("OK");
    return true;
  }
}

void Teleport::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
}

EXPORT_SINGLE_STATE("Teleport", Teleport)