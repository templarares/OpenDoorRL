#include "RLParallel.h"
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_solver/GenericLoader.h>
#include <mc_control/fsm/Controller.h>

namespace mc_control
{

namespace fsm
{


void RLParallelState::configure(const mc_rtc::Configuration & config)
{
    ParallelState::configure(config);
}

void RLParallelState::start(Controller & ctl)
{
  ParallelState::start(ctl);
  //store RL related callbacks prefixed with state's name
  if (ctl.datastore().has("StateDone")){
    ctl.datastore().assign("StateDone",false);
  }
  else
  {
    ctl.datastore().make<bool>("StateDone",false);
  }
  if (ctl.datastore().has("CurrentState")){
    ctl.datastore().assign("CurrentState",name());
  }
  else
  {
    ctl.datastore().make<std::string>("CurrentState","Initial");
  }

}

bool RLParallelState::run(Controller & ctl)
{
  bool done = ParallelState::run(ctl);
  if (done) {
    ctl.datastore().assign("CurrentState",name());
  }
  return done;
}

void RLParallelState::teardown(Controller & ctl)
{
  ctl.datastore().assign("StateDone",true);
  ctl.datastore().assign("CurrentState",name());
  ParallelState::teardown(ctl);
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("RLParallel", mc_control::fsm::RLParallelState)
