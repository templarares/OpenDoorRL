#include "RLMeta.h"
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_solver/GenericLoader.h>
#include <mc_control/fsm/Controller.h>

namespace mc_control
{

namespace fsm
{

void RLMetaState::configure(const mc_rtc::Configuration & config)
{
    MetaState::configure(config);
}

void RLMetaState::start(Controller & ctl)
{
  MetaState::start(ctl);
  //store RL related callbacks prefixed with state's name
  ctl.datastore().make_call(name()+"::nextState",[this]()->bool{this->executor_.next();});
  ////mc_rtc::log::info("adding ready and next state call back with prefix {}::",name());
  ctl.datastore().make_call(name()+"::ready",[this]()->bool{this->executor_.ready();});
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

bool RLMetaState::run(Controller & ctl)
{
  bool done = MetaState::run(ctl);
  if (done){
    ctl.datastore().assign("CurrentState",name());
  }
  return done;
}

void RLMetaState::teardown(Controller & ctl)
{
  ctl.datastore().assign("StateDone",true);
  ctl.datastore().assign("CurrentState",name());
  MetaState::teardown(ctl);
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("RLMeta", mc_control::fsm::RLMetaState)
