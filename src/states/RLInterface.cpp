#include "RLInterface.h"
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_solver/GenericLoader.h>
#include <mc_control/fsm/Controller.h>

namespace mc_control
{

namespace fsm
{

void RLInterfaceState::configure(const mc_rtc::Configuration & config)
{
  if(!started_)
  {
    actualConfig_.load(config);
    // Clear all configuration entries set in configure_
    add_contacts_config_ = {};
    remove_contacts_config_ = {};
    add_contacts_after_config_ = {};
    remove_contacts_after_config_ = {};
    add_collisions_config_ = {};
    remove_collisions_config_ = {};
    add_collisions_after_config_ = {};
    remove_collisions_after_config_ = {};
    // constraints_config_ = {};
    tasks_config_ = {};
    remove_posture_task_ = {};
  }
  else
  {
    MetaTasksState::configure(config);
  }
}

void RLInterfaceState::start(Controller & ctl)
{
  MetaTasksState::start(ctl);
  if(!started_)
  {
    started_ = true;
    auto config = ctl.datastore().call<mc_rtc::Configuration>("RLInterface::start", name(), ctl);
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
    mc_rtc::log::info("Config from callback:\n{}", config.dump(true, true));
    actualConfig_.load(config);
    // Note: we call the non virtual interface to handle all entries
    this->configure_(actualConfig_);
    // // set weight and stiffness of the door's posture task to be very high, otherwise collision avoidance wont work properly
    // start_(ctl);
    // auto pt = ctl.getPostureTask("door");
    // pt->stiffness(88.0);
    // pt->weight(10000.0);
  }
}

bool RLInterfaceState::run(Controller & ctl)
{
  iterationCounter_++;
  auto pt = ctl.getPostureTask("door");
  pt->stiffness(88.0);
  pt->weight(10000.0);
  return MetaTasksState::run(ctl);
}

void RLInterfaceState::teardown(Controller & ctl)
{
  ctl.datastore().call("RLInterface::done", name(), ctl);
  sva::PTransformd realPosW(ctl.robot().bodySensor("FloatingBase").orientation(),ctl.robot().bodySensor("FloatingBase").position());
  ctl.robot().posW(realPosW);
  mc_control::fsm::ContactSet tempContacts;
  double duration = iterationCounter_*ctl.timeStep;
  if (ctl.datastore().has("simDuration")){
    ctl.datastore().assign("simDuration",duration);
  }
  else
  {
    ctl.datastore().make<double>("simDuration",duration);
  }    
  ctl.datastore().call("RLInterface::done", name(), ctl);
  ctl.datastore().assign("StateDone",true);
  // for (auto contact : ctl.contacts()){
  //   tempContacts.emplace(contact);
  //   ctl.removeContact(contact);
  // }
  // for (auto contact : tempContacts){
  //   ctl.addContact(contact);
  // }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("RLInterface", mc_control::fsm::RLInterfaceState)
