#include "Initial.h"

#include "../OpenDoorRL.h"

static sva::PTransformd LoadInitialPose(mc_control::fsm::Controller & ctl)
{
  std::string etc_dir = ctl.config()("ETC_DIR");
  pid_t pid = ::getpid();  
  mc_rtc::Configuration initial(etc_dir + "/initial_pose"+".yaml");
  if (initial.has("initial_pose_rand"))
  {
    return initial("initial_pose_rand", ctl.robot().posW());
  }
  else if (initial.has("initial_pose"))
  {
    return initial("initial_pose", ctl.robot().posW());
  }
  
}

static void SetInitialPose(mc_control::fsm::Controller & ctl, const sva::PTransformd & pose)
{
  ctl.robot().posW(pose);
  ctl.removeContact({"bit_humanoid", "ground", "LeftFoot", "AllGround"});
  ctl.removeContact({"bit_humanoid", "ground", "RightFoot", "AllGround"});
  ctl.addContact({"bit_humanoid", "ground", "LeftFoot", "AllGround"});
  ctl.addContact({"bit_humanoid", "ground", "RightFoot", "AllGround"});
  //SaveInitialPose(ctl);
}


void Initial::configure(const mc_rtc::Configuration & config)
{
}

void Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  auto initial_pose_ = LoadInitialPose(ctl);
  sva::PTransformd default_pose;
  {
    const auto & da = ctl.robot().module().default_attitude();
    default_pose.translation() << da[4], da[5], da[6];
    default_pose.rotation() = Eigen::Quaterniond{da[0], da[1], da[2], da[3]}.toRotationMatrix();
  }
  if(ctl.robot().posW() == default_pose)
  {
    SetInitialPose(ctl, initial_pose_);
  }
  // mc_solver::DynamicsConstraint dynamicsConstraint_;
  // ctl.solver().addConstraintSet(dynamicsConstraint_);
  // // set weight and stiffness of the door's posture task to be very high, otherwise collision avoidance wont work properly
  // auto pt = ctl.getPostureTask("door");
  // pt->stiffness(88.0);
  // pt->weight(10000.0);
}

bool Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  iteration_counter_++;
  //add a timeout completion criteria
  if (iteration_counter_>1500){
    output("OK");
  }
  return output().size() != 0;
}

void Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<OpenDoorRL &>(ctl_);
  // sva::PTransformd realPosW(ctl.robot().bodySensor("FloatingBase").orientation(),ctl.robot().bodySensor("FloatingBase").position());
  // ctl.robot().posW(realPosW);
  if (ctl.datastore().has("StateDone")){
    ctl.datastore().assign("StateDone",true);
  }
  else
  {
    ctl.datastore().make<bool>("StateDone",true);
  }
  
}

EXPORT_SINGLE_STATE("Initial", Initial)
