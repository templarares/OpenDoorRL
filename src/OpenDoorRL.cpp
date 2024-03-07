#include "OpenDoorRL.h"

OpenDoorRL::OpenDoorRL(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  mc_rtc::Configuration init_pose_cfg(static_cast<std::string>(config("ETC_DIR")) + "/initial_pose.yaml");
  robot().posW(init_pose_cfg("initial_pose"));

  /* Callback for when an RL interface state just completed */
  datastore().make_call("RLInterface::done", [](const std::string &, mc_control::fsm::Controller &) {});
  /** Callback for when an RL interface state is about to start */
  datastore().make_call("RLInterface::start", [](const std::string &, mc_control::fsm::Controller &) -> mc_rtc::Configuration { return {}; });
  datastore().make_call("nextState",[this]()->bool{this->executor_.next();});
  ////mc_rtc::log::success("BITCarInOut init done ");
  datastore().make_call("ready",[this]()->bool{this->executor_.ready();});
  ////mc_rtc::log::info("At initialization, current state is {}",this->executor_.state());
  //suppress or un-suppress terminal logging based on config key value
  bool terminalLog=false;
  if (config.has("TerminalLog"))
  {
    config("TerminalLog",terminalLog);
  }
  
  if (!terminalLog)
  {
    mc_rtc::log::details::success().set_level(spdlog::level::off);
    mc_rtc::log::details::info().set_level(spdlog::level::off);
    mc_rtc::log::details::cerr().set_level(spdlog::level::off);
  }
  else
  {
    mc_rtc::log::details::success().set_level(spdlog::level::debug);
    mc_rtc::log::details::info().set_level(spdlog::level::debug);
    mc_rtc::log::details::cerr().set_level(spdlog::level::debug);

  }
  srand(time(0));
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    bool has_left_foot = false;
    bool has_right_foot = false;
    for(const auto & c : solver().contacts())
    {
      if(c.r1Surface()->name() == "LeftFoot")
      {
        has_left_foot = true;
      }
      if(c.r1Surface()->name() == "RightFoot")
      {
        has_right_foot = true;
      }
    }
    double ratio = 0.5;
    if(has_left_foot && !has_right_foot)
    {
      ratio = 1.0;
    }
    if(has_right_foot && !has_left_foot)
    {
      ratio = 0.0;
    }
    return sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), ratio);
  });

  // //use pid to deal with multi-occupation of the intitial pose file
  // pid_t pid = ::getpid();
  // mc_rtc::Configuration init_pose_cfg(static_cast<std::string>(config("ETC_DIR")) + "/initial_pose"+std::to_string(pid)+".yaml");
  // if(init_pose_cfg.has("initial_pose_rand"))
  // {
  //   robot().posW(init_pose_cfg("initial_pose_rand"));
  // }
  // else if(init_pose_cfg.has("initial_pose"))
  // {
  //   robot().posW(init_pose_cfg("initial_pose"));
  // }


  mc_rtc::log::success("OpenDoorRL init done ");
}

bool OpenDoorRL::run()
{
  return mc_control::fsm::Controller::run();
}

void OpenDoorRL::reset(const mc_control::ControllerResetData & reset_data)
{
  auto posW = robot().posW();
  Eigen::Quaterniond q(posW.rotation());
  mc_rtc::log::info("Initial orientation: {}, {}, {}, {}", q.w(), q.x(), q.y(), q.z());
  mc_rtc::log::info("Initial translation: {}", posW.translation().transpose());
  mc_control::fsm::Controller::reset(reset_data);
}


