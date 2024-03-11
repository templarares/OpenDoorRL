#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_global_controller.h>

#include <mc_mujoco/mj_sim.h>

namespace py = pybind11;

// The Python callbacks uses a pointer as a work-around for https://github.com/pybind/pybind11/issues/1241
using done_cb = std::function<void(const std::string &, mc_control::fsm::Controller &)>;
using python_done_cb = std::function<void(const std::string &, mc_control::fsm::Controller *)>;
using start_cb = std::function<mc_rtc::Configuration(const std::string &, mc_control::fsm::Controller &)>;
using python_start_cb = std::function<mc_rtc::Configuration(const std::string &, mc_control::fsm::Controller *)>;

static std::unique_ptr<mc_mujoco::MjSim> make_mj_sim(const std::string & config = "", bool with_visualization = false)
{
  mc_mujoco::MjConfiguration mjc;
  mjc.mc_config = config;
  mjc.sync_real_time = false;
  mjc.with_visualization = with_visualization;
  return std::make_unique<mc_mujoco::MjSim>(mjc);
}

namespace mc_rtc
{

template<>
struct ConfigurationLoader<py::handle>
{
  static mc_rtc::Configuration save(const py::handle & obj)
  {
#define TRY_CONVERT(PYT, CPPT)         \
  if(py::isinstance<PYT>(obj))         \
  {                                    \
    mc_rtc::Configuration out;         \
    out.add("data", obj.cast<CPPT>()); \
    return out("data");                \
  }
    TRY_CONVERT(py::bool_, bool)
    TRY_CONVERT(py::int_, int64_t)
    TRY_CONVERT(py::float_, double)
    TRY_CONVERT(py::bytes, std::string)
    TRY_CONVERT(py::str, std::string)
#undef TRY_CONVERT
    if(py::isinstance<py::tuple>(obj))
    {
      const auto & tuple = obj.cast<py::tuple>();
      auto out = mc_rtc::Configuration::rootArray();
      for(const auto & o : tuple)
      {
        out.push(o);
      }
      return out;
    }
    if(py::isinstance<py::list>(obj))
    {
      const auto & list = obj.cast<py::list>();
      auto out = mc_rtc::Configuration::rootArray();
      for(const auto & o : list)
      {
        out.push(o);
      }
      return out;
    }
    if(py::isinstance<py::dict>(obj))
    {
      mc_rtc::Configuration out;
      const auto & dict = obj.cast<py::dict>();
      for(const auto & itm : dict)
      {
        out.add(itm.first.cast<std::string>(), itm.second);
      }
      return out;
    }
    if(py::isinstance<mc_rtc::Configuration>(obj))
    {
      return obj.cast<mc_rtc::Configuration>();
    }
    ////mc_rtc::log::error("Failed to convert the provided Python object to mc_rtc::Configuration");
    return {};
  }
};

} // namespace mc_rtc

// clang-format off

PYBIND11_MODULE(mc_rtc_rl, m)
{
  m.doc() = "mc_rtc simplified bindings to interact with RL Python libs";

  py::register_exception<mc_rtc::Configuration::Exception>(m, "ConfigurationException");

  // Only support creation from a YAML string
  py::class_<mc_rtc::Configuration>(m, "Configuration")
    .def(py::init<>())
    .def(py::init<const std::string &>())
    .def("__call__", [](mc_rtc::Configuration & c, const std::string & k) { return c(k); })
    .def("__repr__", [](mc_rtc::Configuration & c) { return c.dump(true, true); })
    .def("__getattr__", [](mc_rtc::Configuration & c, size_t idx) { return c[idx]; })
    .def("empty", [](const mc_rtc::Configuration & c) { return c.empty(); })
    .def("size", [](const mc_rtc::Configuration & c) { return c.size(); })
    .def("keys", [](const mc_rtc::Configuration & c) { return c.keys(); })
    .def("add", [](mc_rtc::Configuration & c, const std::string & k) { return c.add(k); })
    .def("array", [](mc_rtc::Configuration & c, const std::string & k) { return c.array(k); })
    .def("add", [](mc_rtc::Configuration & c, const std::string & k, const py::handle & o) { return c.add(k, o); })
    .def("add_array", [](mc_rtc::Configuration & c, const std::string & k, const std::vector<double> & o) { return c.add(k, o); })
    .def("push", [](mc_rtc::Configuration & c, const py::handle & o) { return c.push(o); })
    .def_static("from_string", [](const std::string & s) { return mc_rtc::Configuration::fromYAMLData(s); });

  // Simplified, we can get access to any robot data this way
  py::class_<mc_rbdyn::Robot>(m, "Robot")
    .def_property_readonly("q", [](const mc_rbdyn::Robot & robot) { return rbd::dofToVector(robot.mb(), robot.mbc().q); });

  // Simplified, we can give access to anything in the controller this way
  py::class_<mc_control::fsm::Controller>(m, "Controller")
    .def("robot", [](mc_control::fsm::Controller & ctl) -> mc_rbdyn::Robot & { return ctl.robot(); }, py::return_value_policy::reference)
    .def("robot", [](mc_control::fsm::Controller & ctl, const std::string & name) -> mc_rbdyn::Robot & { return ctl.robot(name); }, py::return_value_policy::reference);

  py::class_<mc_control::MCGlobalController>(m, "GlobalController")
    .def(py::init<const std::string &>())
    // Simplified init callback
    .def("reset", [](mc_control::MCGlobalController & gc) {
          std::string etc_dir = gc.controller().config()("ETC_DIR");
          pid_t pid = ::getpid();
          std::string initial_pose_yaml = etc_dir + "/initial_pose"+std::to_string(pid)+".yaml";
          mc_rtc::Configuration initial(etc_dir + "/initial_pose.yaml");
          sva::PTransformd initPose;
          initPose=initial("initial_pose");
          mc_rtc::Configuration initialOrigin;
          initialOrigin.add("initial_pose", initPose);
          initialOrigin.save(initial_pose_yaml);
          gc.reset();
        })
    // When reset, apply some random deviation to the initial pose configured in initial_pose.yaml
    .def("reset_random", [](mc_control::MCGlobalController & gc) {
          std::string etc_dir = gc.controller().config()("ETC_DIR");
          mc_rtc::Configuration initial(etc_dir + "/initial_pose.yaml");          
          std::map<std::string, std::vector<double>> resetqs = {};
          std::map<std::string, sva::PTransformd> resetAttitudes = {};
          sva::PTransformd initPose;
          initPose=initial("initial_pose");
          //generate three random numbers in the range of (-0.02,0.02)
          double rand1=0.25*(rand()-0.5*RAND_MAX)/RAND_MAX;
          double rand2=0.40*(rand()-0.5*RAND_MAX)/RAND_MAX;
          double rand3=0.40*(rand()-0.5*RAND_MAX)/RAND_MAX;
          sva::PTransformd deviation=sva::PTransformd(sva::RotZ(rand1*4), Eigen::Vector3d(rand2,abs(rand3), 0));
          sva::PTransformd newPose=deviation*initPose;
          resetAttitudes.insert({gc.robot().name(),newPose});
          resetAttitudes.insert({"BITXuanyuan",newPose});
          pid_t pid = ::getpid();
          std::string initial_pose_yaml = etc_dir + "/initial_pose"+std::to_string(pid)+".yaml";
          mc_rtc::Configuration initialRand;
          initialRand.add("initial_pose", initPose);
          initialRand.add("initial_pose_rand",newPose);
          initialRand.save(initial_pose_yaml);
          gc.reset(resetqs,resetAttitudes);
        })

    .def("init", [](mc_control::MCGlobalController & gc) {
           std::vector<double> q;
           {
             auto & mbc = gc.robot().mbc();
             const auto & rjo = gc.ref_joint_order();
             for(const auto & jn : rjo)
             {
               if(gc.robot().hasJoint(jn))
               {
                 for(auto & qj : mbc.q[gc.robot().jointIndexByName(jn)])
                 {
                   q.push_back(qj);
                 }
               }
               else
               {
                 // FIXME This assumes that a joint that is in ref_joint_order but missing from the robot is of size 1 (very
                 // likely to be true)
                 q.push_back(0);
               }
             }
           }
           gc.setEncoderValues(q);
           gc.init(q);
           gc.running = true;
           auto & gui = *gc.controller().gui();
           gui.addElement({"RL"}, mc_rtc::gui::Button("Stop", [&gc]() { gc.running = false; }));
         })
    // Simplified run callback
    .def("run", [](mc_control::MCGlobalController & gc) {
            static std::vector<double> q;
            auto & mbc = gc.robot().mbc();
            const auto & rjo = gc.ref_joint_order();
            q.resize(rjo.size());
            size_t index = 0;
            for(size_t i = 0; i < rjo.size(); ++i)
            {
              const auto & jn = rjo[i];
              if(gc.robot().hasJoint(jn))
              {
                for(auto & qj : mbc.q[gc.robot().jointIndexByName(jn)])
                {
                  q[index] = qj;
                  index++;
                }
              }
              else
              {
                q[index] = 0;
                index++;
              }
            }
            gc.setEncoderValues(q);
            return gc.run();
         })
    .def_readonly("running", &mc_control::MCGlobalController::running)
    .def("stateDone",[](mc_control::MCGlobalController & gc){
          auto & store = gc.controller().datastore();
          bool statedone_=false;
          try { store.get<bool>("StateDone",statedone_); } 
          catch (const std::exception& e) 
          { 
            return false;
          }          
          return statedone_;
         })
    .def("currentState",[](mc_control::MCGlobalController & gc){
          auto & store = gc.controller().datastore();
          std::string currentState="Initial";
          try { store.get<std::string>("CurrentState",currentState); } 
          catch (const std::exception& e) 
          { 
            return currentState;
          }          
          return currentState;
          })
    .def("duration",[](mc_control::MCGlobalController & gc){
          auto & store = gc.controller().datastore();
          double duration=0.01;
          try { store.get<double>("simDuration",duration); } 
          catch (const std::exception& e) 
          { 
            return duration;
          }          
          return duration;
          })
    .def("nextState",[](mc_control::MCGlobalController & gc){
          auto & store = gc.controller().datastore();
          bool next=false;
          //current state name
          std::string currentState="Initial";
          try { store.get<std::string>("CurrentState",currentState); } 
          catch (const std::exception& e) 
          { 
          }
          std::size_t found = currentState.find_last_of("::");
          //filter out the RLMeta's name, if any                    
          std::string prefix;
          std::string prefix_short;
          if (found!=std::string::npos){
            prefix_short=currentState.substr(0,found-1);
            prefix=prefix_short+"::";
          }
          else
          {
            prefix="";
          }
          mc_rtc::log::success("trying to call nextState with prefix {}", prefix);
          try{next = store.call<bool>(prefix+"nextState");}
          catch (const std::exception& e) 
          { 
            mc_rtc::log::error("call nextState failed!");
            //next = store.call<bool>(prefix_short+"FSM::"+"nextState");
          }
          return next;
          })
    .def("ready",[](mc_control::MCGlobalController & gc){
          auto & store = gc.controller().datastore();
          bool ready=false;
          //current state name
          std::string currentState="Initial";
          try { store.get<std::string>("CurrentState",currentState); } 
          catch (const std::exception& e) 
          { 
          }
          std::size_t found = currentState.find_last_of("::");
          //filter out the RLMeta's name, if any                    
          std::string prefix;
          std::string prefix_short;
          if (found!=std::string::npos){
            prefix_short=currentState.substr(0,found-1);
            prefix=prefix_short+"::";
          }
          else
          {
            prefix="";
          }
          try{ready = store.call<bool>(prefix+"ready");}
          catch (const std::exception& e) 
          {
             //ready = store.call<bool>(prefix_short+"FSM::"+"ready");
          }
          return ready;
          })
    .def("EF_trans",[](mc_control::MCGlobalController & gc, const std::string SurfaceName){
          sva::PTransformd lhw_o=(gc.realRobot().surface(SurfaceName)).X_0_s(gc.realRobot());
          std::vector<double> lh_trans(lhw_o.translation().data(), lhw_o.translation().data() + 3);
          return lh_trans;
          })
    .def("EF_rot",[](mc_control::MCGlobalController & gc, const std::string SurfaceName){
          sva::PTransformd lhw_o=(gc.realRobot().surface(SurfaceName)).X_0_s(gc.realRobot());
          //std::vector<double> lh_trans(lhw_o.rotation().data(), lhw_o.rotation().data() + 4);
          Eigen::Quaterniond rot_quat=Eigen::Quaterniond(lhw_o.rotation());
          std::vector<double> lh_trans(rot_quat.coeffs().data(), rot_quat.coeffs().data() + 4);
          return lh_trans;
          })
    .def("Body_trans",[](mc_control::MCGlobalController & gc, const std::string BodyName){
          sva::PTransformd lhw_o=gc.realRobot().bodyPosW(BodyName);
          std::vector<double> lh_trans(lhw_o.translation().data(), lhw_o.translation().data() + 3);
          return lh_trans;
          })
    .def("Body_rot",[](mc_control::MCGlobalController & gc, const std::string BodyName){
          sva::PTransformd lhw_o=gc.realRobot().bodyPosW(BodyName);
          Eigen::Quaterniond rot_quat=Eigen::Quaterniond(lhw_o.rotation());
          std::vector<double> lh_trans(rot_quat.coeffs().data(), rot_quat.coeffs().data() + 4);
          return lh_trans;
          })
    .def("EF_force",[](mc_control::MCGlobalController & gc, const std::string SurfaceName){
          sva::ForceVecd lhw_o=(gc.realRobot().surfaceWrench(SurfaceName));
          std::vector<double> lh_trans(lhw_o.force().data(), lhw_o.force().data() + 3);
          return lh_trans;
          })
    .def("EF_couple",[](mc_control::MCGlobalController & gc, const std::string SurfaceName){
          sva::ForceVecd lhw_o=(gc.realRobot().surfaceWrench(SurfaceName));
          std::vector<double> lh_trans(lhw_o.couple().data(), lhw_o.couple().data() + 3);
          return lh_trans;
          })
    .def("posW_trans",[](mc_control::MCGlobalController & gc){
          sva::PTransformd posW=(gc.realRobot().posW());
          std::vector<double> posW_trans(posW.translation().data(), posW.translation().data() + 3);
          return posW_trans;
          })
    .def("posW_rot",[](mc_control::MCGlobalController & gc){
          sva::PTransformd posW=(gc.realRobot().posW());
          Eigen::Quaterniond rot_quat=Eigen::Quaterniond(posW.rotation());
          std::vector<double> lh_trans(rot_quat.coeffs().data(), rot_quat.coeffs().data() + 4);
          return lh_trans;
          })
    .def("velW_trans",[](mc_control::MCGlobalController & gc){
          sva::MotionVecd posW=(gc.realRobot().velW());
          std::vector<double> posW_trans(posW.linear().data(), posW.linear().data() + 3);
          return posW_trans;
          })
    .def("velW_rot",[](mc_control::MCGlobalController & gc){
          sva::MotionVecd posW=(gc.realRobot().velW());
          std::vector<double> posW_rot(posW.angular().data(), posW.angular().data() + 3);
          return posW_rot;
          })
    .def("accW_trans",[](mc_control::MCGlobalController & gc){
          sva::MotionVecd posW=(gc.realRobot().accW());
          std::vector<double> posW_trans(posW.linear().data(), posW.linear().data() + 3);
          return posW_trans;
          })
    .def("accW_rot",[](mc_control::MCGlobalController & gc){
          sva::MotionVecd posW=(gc.realRobot().accW());
          std::vector<double> posW_rot(posW.angular().data(), posW.angular().data() + 3);
          return posW_rot;
          })
    .def("door_handle",[](mc_control::MCGlobalController & gc){
            return gc.realRobots().robots()[1].q()[2][0];
          })
    .def("door_door",[](mc_control::MCGlobalController & gc){
            return gc.realRobots().robots()[1].q()[1][0];
          })
    .def("com",[](mc_control::MCGlobalController & gc){
            return gc.realRobot().com();
          })
    .def("comVel",[](mc_control::MCGlobalController & gc){
            return gc.realRobot().comVelocity();
          })
    .def("comAcc",[](mc_control::MCGlobalController & gc){
            return gc.realRobot().comAcceleration();
          })
    .def("real_com",[](mc_control::MCGlobalController & gc){
            return gc.realRobot().com();
          })
    .def("gripper_torque",[](mc_control::MCGlobalController & gc){
            if (isnan(gc.realRobot().jointTorques()[16]))
            {
              return 0.0;
            }
            else
            {
              return gc.realRobot().jointTorques()[16];
            }            
          })
    .def("saveAction",[](mc_control::MCGlobalController & gc, std::vector<double> action){
          auto & store = gc.controller().datastore();
          if (store.has("action")){
            store.assign("action",action);
          }
          else
          {
            store.make<std::vector<double>>("action",action);
          }
          })
    .def("set_rlinterface_done_cb", [](mc_control::MCGlobalController & gc, python_done_cb cb) {
            auto & store = gc.controller().datastore();
            store.assign("RLInterface::done", done_cb([cb](const std::string & s, mc_control::fsm::Controller & ctl) { return cb(s, &ctl); }));
         })
    .def("set_rlinterface_start_cb", [](mc_control::MCGlobalController & gc, python_start_cb cb) {
            auto & store = gc.controller().datastore();
            store.assign("RLInterface::start", start_cb([cb](const std::string & s, mc_control::fsm::Controller & ctl) { return cb(s, &ctl); }));
         });

  py::class_<mc_mujoco::MjSim>(m, "MjSim")
    .def(py::init([]() { return make_mj_sim(); }))
    .def(py::init([](const std::string & config) { return make_mj_sim(config); }))
    .def(py::init([](bool with_visualization) { return make_mj_sim("", with_visualization); }))
    .def(py::init(&make_mj_sim))
    .def("gc", [](mc_mujoco::MjSim & sim) -> mc_control::MCGlobalController & { return *sim.controller(); }, py::return_value_policy::reference)
    .def("stepSimulation", &mc_mujoco::MjSim::stepSimulation, py::call_guard<py::gil_scoped_release>())
    .def("stopSimulation", &mc_mujoco::MjSim::stopSimulation, py::call_guard<py::gil_scoped_release>())
    .def("updateScene", &mc_mujoco::MjSim::updateScene, py::call_guard<py::gil_scoped_release>())
    .def("render", &mc_mujoco::MjSim::render, py::call_guard<py::gil_scoped_release>())
    .def("reset_random", [](mc_mujoco::MjSim & sim) {
      auto & gc = *sim.controller();
      std::string etc_dir = gc.controller().config()("ETC_DIR");
      mc_rtc::Configuration initial(etc_dir + "/initial_pose.yaml");
      std::map<std::string, std::vector<double>> resetqs = {};
      std::map<std::string, sva::PTransformd> resetAttitudes = {};
      sva::PTransformd initPose;
      initPose=initial("initial_pose");
      //generate three random numbers in the range of (-0.02,0.02)
      double rand1=0.25*(rand()-0.5*RAND_MAX)/RAND_MAX;
      double rand2=0.40*(rand()-0.5*RAND_MAX)/RAND_MAX;
      double rand3=0.40*(rand()-0.5*RAND_MAX)/RAND_MAX;
      sva::PTransformd deviation=sva::PTransformd(sva::RotZ(4.0*rand1), Eigen::Vector3d(rand2,abs(rand3), 0));
      sva::PTransformd newPose=deviation*initPose;
      resetAttitudes.insert({gc.robot().name(),newPose});
      resetAttitudes.insert({"BITHumanoid",newPose}); 
      pid_t pid = ::getpid();
      std::string initial_pose_yaml = etc_dir + "/initial_pose"+std::to_string(pid)+".yaml";
      mc_rtc::Configuration initialRand;
      initialRand.add("initial_pose", initPose);
      initialRand.add("initial_pose_rand",newPose);
      initialRand.save(initial_pose_yaml);
      sim.resetSimulation(resetqs, resetAttitudes);
    })
    .def("reset", [](mc_mujoco::MjSim & sim) {
      auto & gc = *sim.controller();
      std::string etc_dir = gc.controller().config()("ETC_DIR");
      mc_rtc::Configuration initial(etc_dir + "/initial_pose.yaml");
      mc_rtc::Configuration initialQs(etc_dir + "/initial_posture.yaml");
      std::map<std::string, std::vector<double>> resetqs = {};
      std::map<std::string, sva::PTransformd> resetAttitudes = {};
      sva::PTransformd initPose;
      initPose=initial("initial_pose");
      //generate three random numbers in the range of (-0.1,0.1)
      resetAttitudes.insert({gc.robot().name(),initPose});
      pid_t pid = ::getpid();
      std::string initial_pose_yaml = etc_dir + "/initial_pose"+std::to_string(pid)+".yaml";
      mc_rtc::Configuration initialOrigin;
      initialOrigin.add("initial_pose", initPose);
      initial.save(initial_pose_yaml);
      sim.resetSimulation(resetqs, resetAttitudes);
    });
}

// clang-format on
