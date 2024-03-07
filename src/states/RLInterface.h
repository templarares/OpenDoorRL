#pragma once

#include <mc_control/fsm/states/MetaTasks.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/inotify.h>

#include <boost/algorithm/string.hpp>

#define EVENT_SIZE  ( sizeof (struct inotify_event) )
#define EVENT_BUF_LEN     ( 1024 * ( EVENT_SIZE + 16 ) )


namespace mc_control
{

namespace fsm
{

struct RLInterfaceState : MetaTasksState
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override;
private:
  mc_rtc::Configuration actualConfig_;
  bool started_ = false;
  int iterationCounter_ =0;
};

} // namespace fsm

} // namespace mc_control
