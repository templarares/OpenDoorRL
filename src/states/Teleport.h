#pragma once

#include <mc_control/fsm/State.h>

struct Teleport : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
// private:
//     bool openDoor_=false;
private:
    //teleportation transform, including 2 dof in translation and 1 dof in rotation    
    double dx=0.01;
    double dy=0;
    double dyaw=0.01;
    //number of steps the overall teleporation is granulized; this is to enable collision with the door;
    double steps=1;
    int counter = 0;
};
