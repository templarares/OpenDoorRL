#!/usr/bin/env python3
import sys
import mc_rtc_rl

gc = mc_rtc_rl.GlobalController('mc_rtc.yaml')

gc.init()

def done_callback(name, controller):
    #print("{} done, robot configuration: {}".format(name, controller.robot().q))
    pass
def stateName() -> str:
    """just a test funtion that outputs the name of the state to be modified"""
    return "IngressFSM::RightFootCloseToCar::LiftFoot"

def start_callback(name, controller):
    print("{} starting to run".format(name))
    if (name==stateName()):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        com.add("name", "CoMgg")
        return config

    # add custom codes here. Remove all entries but the "base:" one. Enter them here.
    return mc_rtc_rl.Configuration.from_string("{}")

gc.set_rlinterface_done_cb(done_callback)
gc.set_rlinterface_start_cb(start_callback)
while gc.running:
    gc.run()
print("global controller died")
