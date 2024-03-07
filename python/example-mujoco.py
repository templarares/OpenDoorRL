#!/usr/bin/env python3
import sys
import mc_rtc_rl

sim = mc_rtc_rl.MjSim('mc_rtc.yaml', True)
gc = sim.gc()

render = True

iter_ = 0
while render:
    sim.stepSimulation()
    if iter_ % 50 == 0:
        sim.updateScene()
        render = sim.render()
    if (iter_+1) %20000==0:
        sim.reset()
    iter_ += 1

sim.stopSimulation()
