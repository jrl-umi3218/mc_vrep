#include "vrep_remote_api_wrapper.h"
#include "mc_vrep_cli.h"

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <cmath>
#include <iostream>
#include <thread>

void simThread(VREPRemoteAPIWrapper & vrep, mc_control::MCGlobalController & controller, MCVREPCLI & cli)
{
  while(!cli.done())
  {
    vrep.nextSimulationStep(controller);
  }
  vrep.stopSimulation();
}

int main(int, char *[])
{
  /* Start the VREP remote API */
  VREPRemoteAPIWrapper vrep("127.0.0.1", 4242);

  /* Create a global controller */
  mc_control::MCGlobalController controller;

  vrep.startSimulation(controller);

  MCVREPCLI cli(controller);
  std::thread th(std::bind(&simThread,
                           std::ref<VREPRemoteAPIWrapper>(vrep),
                           std::ref<mc_control::MCGlobalController>(controller),
                           std::ref<MCVREPCLI>(cli)));

  cli.run();

  th.join();
  return 0;
}
