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

int main(int argc, char * argv[])
{
  /* Create a global controller */
  std::string conf_file = mc_rtc::CONF_PATH;
  if(argc > 1)
  {
    conf_file = argv[1];
  }
  mc_control::MCGlobalController controller(conf_file);
  //mc_control::MCGlobalController controller("C:/Work/Code/Joris/mc_vrep/etc/mc_vrep.conf");

  /* Start the VREP remote API */
  VREPRemoteAPIWrapper vrep(controller.robot().name(), "127.0.0.1", 19997);

  
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
