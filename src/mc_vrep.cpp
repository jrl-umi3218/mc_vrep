#include "vrep_remote_api_wrapper.h"

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <cmath>
#include <iostream>

int main(int, char *[])
{
  /* Start the VREP remote API */
  VREPRemoteAPIWrapper vrep("127.0.0.1", 4242);

  /* Create a global controller */
  mc_control::MCGlobalController controller;

  vrep.startSimulation(controller);

  double cur_cmd = 0.6;
  for(unsigned int i = 0; i < 200*20; ++i)
  {
    if(fabs(controller.robot().mbc().q[controller.robot().jointIndexByName("HEAD_JOINT1")][0] - cur_cmd) < 0.05) { cur_cmd = -cur_cmd; }
    controller.set_joint_pos("HEAD_JOINT1", cur_cmd);
    vrep.nextSimulationStep(controller);
  }

  vrep.stopSimulation();

  return 0;
}
