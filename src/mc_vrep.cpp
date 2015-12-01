#include "vrep_remote_api_wrapper.h"

#include <cmath>
#include <iostream>

int main(int, char *[])
{
  VREPRemoteAPIWrapper vrep("127.0.0.1", 4242);

  vrep.startSimulation();

  float dir = 0.01f;
  float j7 = vrep.getJointPosition("RARM_JOINT7");
  for(unsigned int i = 0; i < 200*20; ++i)
  {
    if(std::fabs(j7) >= 0.7f)
    {
      dir = -1.0f*dir;
    }
    j7 += dir;
    vrep.setJointTargetPosition("RARM_JOINT7", j7);
    vrep.nextSimulationStep();
  }

  vrep.stopSimulation();

  return 0;
}
