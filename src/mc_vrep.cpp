#include "vrep_simulation.h"
#include "mc_vrep_cli.h"

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>

#include <cmath>
#include <iostream>
#include <thread>

void simThread(VREPSimulation & vrep, MCVREPCLI & cli)
{
  while(!cli.done())
  {
    vrep.nextSimulationStep();
  }
  vrep.stopSimulation();
}

int main(int argc, char * argv[])
{
  /* Create a global controller */
  std::string conf_file = "";
  if(argc > 1)
  {
    conf_file = argv[1];
  }
  mc_control::MCGlobalController controller(conf_file);

  /* Start the VREP remote API */
  VREPSimulation vrep(controller);


  vrep.startSimulation();

  MCVREPCLI cli(controller);
  std::thread th(std::bind(&MCVREPCLI::run, &cli));
  simThread(vrep, cli);

  th.join();
  return 0;
}
