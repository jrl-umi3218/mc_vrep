/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "vrep_simulation.h"
#include "mc_vrep_cli.h"

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>

#include <cmath>
#include <iostream>
#include <thread>

void simThread(VREPSimulation & vrep, MCVREPCLI & cli, bool stepByStep)
{
  while(!cli.done())
  {
    vrep.nextSimulationStep();
    while(stepByStep && !cli.next() && !cli.done())
    {
    }
    cli.play();
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
  VREPSimulationConfiguration config;
  auto vrep_c = controller.configuration().config("VREP", mc_rtc::Configuration{});
  vrep_c("Host", config.host);
  vrep_c("Port", config.port);
  vrep_c("Timeout", config.timeout);
  vrep_c("WaitUntilConnected", config.waitUntilConnected);
  vrep_c("DoNotReconnect", config.doNotReconnect);
  vrep_c("CommThreadCycleInMs", config.commThreadCycleInMs);
  vrep_c("SimulationTimestep", config.simulationTimestep);
  if(config.simulationTimestep < 0)
  {
    config.simulationTimestep = controller.timestep();
  }
  vrep_c("StepByStep", config.stepByStep);
  vrep_c("TorqueControl", config.torqueControl);
  if(vrep_c.has("Extras"))
  {
    auto extras_c = vrep_c("Extras");
    for(size_t i = 0; i < extras_c.size(); ++i)
    {
      unsigned int idx = extras_c[i]("index");
      std::string suffix = extras_c[i]("suffix", std::string(""));
      config.extras.push_back({idx, suffix});
    }
  }
  VREPSimulation vrep(controller, config);


  vrep.startSimulation();

  MCVREPCLI cli(controller, vrep);
  std::thread th(std::bind(&MCVREPCLI::run, &cli));
  simThread(vrep, cli, config.stepByStep);

  th.join();
  return 0;
}
