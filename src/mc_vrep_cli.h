#pragma once

#include <mc_control/mc_global_controller.h>
#include "vrep_simulation.h"

struct MCVREPCLI
{
public:
  MCVREPCLI(mc_control::MCGlobalController & controller, VREPSimulation& vrep);

  void run();

  bool done() const;
private:
  mc_control::MCGlobalController & controller;
  VREPSimulation & vrep;
  bool done_ = false;
};
