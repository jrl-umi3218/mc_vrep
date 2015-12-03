#pragma once

#include <mc_control/mc_global_controller.h>

struct MCVREPCLI
{
public:
  MCVREPCLI(mc_control::MCGlobalController & controller);

  void run();

  bool done() const;
private:
  mc_control::MCGlobalController & controller;
  bool done_ = false;
};
