#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>

struct VREPRemoteAPIWrapperImpl;

struct VREPRemoteAPIWrapper
{
public:
  VREPRemoteAPIWrapper(const std::string & host, int port, int timeout = 3000, bool waitUntilConnected = true, bool doNotReconnect = true, int commThreadCycleInMs = 1);

  ~VREPRemoteAPIWrapper();

  void startSimulation(mc_control::MCGlobalController & controller);

  void nextSimulationStep(mc_control::MCGlobalController & controller);

  void stopSimulation();
private:
  std::unique_ptr<VREPRemoteAPIWrapperImpl> impl;
};
