#pragma once

#include <memory>

struct VREPRemoteAPIWrapperImpl;

struct VREPRemoteAPIWrapper
{
public:
  VREPRemoteAPIWrapper(const std::string & host, int port, int timeout = 3000, bool waitUntilConnected = true, bool doNotReconnect = true, int commThreadCycleInMs = 1);

  ~VREPRemoteAPIWrapper();

  void startSimulation();

  void stopSimulation();

  void nextSimulationStep();

  int getJointHandle(const std::string & jname);

  void setJointTargetPosition(const std::string & jname, float value);

  float getJointPosition(const std::string & jname);
private:
  std::unique_ptr<VREPRemoteAPIWrapperImpl> impl;
};
