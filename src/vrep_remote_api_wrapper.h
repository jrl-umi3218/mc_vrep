#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

/* FIXME For now, this reads raw-data */

struct VREPRemoteAPIWrapperImpl;

struct VREPRemoteAPIWrapper
{
public:
  VREPRemoteAPIWrapper(const std::string & host, int port, int timeout = 3000, bool waitUntilConnected = true, bool doNotReconnect = true, int commThreadCycleInMs = 1);

  ~VREPRemoteAPIWrapper();

  void readForceSensors(const std::vector<std::string> & sensors);

  void startSimulation();

  void stopSimulation();

  void nextSimulationStep();

  int getJointHandle(const std::string & jname);

  void setJointTargetPosition(const std::string & jname, float value);

  float getJointPosition(const std::string & jname);

  Eigen::Vector3f getForce(const std::string & sensor);
private:
  std::unique_ptr<VREPRemoteAPIWrapperImpl> impl;
};
