#include "vrep_remote_api_wrapper.h"

extern "C"
{
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpedantic"
  #include "vrep_remote_api/extApi.h"
  #pragma GCC diagnostic pop
}

#include <iostream>
#include <map>

struct VREPForceSensor
{
  uint8_t status = 0;
  float forces[3] = {0.0f, 0.0f, 0.0f};
  float torques[3] = {0.0f, 0.0f, 0.0f};
};

struct VREPRemoteAPIWrapperImpl
{
  /* Store client id */
  int cId = -1;
  std::map<std::string, VREPForceSensor> sensors = {};
  /* Default communication mode with the API */
  const static int opmode = simx_opmode_oneshot_wait;
  const static int opmode_streaming = simx_opmode_streaming;
  const static int opmode_buffer = simx_opmode_buffer;
  int handle(const std::string & name)
  {
    if(handles.count(name) == 0)
    {
      int id;
      simxGetObjectHandle(cId, name.c_str(), &id, opmode);
      std::cout << "New handle for object " << name << " " << id << std::endl;
      if(id == 0)
      {
        std::cerr << "No handle for object " << name << std::endl;
        throw("Unrecognized object");
      }
      handles[name] = id;
    }
    return handles[name];
  }
private:
  /* Store handles */
  std::map<std::string, int> handles = {};
};

VREPRemoteAPIWrapper::VREPRemoteAPIWrapper(const std::string & host, int port, int timeout, bool waitUntilConnected, bool doNotReconnect, int commThreadCycleInMs): impl(new VREPRemoteAPIWrapperImpl())
{
  impl->cId = simxStart(host.c_str(), port, waitUntilConnected ? 1 : 0, doNotReconnect ? 1 : 0, timeout, commThreadCycleInMs);
  if(impl->cId >= 0)
  {
    std::cout << "Connected to VREP" << std::endl;
  }
  else
  {
    throw("Failed to connect to VREP");
  }
  simxSynchronous(impl->cId, 1);
}

VREPRemoteAPIWrapper::~VREPRemoteAPIWrapper()
{
  simxFinish(impl->cId);
}

void VREPRemoteAPIWrapper::readForceSensors(const std::vector<std::string> & sensors)
{
  for(const auto & sensor : sensors)
  {
    int opmode = impl->opmode_buffer;
    if(impl->sensors.count(sensor) == 0)
    {
      std::cout << "Add sensor " << sensor << std::endl;
      impl->sensors[sensor] = VREPForceSensor();
      opmode = impl->opmode_streaming;
    }
    simxReadForceSensor(impl->cId, impl->handle(sensor),
                        &(impl->sensors[sensor].status),
                        &(impl->sensors[sensor].forces[0]),
                        &(impl->sensors[sensor].torques[0]),
                        opmode);
  }
}

void VREPRemoteAPIWrapper::startSimulation()
{
  simxStartSimulation(impl->cId, impl->opmode);
}

void VREPRemoteAPIWrapper::stopSimulation()
{
  simxStopSimulation(impl->cId, impl->opmode);
}

void VREPRemoteAPIWrapper::nextSimulationStep()
{
    simxSynchronousTrigger(impl->cId);
}

int VREPRemoteAPIWrapper::getJointHandle(const std::string & jname)
{
  return impl->handle(jname);
}

void VREPRemoteAPIWrapper::setJointTargetPosition(const std::string & jname, float value)
{
  simxSetJointTargetPosition(impl->cId, impl->handle(jname), value, impl->opmode);
}

float VREPRemoteAPIWrapper::getJointPosition(const std::string & jname)
{
  float ret;
  simxGetJointPosition(impl->cId, impl->handle(jname), &ret, impl->opmode);
  return ret;
}

Eigen::Vector3f VREPRemoteAPIWrapper::getForce(const std::string & sensor)
{
  if(impl->sensors.count(sensor) == 0)
  {
    std::cerr << "Call readForceSensors with sensor " << sensor << " before attempting to get sensor values" << std::endl;
    throw("Call readForceSensors before reading a sensor");
  }
  Eigen::Vector3f ret;
  ret.x() = impl->sensors[sensor].forces[0];
  ret.y() = impl->sensors[sensor].forces[1];
  ret.z() = impl->sensors[sensor].forces[2];
  return ret;
}
