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

struct VREPRemoteAPIWrapperImpl
{
  /* Store client id */
  int cId = -1;
  /* Default communication mode with the API */
  const static int opmode = simx_opmode_oneshot_wait;
  int handle(const std::string & jname)
  {
    if(handles.count(jname) == 0)
    {
      int jid;
      simxGetObjectHandle(cId, jname.c_str(), &jid, opmode);
      std::cout << "Got new handle for " << jname << " " << jid << std::endl;
      handles[jname] = jid;
    }
    return handles[jname];
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
