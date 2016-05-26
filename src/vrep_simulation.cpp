#include "vrep_simulation.h"

#include <vrep-api-wrapper/vrep.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <array>
#include <fstream>
#include <iostream>
#include <map>

struct VREPSimulationImpl
{
private:
  mc_control::MCGlobalController & controller;
  vrep::VREP vrep;

  std::string baseName;
  std::vector<double> jQs;
  std::vector<double> jTorques;
  std::map<std::string, vrep::VREP::ForceSensor> fSensors;
  vrep::VREP::Accelerometer accel;
  vrep::VREP::Gyrometer gyro;
  sva::PTransformd basePos;

  /* Gripper related */
  std::map<std::string, std::vector<size_t>> gripper_in_index;
  std::map<std::string, std::vector<double>> realGripperQs;
public:
  VREPSimulationImpl(mc_control::MCGlobalController & controller, const std::string & host, int port, int timeout, bool waitUntilConnected, bool doNotReconnect, int commThreadCycleInMs)
  : controller(controller), vrep(host, port, timeout, waitUntilConnected, doNotReconnect, commThreadCycleInMs)
  {
    auto gripperJs = controller.gripperJoints();
    auto gripperActiveJs = controller.gripperActiveJoints();
    const auto & ref_joint_order = controller.ref_joint_order();
    for(const auto & g : gripperActiveJs)
    {
      gripper_in_index[g.first] = {};
      realGripperQs[g.first] = {};
      for(const auto & jn : g.second)
      {
        for(size_t i = 0; i < ref_joint_order.size(); ++i)
        {
          if(ref_joint_order[i] == jn)
          {
            gripper_in_index[g.first].push_back(i);
            realGripperQs[g.first].push_back(0.0);
          }
        }
      }
    }
  }

  typedef sva::ForceVecd wrench_t;
  std::map<std::string, wrench_t> wrenches()
  {
    std::map<std::string, wrench_t> res;
    for(const auto & fs : fSensors)
    {
      res.emplace(fs.first, wrench_t{fs.second.force, fs.second.torque});
    }
    return res;
  }


  void startSimulation()
  {
    const mc_rbdyn::Robot & robot = controller.robot();
    std::string jName = "";
    for(const auto & j : robot.mb().joints())
    {
      if(j.dof() == 1)
      {
        jName = j.name();
        break;
      }
    }
    if(jName == "")
    {
      LOG_ERROR("No 1-dof joints in your main robot, aborting")
      throw("Invalid robot");
    }
    baseName = vrep.getModelBase(jName);
    for(const auto & fn : robot.forceSensorsByName())
    {
      fSensors[fn] = {};
    }
    vrep.startSimulation(baseName,
                         controller.ref_joint_order(),
                         fSensors);
    /* Run simulation until the data arrives */
    while(! (vrep.getJointsData(controller.ref_joint_order(), jQs, jTorques) &&
             vrep.getSensorData(fSensors, accel, gyro) &&
             vrep.getBasePos(baseName, basePos)) )
    {
      vrep.nextSimulationStep();
    }
    for(size_t i = 0; i < 10; ++i)
    {
      vrep.nextSimulationStep();
    }
    controller.running = true;
    Eigen::Vector3d t(basePos.translation());
    Eigen::Quaterniond q(basePos.rotation());
    q = q.inverse();
    std::array<double, 7> initAttitude {{q.w(), q.x(), q.y(), q.z(), t.x(), t.y(), t.z()}};
    updateData();
    controller.init(jQs, initAttitude);
    LOG_SUCCESS("Simulation started")
  }

  void updateData()
  {
    Eigen::Vector3d rpy = basePos.rotation().eulerAngles(0, 1, 2);
    controller.setSensorOrientation(rpy);
    controller.setSensorVelocity(gyro.data);
    controller.setSensorAcceleration(accel.data);
    controller.setEncoderValues(jQs);
    controller.setWrenches(wrenches());
    controller.setJointTorques(jTorques);
  }

  void nextSimulationStep()
  {
    vrep.getJointsData(controller.ref_joint_order(), jQs, jTorques);
    vrep.getSensorData(fSensors, accel, gyro);
    vrep.getBasePos(baseName, basePos);

    updateData();
    if(controller.run())
    {
      /*FIXME Gripper control stuff*/
      auto gripperQs = controller.gripperQ();
      for(auto & rG : realGripperQs)
      {
        const auto & idx = gripper_in_index[rG.first];
        auto & qs = rG.second;
        for(size_t i = 0; i < idx.size(); ++i)
        {
          qs[i] = jQs[idx[i]];
        }
      }
      controller.setActualGripperQ(realGripperQs);

      auto mbc = controller.robot().mbc();
      auto gripperJs = controller.gripperJoints();
      for(const auto & gQ : gripperQs)
      {
        const auto & qs = gQ.second;
        const auto & jns = gripperJs[gQ.first];
        for(size_t i = 0; i < qs.size(); ++i)
        {
          if(controller.robot().hasJoint(jns[i]))
          {
            auto idx = controller.robot().jointIndexByName(jns[i]);
            mbc.q[idx][0] = qs[i];
          }
        }
      }
      vrep.setRobotTargetConfiguration(controller.robot().mb(), mbc);
    }
    vrep.nextSimulationStep();
  }

  void stopSimulation()
  {
    vrep.stopSimulation();
  }
};

VREPSimulation::VREPSimulation(mc_control::MCGlobalController & controller, const std::string & host, int port, int timeout, bool waitUntilConnected, bool doNotReconnect, int commThreadCycleInMs)
: impl(new VREPSimulationImpl(controller, host, port, timeout, waitUntilConnected, doNotReconnect, commThreadCycleInMs))
{
}

VREPSimulation::~VREPSimulation()
{
}

void VREPSimulation::startSimulation()
{
  impl->startSimulation();
}

void VREPSimulation::nextSimulationStep()
{
  impl->nextSimulationStep();
}

void VREPSimulation::stopSimulation()
{
  impl->stopSimulation();
}
