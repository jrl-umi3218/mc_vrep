/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

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

  const bool torqueControl;

  std::vector<VREPSimulationConfiguration::ExtraRobot> extraRobots;

  std::vector<unsigned int> rIdx;
  std::vector<std::string> suffixes;
  std::vector<std::string> baseNames;
  std::vector<std::string> joints;
  std::vector<double> jQs;
  std::vector<double> jTorques;
  std::map<std::string, vrep::VREP::ForceSensor> fSensors;
  vrep::VREP::Accelerometer accel;
  vrep::VREP::Gyrometer gyro;
  std::vector<sva::PTransformd> basePoses;
  std::vector<sva::MotionVecd> baseVels;

  /* Gripper related */
  std::map<std::string, std::vector<size_t>> gripper_in_index;
  std::map<std::string, std::vector<double>> realGripperQs;

  std::map<std::string, sva::ForceVecd> external_force;
  std::map<std::string, sva::ForceVecd> impact_force;

  double simulationTimestep = 0.005;
  size_t iter = 0;
  size_t frameskip = 1;
public:
  VREPSimulationImpl(mc_control::MCGlobalController & controller, bool torqueControl, const std::string & host, int port, int timeout, bool waitUntilConnected, bool doNotReconnect, int commThreadCycleInMs, const std::vector<VREPSimulationConfiguration::ExtraRobot> & extraRobots, double simulationTimestep)
  : controller(controller), vrep(host, port, timeout, waitUntilConnected, doNotReconnect, commThreadCycleInMs), torqueControl(torqueControl), extraRobots(extraRobots)
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
    auto & ctl = controller.controller();
    auto gui = ctl.gui();
    if(gui)
    {
      auto data = gui->data();
      auto vrep_data = data.add("VREP");
      auto vrep_bodies = vrep_data.array("bodies");
      for(auto & b : ctl.robot().mb().bodies())
      {
        if(b.inertia().mass() != 0)
        {
          vrep_bodies.push(b.name() + "_respondable");
        }
      }
      gui->addElement({"VREP", "Force"},
        mc_rtc::gui::Form("Apply force",
                          [this,gui](const mc_rtc::Configuration & data)
                          {
                            std::string body = data("Body");
                            Eigen::Vector6d force = data("Force");
                            setExternalForce(body, {force});
                            gui->addElement({"VREP", "Force"},
                              mc_rtc::gui::Button("Remove force on " + body,
                                                  [this,body,gui]()
                                                  {
                                                    std::string msg = "removeForce " + body; 
                                                    removeExternalForce(body);
                                                    gui->removeElement({"VREP", "Force"}, "Remove force on " + body);
                                                  }
                                                  )
                            );
                          },
                          mc_rtc::gui::FormDataComboInput("Body", true, {"VREP", "bodies"}),
                          mc_rtc::gui::FormArrayInput<Eigen::Vector6d>("Force", true, Eigen::Vector6d::Zero())
                         )
        );
      gui->addElement({"VREP", "Impact"},
        mc_rtc::gui::Form("Apply impact",
                          [this,gui](const mc_rtc::Configuration & data)
                          {
                            std::string body = data("Body");
                            Eigen::Vector6d force = data("Force (N.s)");
                            applyImpact(body, {force});
                          },
                          mc_rtc::gui::FormDataComboInput("Body", true, {"VREP", "bodies"}),
                          mc_rtc::gui::FormArrayInput<Eigen::Vector6d>("Force (N.s)", true, Eigen::Vector6d::Zero())
                         )
      );
      gui->addElement({"VREP"},
                      mc_rtc::gui::Button("Stop", [this](){ stopSimulation(); std::exit(0); }));
    }

    this->simulationTimestep = simulationTimestep;
    frameskip = std::round(ctl.timeStep / simulationTimestep);
    LOG_INFO("[mc_vrep] Frameskip: " << frameskip)
  }

  typedef sva::ForceVecd wrench_t;
  std::map<std::string, wrench_t> wrenches(const mc_rbdyn::Robot & robot, const std::string & suffix)
  {
    std::map<std::string, wrench_t> res;
    for(const auto & fs : robot.forceSensors())
    {
      for(const auto & fIn : fSensors)
      {
        if(fIn.first == fs.name() + suffix)
        {
          res.emplace(fs.name(), wrench_t{fIn.second.torque, fIn.second.force});
        }
      }
    }
    return res;
  }

  void startSimulation()
  {
    auto & robots = controller.controller().robots();
    for(size_t i = controller.realRobots().size(); i < robots.size(); ++i)
    {
      controller.realRobots().robotCopy(robots.robot(i));
    }
    rIdx.push_back(0);
    suffixes.push_back("");
    for(auto & e : extraRobots)
    {
      rIdx.push_back(e.index);
      suffixes.push_back(e.suffix);
    }
    for(size_t i = 0; i < rIdx.size(); ++i)
    {
      const auto & suffix = suffixes[i];
      const auto & robot = robots.robot(rIdx[i]);
      std::string jName = "";
      std::string baseName = jName;
      for(const auto & j : robot.mb().joints())
      {
        if(j.dof() == 1)
        {
          jName = j.name();
          break;
        }
      }
      if(jName == "" && i == 0)
      {
        LOG_ERROR_AND_THROW(std::runtime_error, "No 1-dof joints in your main robot, aborting")
      }
      else if(jName == "")
      {
        if(robot.mb().bodies().size() > 1 && robot.mb().body(0).name() == "base_link")
        {
          baseName = robot.mb().body(1).name();
        }
        else
        {
          baseName = robot.mb().body(0).name();
        }
        LOG_WARNING("ExtraRobot with index " << i << " cannot be controlled, will only track the base position " << baseName)
      }
      else
      {
        baseName = vrep.getModelBase(jName + suffix);
      }
      baseNames.push_back(baseName);
      for(const auto & fs : robot.forceSensors())
      {
        fSensors[fs.name() + suffix] = {};
      }
      for(const auto & j : robot.refJointOrder())
      {
        joints.push_back(j + suffix);
      }
    }
    vrep.startSimulation(baseNames,
                         joints,
                         fSensors);
    /* Run simulation until the data arrives */
    while(! vrep.getSimulationState(joints, jQs, jTorques, fSensors, accel, gyro, baseNames, basePoses, baseVels))
    {
      vrep.nextSimulationStep();
    }
    for(size_t i = 0; i < 10; ++i)
    {
      vrep.nextSimulationStep();
    }
    controller.running = true;
    for(size_t i = 0; i < rIdx.size(); ++i)
    {
      auto & robot = robots.robot(rIdx[i]);
      robot.posW(basePoses[i]);
    }
    updateData();
    controller.init(controller.robot().encoderValues());
    LOG_SUCCESS("Simulation started")
  }

  void updateData()
  {
    size_t jQi = 0;
    for(size_t i = 0; i < rIdx.size(); ++i)
    {
      auto & robot = controller.controller().robots().robot(rIdx[i]);
      robot.bodySensor().position(basePoses[i].translation());
      robot.bodySensor().orientation(Eigen::Quaterniond(basePoses[i].rotation()));
      robot.bodySensor().linearVelocity(baseVels[i].linear());
      robot.bodySensor().angularVelocity(baseVels[i].angular());
      std::vector<double> encoders(robot.refJointOrder().size());
      std::vector<double> torques(robot.refJointOrder().size());
      std::vector<double> prevEncoders = robot.encoderValues();
      for(size_t j = 0; j < robot.refJointOrder().size(); ++j)
      {
        encoders[j] = jQs[jQi + j];
        torques[j] = jTorques[jQi + j];
      }
      jQi += robot.refJointOrder().size();
      robot.encoderValues(encoders);
      robot.jointTorques(jTorques);
      controller.setWrenches(rIdx[i], wrenches(robot, suffixes[i]));
      auto & real_robot = controller.realRobots().robot(rIdx[i]);
      real_robot.encoderValues(encoders);
      if(prevEncoders.size() == 0) { prevEncoders = robot.encoderValues(); }
      if(robot.mb().joint(0).type() == rbd::Joint::Type::Free)
      {
        real_robot.mbc().alpha[0] = {
          baseVels[i].angular().x(),
          baseVels[i].angular().y(),
          baseVels[i].angular().z(),
          baseVels[i].linear().x(),
          baseVels[i].linear().y(),
          baseVels[i].linear().z()
        };
      }
      for(size_t j = 0; j < robot.refJointOrder().size(); ++j)
      {
        const auto & jN = robot.refJointOrder()[j];
        if(robot.hasJoint(jN))
        {
          auto jIndex = robot.jointIndexByName(jN);
          real_robot.mbc().q[jIndex][0] = encoders[j];
          real_robot.mbc().alpha[jIndex][0] = (encoders[j] - prevEncoders[j])/0.005;
        }
      }
      real_robot.posW(basePoses[i]);
      real_robot.forwardVelocity();
    }
    controller.setSensorAcceleration(accel.data);
  }

  bool setExternalForce(const std::string& body_respondable, const sva::ForceVecd& force)
  {
    external_force[body_respondable] = force;
    return true;
  }

  bool removeExternalForce(const std::string& body_respondable)
  {
    if(external_force.count(body_respondable))
    {
      external_force.erase(body_respondable);
      return true;
    }
    return false;
  }

  bool applyImpact(const std::string& body_respondable, const sva::ForceVecd& impact)
  {
    impact_force[body_respondable] = impact / controller.timestep();
    return true;
  }

  void nextSimulationStep()
  {
    float startT = vrep.getSimulationTime();
    static float prevT = startT - simulationTimestep;
    if(fabs(startT - prevT - simulationTimestep) > 1e-4)
    {
      std::cerr << "Missed a simulation step " << startT << " " << prevT << "\n";
    }
    prevT = startT;
    if(iter % frameskip == 0)
    {
      vrep.getSimulationState(joints, jQs, jTorques, fSensors, accel, gyro, baseNames, basePoses, baseVels);

      // Add external forces
      for(const auto& f : external_force)
      {
        vrep.addForce(f.first, f.second);
      }

      // apply impact forces
      for(const auto& f : impact_force)
      {
        vrep.addForce(f.first, f.second);
      }
      impact_force.clear();

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
        for(size_t i = 0; i < rIdx.size(); ++i)
        {
          auto & robot = controller.controller().robots().robot(rIdx[i]);
          const auto & suffix = suffixes[i];
          if(!torqueControl)
          {
            vrep.setRobotTargetConfiguration(robot.mb(), robot.mbc(), suffix);
          }
          else
          {
            vrep.setRobotTargetTorque(robot.mb(), robot.mbc(), suffix);
          }
        }
      }
    }
    iter++;
    float endT = vrep.getSimulationTime();
    if(endT != startT)
    {
      std::cerr << "One iteration occured while the simulation was running\n";
    }
    vrep.nextSimulationStep();
  }

  void stopSimulation()
  {
    vrep.stopSimulation();
  }
};

VREPSimulation::VREPSimulation(mc_control::MCGlobalController & controller, const VREPSimulationConfiguration & config)
: impl(new VREPSimulationImpl(controller, config.torqueControl, config.host, config.port, config.timeout, config.waitUntilConnected, config.doNotReconnect, config.commThreadCycleInMs, config.extras, config.simulationTimestep))
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

bool VREPSimulation::setExternalForce(const std::string& body_respondable, const sva::ForceVecd& force)
{
  return impl->setExternalForce(body_respondable, force);
}

bool VREPSimulation::removeExternalForce(const std::string& body_respondable)
{
  return impl->removeExternalForce(body_respondable);
}

bool VREPSimulation::applyImpact(const std::string& body_respondable, const sva::ForceVecd& impact)
{
  return impl->applyImpact(body_respondable, impact);
}
