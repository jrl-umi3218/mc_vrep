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
  sva::MotionVecd baseVel;

  /* Gripper related */
  std::map<std::string, std::vector<size_t>> gripper_in_index;
  std::map<std::string, std::vector<double>> realGripperQs;

  std::map<std::string, sva::ForceVecd> external_force;
  std::map<std::string, sva::ForceVecd> impact_force;

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
                          [this,gui,&ctl](const mc_rtc::Configuration & data)
                          {
                            std::string body = data("Body");
                            std::string msg = "addForce " + body; 
                            ctl.read_msg(msg);
                            Eigen::Vector6d force = data("Force");
                            setExternalForce(body, {force});
                            gui->addElement({"VREP", "Force"},
                              mc_rtc::gui::Button("Remove force on " + body,
                                                  [this,body,gui,&ctl]()
                                                  {
                                                    std::string msg = "removeForce " + body; 
                                                    ctl.read_msg(msg);
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
  }

  typedef sva::ForceVecd wrench_t;
  std::map<std::string, wrench_t> wrenches()
  {
    std::map<std::string, wrench_t> res;
    for(const auto & fs : fSensors)
    {
      res.emplace(fs.first, wrench_t{fs.second.torque, fs.second.force});
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
    for(const auto & fs : robot.forceSensors())
    {
      fSensors[fs.name()] = {};
    }
    vrep.startSimulation(baseName,
                         controller.ref_joint_order(),
                         fSensors);
    /* Run simulation until the data arrives */
    while(! (vrep.getJointsData(controller.ref_joint_order(), jQs, jTorques) &&
             vrep.getSensorData(fSensors, accel, gyro) &&
             vrep.getBasePos(baseName, basePos) &&
             vrep.getBaseVelocity(baseName, baseVel)) )
    {
      vrep.nextSimulationStep();
    }
    for(size_t i = 0; i < 10; ++i)
    {
      vrep.nextSimulationStep();
    }
    controller.running = true;
    Eigen::Vector3d t(basePos.translation());
    Eigen::Quaterniond q(basePos.rotation().transpose());
    std::array<double, 7> initAttitude {{q.w(), q.x(), q.y(), q.z(), t.x(), t.y(), t.z()}};
    updateData();
    controller.init(jQs, initAttitude);
    LOG_SUCCESS("Simulation started")
  }

  void updateData()
  {
    controller.setSensorPosition(basePos.translation());
    controller.setSensorOrientation(Eigen::Quaterniond(basePos.rotation()));
    controller.setSensorLinearVelocity(baseVel.linear());
    controller.setSensorAngularVelocity(gyro.data);
    controller.setSensorAcceleration(accel.data);
    controller.setEncoderValues(jQs);
    controller.setWrenches(wrenches());
    controller.setJointTorques(jTorques);
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
    vrep.getJointsData(controller.ref_joint_order(), jQs, jTorques);
    vrep.getSensorData(fSensors, accel, gyro);
    vrep.getBasePos(baseName, basePos);
    vrep.getBaseVelocity(baseName, baseVel);

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
