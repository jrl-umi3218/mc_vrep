#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>

struct VREPSimulationImpl;

/** Configuration for the connection to VREP and the simulation */
struct VREPSimulationConfiguration
{
  /** See vrep::VREP documentation */
  std::string host = "127.0.0.1";
  /** See vrep::VREP documentation */
  int port = 19997;
  /** See vrep::VREP documentation */
  int timeout = 3000;
  /** See vrep::VREP documentation */
  bool waitUntilConnected = true;
  /** See vrep::VREP documentation */
  bool doNotReconnect = true;
  /** See vrep::VREP documentation */
  int commThreadCycleInMs = 1;
  /** Simulation timestep, defaults to the controller timestep if not provided */
  double simulationTimestep = -1;
  /** If true, run the simulation step by step */
  bool stepByStep = false;
  /** If true, use computed torques as control input rather than joint position */
  bool torqueControl = false;
  /** Configuration for extra-robots */
  struct ExtraRobot
  {
    unsigned int index;
    std::string suffix;
  };
  /** Suffix to apply given a robot index */
  std::vector<ExtraRobot> extras = {};
};

/*! \brief This class is a thin wrapper around the
 * vrep-api-wrapper to drive a simulation in VREP using the
 * mc_rtc control framework.
 */
struct VREPSimulation
{
public:
  /*! \brief Constructor
   *
   * Prepare to start a simulation.
   *
   * \param controller The mc_rtc controller instance used in the simulation.
   * \param config See VREPSimulationConfiguration
   *
   */
  VREPSimulation(mc_control::MCGlobalController & controller, const VREPSimulationConfiguration & config);

  /*! \brief Destructor */
  ~VREPSimulation();

  /*! \brief Start the simulation. This should be called
   * only once
   */
  void startSimulation();

  /*! Trigger the next simulation step. This should be
   * called as long as the simulation is running.
   */
  void nextSimulationStep();

  /*! Stop the simulation */
  void stopSimulation();

  /*! Apply an external force to a respondable body. The force will be applied at
   * each iteration until it is explicitely removed.
   *
   * \param body_respondable Name of respondable body in V-REP scene
   *
   * \param force External force in world frame
   *
   */
  bool setExternalForce(const std::string& body_respondable, const sva::ForceVecd& force);

  /*! Remove external force applied on a respondable body.
   *
   * \param body_respondable Name of respondable body in V-REP scene
   *
   */
  bool removeExternalForce(const std::string& body_respondable);

  /*! Add an external impact force. The force will be applied for one iteration only,
   * then removed immediately.
   *
   * \param body_respondable Name of respondable body in V-REP scene
   *
   * \param impact Impact vector in world frame (impact = force * dt where dt is the
   * simulation time step)
   *
   */
  bool applyImpact(const std::string& body_respondable, const sva::ForceVecd& impact);

private:
  std::unique_ptr<VREPSimulationImpl> impl;
};
