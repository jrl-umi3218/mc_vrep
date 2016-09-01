#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/config.h>

struct VREPSimulationImpl;

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
   * \param host See vrep::VREP documentation
   * \param port See vrep::VREP documentation
   * \param timeout See vrep::VREP documentation
   * \param waitUntilConnected See vrep::VREP documentation
   * \param doNotReconnect See vrep::VREP documentation
   * \param commThreadCycleInMs See vrep::VREP documentation
   */
  VREPSimulation(mc_control::MCGlobalController & controller, const std::string & host = "127.0.0.1", int port = 19997, int timeout = 3000, bool waitUntilConnected = true, bool doNotReconnect = true, int commThreadCycleInMs = 1);

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
private:
  std::unique_ptr<VREPSimulationImpl> impl;
};
