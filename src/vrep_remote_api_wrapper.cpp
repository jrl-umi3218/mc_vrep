#include "vrep_remote_api_wrapper.h"

extern "C"
{
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpedantic"
  #include "vrep_remote_api/extApi.h"
  #include "vrep_remote_api/extApiCustom.h"
  #pragma GCC diagnostic pop
}

#include <mc_rtc/logging.h>

#include <array>
#include <fstream>
#include <iostream>
#include <map>

struct VREPForceSensor
{
  int status = 0;
  Eigen::Vector3d force;
  Eigen::Vector3d torque;
};

struct VREPRemoteAPIWrapperImpl
{
  VREPRemoteAPIWrapperImpl(const std::string & robot_name)
  : log("/tmp/mc_vrep.log")
  {
    if(robot_name == "hrp2_drc")
    {
      REF_JOINT_ORDER = {
  "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2", "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
  "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2", "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5",
  "CHEST_JOINT0", "CHEST_JOINT1", "HEAD_JOINT0", "HEAD_JOINT1",
  "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6", "RARM_JOINT7",
  "LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6", "LARM_JOINT7",
  "RHAND_JOINT0", "RHAND_JOINT1", "RHAND_JOINT2", "RHAND_JOINT3", "RHAND_JOINT4",
  "LHAND_JOINT0", "LHAND_JOINT1", "LHAND_JOINT2", "LHAND_JOINT3", "LHAND_JOINT4"};

      MAIN_JOINTS = {
  "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2", "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
  "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2", "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5",
  "CHEST_JOINT0", "CHEST_JOINT1", "HEAD_JOINT0", "HEAD_JOINT1",
  "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6",
  "LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6"
};

      RGRIPPER_JOINTS = {
  "RARM_JOINT7", "RHAND_JOINT0", "RHAND_JOINT1", "RHAND_JOINT2", "RHAND_JOINT3", "RHAND_JOINT4"
};

      LGRIPPER_JOINTS = {
  "LARM_JOINT7", "LHAND_JOINT0", "LHAND_JOINT1", "LHAND_JOINT2", "LHAND_JOINT3", "LHAND_JOINT4"
};
    }
    else if(robot_name == "hrp4")
    {
      REF_JOINT_ORDER = {
  //"Root"
  "R_HIP_Y", "R_HIP_R","R_HIP_P", "R_KNEE_P", "R_ANKLE_P", "R_ANKLE_R",
  "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE_P", "L_ANKLE_P", "L_ANKLE_R",
  "CHEST_P", "CHEST_Y", "NECK_Y", "NECK_P", 
  "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P", "R_WRIST_Y", "R_WRIST_P", "R_WRIST_R", 
  "R_HAND_J0", "R_HAND_J1", "R_F22", "R_F23", "R_F32", "R_F33", "R_F42", "R_F43", "R_F52", "R_F53", 
  "L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y", "L_ELBOW_P", "L_WRIST_Y", "L_WRIST_P", "L_WRIST_R", 
  "L_HAND_J0", "L_HAND_J1", "L_F22", "L_F23", "L_F32", "L_F33", "L_F42", "L_F43", "L_F52", "L_F53"
};

      MAIN_JOINTS = {"R_HIP_Y", "R_HIP_R","R_HIP_P", "R_KNEE_P", "R_ANKLE_P", "R_ANKLE_R",
  "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE_P", "L_ANKLE_P", "L_ANKLE_R",
  "CHEST_P", "CHEST_Y", "NECK_Y", "NECK_P", 
  "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P", "R_WRIST_Y", "R_WRIST_P", "R_WRIST_R", 
  "L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y", "L_ELBOW_P", "L_WRIST_Y", "L_WRIST_P", "L_WRIST_R"
};

      RGRIPPER_JOINTS = {
  "R_HAND_J0", "R_HAND_J1", "R_F22", "R_F23", "R_F32", "R_F33", "R_F42", "R_F43", "R_F52", "R_F53"
};

      LGRIPPER_JOINTS = {
  "L_HAND_J0", "L_HAND_J1", "L_F22", "L_F23", "L_F32", "L_F33", "L_F42", "L_F43", "L_F52", "L_F53"
};
    }
    else
    {
      LOG_ERROR("Do not know how to send commands to " << robot_name)
      throw("Unsupported robot");
    }
  }
  /* Store client id */
  int cId = -1;
  std::map<std::string, float> joints = {};
  std::map<std::string, VREPForceSensor> sensors = {};
  /* Default communication mode with the API */
  const static int opmode = simx_opmode_oneshot_wait;
  const static int opmode_streaming = simx_opmode_streaming;
  const static int opmode_buffer = simx_opmode_buffer;
  std::vector<std::string> string_data_to_vec()
  {
    std::vector<std::string> res;
    unsigned int j = 0;
    for(int i = 0; i < string_data_c; ++i)
    {
      std::stringstream ss;
      while(string_data[j] != '\0')
      {
        ss << string_data[j];
        ++j;
      }
      res.push_back(ss.str());
      ++j;
    }
    return res;
  }
  void get_handles(int object_type, const std::vector<std::string> & names)
  {
    int handle_data_c; int * handle_data = NULL;
    simxGetObjectGroupData(cId, object_type, 0,
                           &handle_data_c, &handle_data,
                           NULL, NULL, NULL, NULL,
                           &string_data_c, &string_data, opmode);
    auto simNames = string_data_to_vec();
    for(const auto & n : names)
    {
      size_t i = 0;
      for(i = 0; i < simNames.size(); ++i)
      {
        if(simNames[i] == n)
        {
          handles[n] = handle_data[i];
          this->names[handle_data[i]] = n;
          break;
        }
      }
      if(i == simNames.size())
      {
        LOG_WARNING("No object named " << n << " in VREP simulation")
      }
    }
  }
  void joint_handles(const std::vector<std::string> & jnames)
  {
    get_handles(sim_object_joint_type, jnames);
    for(const auto & jn : jnames)
    {
      if(handles.count(jn))
      {
        joints[jn] = 0;
      }
      else
      {
        LOG_WARNING("No joint named " << jn << " in VREP simulation")
      }
    }
  }
  void sensor_handles(const std::vector<std::string> & snames)
  {
    get_handles(sim_object_forcesensor_type, snames);
    for(const auto & sn : snames)
    {
      if(handles.count(sn))
      {
        sensors[sn] = VREPForceSensor();
      }
    }
  }

  void init()
  {
    simxGetObjectGroupData(cId, sim_object_joint_type, 15,
                           &joint_handle_c, &joint_handle,
                           NULL, NULL,
                           &joint_data_c, &joint_data,
                           NULL, NULL, opmode_streaming);
    simxGetObjectGroupData(cId, sim_object_forcesensor_type, 14,
                           &force_handle_c, &force_handle,
                           &force_status_data_c, &force_status_data,
                           &force_data_c, &force_data,
                           NULL, NULL, opmode_streaming);
  }

  void update()
  {
    /* Read joint data */
    simxGetObjectGroupData(cId, sim_object_joint_type, 15,
                           &joint_handle_c, &joint_handle,
                           NULL, NULL,
                           &joint_data_c, &joint_data,
                           NULL, NULL, opmode_buffer);
    for(int i = 0; i < joint_handle_c; ++i)
    {
      int h = joint_handle[i];
      if(names.count(h) && joints.count(names[h]))
      {
        joints[names[h]] = joint_data[2*i];
      }
    }
    /* Read force sensor data */
    simxGetObjectGroupData(cId, sim_object_forcesensor_type, 14,
                           &force_handle_c, &force_handle,
                           &force_status_data_c, &force_status_data,
                           &force_data_c, &force_data,
                           NULL, NULL, opmode_buffer);
    for(int i = 0; i < force_handle_c; ++i)
    {
      int h = force_handle[i];
      if(names.count(h) && sensors.count(names[h]))
      {
        auto & s = sensors[names[h]];
        s.status = force_status_data[i];
        s.force.x() = force_data[6*i+0];
        s.force.y() = force_data[6*i+1];
        s.force.z() = force_data[6*i+2];
        s.torque.x() = force_data[6*i+3];
        s.torque.y() = force_data[6*i+4];
        s.torque.z() = force_data[6*i+5];
      }
    }
  }

  std::vector<double> qIn()
  {
    if(qOut_started)
    {
      log << ++iter;
    }
    std::vector<double> res(REF_JOINT_ORDER.size());
    for(size_t i = 0; i < REF_JOINT_ORDER.size(); ++i)
    {
      res[i] = joints[REF_JOINT_ORDER[i]];
      if(qOut_started)
      {
        log << " " << res[i];
      }
    }
    return res;
  }

  typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> wrench_t;
  std::vector<wrench_t> wrenches()
  {
    std::vector<wrench_t> res;
    for(const auto & fs : sensors)
    {
      res.push_back({fs.second.force, fs.second.torque});
    }
    return res;
  }

  void qOut(mc_control::MCGlobalController & controller)
  {
    const mc_control::QPResultMsg & res = controller.send(0);
    if(!qOut_started)
    {
      qOut_handles.resize(MAIN_JOINTS.size() + 2);
      qOut_positions.resize(MAIN_JOINTS.size() + 2);
    }
    for(size_t i = 0; i < MAIN_JOINTS.size(); ++i)
    {
      const std::string & jn = MAIN_JOINTS[i];
      qOut_handles[i] = handles[jn];
      qOut_positions[i] = static_cast<float>(res.robots_state[0].q[6 + controller.robot().jointIndexByName(jn)]);
      if(qOut_started)
      {
        log << " " << qOut_positions[i];
        if(jn == "RARM_JOINT6")
        {
          log << " " << controller.gripperQ(false)[0];
        }
        if(jn == "LARM_JOINT6")
        {
          log << " " << controller.gripperQ(true)[0] << std::endl;
        }
      }
    }
    log << std::endl;
    if(controller.robot().name() == "hrp2_drc")
    {
      qOut_handles[MAIN_JOINTS.size()] = handles["RARM_JOINT7"];
      qOut_positions[MAIN_JOINTS.size()] = controller.gripperQ(false)[0];
      qOut_handles[MAIN_JOINTS.size() + 1] = handles["LARM_JOINT7"];
      qOut_positions[MAIN_JOINTS.size() + 1] = controller.gripperQ(true)[0];
    }
    simxCustomSetJointsTargetPositions(cId,
                                       static_cast<simxInt>(qOut_handles.size()),
                                       qOut_handles.data(),
                                       qOut_positions.data(),
                                       qOut_started?opmode_streaming:opmode_buffer);
    qOut_started = true;
  }
private:
  /* Store name to handle */
  std::map<std::string, int> handles = {};
  /* Store handle to name */
  std::map<int, std::string> names = {};

  /* Used to retrieve data returned by the remote API */
  int joint_handle_c = 0; int * joint_handle = 0;
  int joint_data_c = 0; float * joint_data = 0;
  int force_handle_c = 0; int * force_handle = 0;
  int force_status_data_c = 0; int * force_status_data = 0;
  int force_data_c = 0; float * force_data = 0;
  int string_data_c = 0; char * string_data = 0;

  /* Used to send data to the remote API */
  bool qOut_started = false;
  std::vector<int> qOut_handles;
  std::vector<float> qOut_positions;

  /* Log data */
  unsigned int iter = 0;
  std::ofstream log;
  std::vector<std::string> REF_JOINT_ORDER;
  std::vector<std::string> MAIN_JOINTS;
  std::vector<std::string> RGRIPPER_JOINTS;
  std::vector<std::string> LGRIPPER_JOINTS;
};

VREPRemoteAPIWrapper::VREPRemoteAPIWrapper(const std::string & main_robot, const std::string & host, int port, int timeout, bool waitUntilConnected, bool doNotReconnect, int commThreadCycleInMs): impl(new VREPRemoteAPIWrapperImpl(main_robot))
{
  impl->cId = simxStart(host.c_str(), port, waitUntilConnected ? 1 : 0, doNotReconnect ? 1 : 0, timeout, commThreadCycleInMs);
  if(impl->cId >= 0)
  {
    LOG_SUCCESS("Connected to VREP")
  }
  else
  {
    LOG_ERROR("Failed to connect to VREP")
    throw("Failed to connect to VREP");
  }
  simxSynchronous(impl->cId, 1);
}

VREPRemoteAPIWrapper::~VREPRemoteAPIWrapper()
{
  simxFinish(impl->cId);
}

void VREPRemoteAPIWrapper::startSimulation(mc_control::MCGlobalController & controller)
{
  const mc_rbdyn::Robot & robot = controller.robot();
  /* Find all joints handle and start the position streaming */
  std::vector<std::string> jnames;
  for(const auto & j : robot.mb().joints())
  {
    jnames.push_back(j.name());
  }
  impl->joint_handles(jnames);
  impl->sensor_handles(robot.forceSensorsByName());
  LOG_SUCCESS("Starting simulation")
  impl->init();
  simxStartSimulation(impl->cId, impl->opmode);

  /* Run one step of the simulation to get the data flowing */
  controller.running = false;
  impl->update();
  simxSynchronousTrigger(impl->cId);
  LOG_SUCCESS("Simulation started")
}

void VREPRemoteAPIWrapper::nextSimulationStep(mc_control::MCGlobalController & controller)
{
  /* Update sensor information */
  impl->update();
  std::vector<double> qIn = impl->qIn();
  if(!controller.running)
  {
    controller.init(qIn);
    controller.running = true;
  }
  else
  {
    controller.setEncoderValues(qIn);
  }
  if(controller.robot().name() == "hrp2_drc")
  {
    controller.setActualGripperQ(qIn[23], qIn[31]);
  }
  /* Send the update to the controller */
  controller.setWrenches(impl->wrenches());
  /* Run */
  if(controller.run())
  {
    impl->qOut(controller);
  }
  else
  {
    simxStopSimulation(impl->cId, impl->opmode);
    std::exit(1);
  }
  /* Trigger next simulation step */
  simxSynchronousTrigger(impl->cId);
}

void VREPRemoteAPIWrapper::stopSimulation()
{
  simxStopSimulation(impl->cId, impl->opmode);
}
