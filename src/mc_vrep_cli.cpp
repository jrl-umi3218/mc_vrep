#include "mc_vrep_cli.h"

#include <mc_rtc/logging.h>

#include <boost/algorithm/string/trim.hpp>

#include <iostream>
#include <string>
#include <sstream>

/* Anonymous namespace to hold the CLI functions */
namespace
{
  bool open_grippers(mc_control::MCGlobalController & controller, std::stringstream&)
  {
    controller.setGripperOpenPercent(1);
    return true;
  }

  bool close_grippers(mc_control::MCGlobalController & controller, std::stringstream&)
  {
    controller.setGripperOpenPercent(0);
    return true;
  }

  bool set_gripper(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    std::string gripper; std::vector<double> v; double tmp;
    args >> gripper;
    while(args.good())
    {
      args >> tmp;
      v.push_back(tmp);
    }
    controller.setGripperTargetQ(gripper, v);
    return true;
  }

  bool set_joint_pos(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    std::string jn;
    double v;
    args >> jn >> v;
    return controller.set_joint_pos(jn, v);
  }

  bool get_joint_pos(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    std::string jn;
    args >> jn;
    if (controller.robot().hasJoint(jn))
    {
      std::cout << jn << ": " << controller.robot().mbc().q[controller.robot().jointIndexByName(jn)][0] << std::endl;
    }
    else
    {
      std::cout << "No joint named " << jn << " in the robot" << std::endl;
    }
    return true;

  }

  bool move_com(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    double x, y, z = 0;
    args >> x >> y >> z;
    return controller.move_com(Eigen::Vector3d(x,y,z));
  }

  bool play_next_stance(mc_control::MCGlobalController & controller, std::stringstream &)
  {
    return controller.play_next_stance();
  }

  bool send_msg(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    return controller.send_msg(args.str());
  }

  bool send_recv_msg(mc_control::MCGlobalController & controller, std::stringstream & args)
  {
    std::string out;
    bool r = controller.send_recv_msg(args.str(), out);
    LOG_INFO("Controller response:" << std::endl << out)
    return r;
  }

  std::map<std::string, std::function<bool(mc_control::MCGlobalController&, std::stringstream&)>> cli_fn = {
    {"set_joint_pos", std::bind(&set_joint_pos, std::placeholders::_1, std::placeholders::_2)},
    {"get_joint_pos", std::bind(&get_joint_pos, std::placeholders::_1, std::placeholders::_2) },
    {"open_grippers", std::bind(&open_grippers, std::placeholders::_1, std::placeholders::_2)},
    {"close_grippers", std::bind(&close_grippers, std::placeholders::_1, std::placeholders::_2)},
    {"set_gripper", std::bind(&set_gripper, std::placeholders::_1, std::placeholders::_2)},
    {"move_com", std::bind(&move_com, std::placeholders::_1, std::placeholders::_2)},
    {"play_next_stance", std::bind(&play_next_stance, std::placeholders::_1, std::placeholders::_2)},
    {"send_msg", std::bind(&send_msg, std::placeholders::_1, std::placeholders::_2)},
    {"send_recv_msg", std::bind(&send_recv_msg, std::placeholders::_1, std::placeholders::_2)}
  };
}

MCVREPCLI::MCVREPCLI(mc_control::MCGlobalController & controller)
: controller(controller)
{
}

void MCVREPCLI::run()
{
  while(!done_)
  {
    std::string ui;
    std::getline(std::cin, ui);
    std::stringstream ss;
    ss << ui;
    std::string token;
    ss >> token;
    if(token == "stop")
    {
      LOG_INFO("Stopping simulation")
      done_ = true;
    }
    else if(cli_fn.count(token))
    {
      std::string rem;
      std::getline(ss, rem);
      boost::algorithm::trim(rem);
      std::stringstream ss2;
      ss2 << rem;
      bool ret = cli_fn[token](controller, ss2);
      if (!ret)
      {
        std::cerr << "Failed to invoke the previous command" << std::endl;
      }
    }
    else
    {
      std::cerr << "Unkwown command " << token << std::endl;
    }
  }
}

bool MCVREPCLI::done() const
{
  return done_;
}
