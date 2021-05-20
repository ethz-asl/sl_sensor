#include "sl_sensor_projector/SendProjectorCommand.h"
#include "sl_sensor_projector/lightcrafter_4500.hpp"

#include <ros/ros.h>
#include <string>

using namespace sl_sensor::projector;

Lightcrafter4500 projector;

bool SendProjectorCommand(sl_sensor_projector::SendProjectorCommand::Request &req,
                          sl_sensor_projector::SendProjectorCommand::Response &res)
{
  // auto command_const_char(req.command.c_str());
  std::string command(req.command);
  int pattern_no = req.pattern_no;

  if (command == "white")
  {
    res.success = projector.DisplayWhite();
  }
  else if (command == "black")
  {
    res.success = projector.DisplayBlack();
  }
  else if (!command.empty())
  {
    if (pattern_no >= 0)
    {
      res.success = projector.ProjectSinglePattern(command, pattern_no);
    }
    else
    {
      res.success = projector.ProjectFullPattern(command);
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lightcrafter_4500_projector");
  ros::NodeHandle nh;

  std::string projector_yaml_directory;
  nh.param<std::string>("projector_yaml_directory", projector_yaml_directory, projector_yaml_directory);

  ros::ServiceServer service = nh.advertiseService("send_projector_command", SendProjectorCommand);

  projector.LoadYaml(projector_yaml_directory);
  bool init_success = projector.Init();

  if (!init_success)
  {
    ROS_INFO("Projector initialisation failed! Terminating node.");
    return 0;
  }
  else
  {
    ROS_INFO("Projector initialisation succeeded! You can start sending commands");
  }

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}