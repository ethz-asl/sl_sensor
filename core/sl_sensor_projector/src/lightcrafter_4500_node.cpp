#include "sl_sensor_projector/CommandProjector.h"
#include "sl_sensor_projector/lightcrafter_4500.hpp"

#include <ros/ros.h>
#include <string>

using namespace sl_sensor::projector;

Lightcrafter4500 projector;

/**
 * @brief Process command projector service call
 * Usage:
 * command - "white", "black" or a pattern name. White and black will display a bright and dark projection respecitvely.
 * pattern_no - Indice of the projection in the pattern sequenc to be displayed.
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool CommandProjector(sl_sensor_projector::CommandProjector::Request &req,
                      sl_sensor_projector::CommandProjector::Response &res)
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
  std::string service_name;
  nh.param<std::string>("projector_yaml_directory", projector_yaml_directory, projector_yaml_directory);
  nh.param<std::string>("projector_service_name", service_name, service_name);

  projector.LoadYaml(projector_yaml_directory);
  bool init_success = projector.Init();

  bool empty_service_name = service_name.empty();

  if (!init_success)
  {
    ROS_WARN("[Lightcrafter4500Node] Projector initialisation failed! Terminating node.");
  }
  else if (empty_service_name)
  {
    ROS_WARN("[Lightcrafter4500Node] No projector service name provided in YAML file! Terminating node.");
  }
  else
  {
    ros::ServiceServer service = nh.advertiseService(service_name, CommandProjector);

    ROS_INFO("[Lightcrafter4500Node] Projector initialisation succeeded! You can start sending commands");

    while (ros::ok())
    {
      ros::spinOnce();
    }
  }

  return 0;
}