#pragma once

#include "sl_sensor_projector/lightcrafter_4500_api.hpp"
#include "sl_sensor_projector/lightcrafter_single_pattern.hpp"

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

namespace sl_sensor
{
namespace projector
{
class Lightcrafter4500
{
public:
  Lightcrafter4500();
  ~Lightcrafter4500();

  void LoadYaml(const std::string& yaml_directory);

  bool Init();

  bool Close();

  bool DisplayWhite();

  bool DisplayBlack();

  bool PatternExists(const std::string& pattern_name);

  bool ProjectSinglePattern(const std::string& pattern_name, int pattern_indice);

  bool ProjectFullPattern(const std::string& pattern_name);

  std::vector<LightcrafterSinglePattern> GetPatternSequence(const std::string& pattern_name);

  LightcrafterSinglePattern GetSinglePattern(const std::string& pattern_name, int pattern_indice);

  unsigned int GetExposurePeriod();

  int GetTriggerType();

  bool SetLed(const std::string& pattern_name = "");

private:
  Lightcrafter4500Api projector_;
  YAML::Node proj_config_;
  unsigned int backup_exposure_period_us_ = 1000;
  LightcrafterSinglePattern backup_single_pattern_ =
      LightcrafterSinglePattern(0, 0, 8, 7, 0, false, false, true, false);
  unsigned char backup_rgb_[3] = { 104, 135, 130 };
};

}  // namespace projector
}  // namespace sl_sensor