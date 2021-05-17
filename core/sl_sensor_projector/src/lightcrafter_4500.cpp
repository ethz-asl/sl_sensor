#include "sl_sensor_projector/lightcrafter_4500.hpp"

#include <algorithm>
#include <iostream>
#include <iterator>

namespace sl_sensor
{
namespace projector
{
Lightcrafter4500::Lightcrafter4500()
{
}

Lightcrafter4500::~Lightcrafter4500()
{
  this->Close();
}

void Lightcrafter4500::LoadYaml(const std::string& yaml_directory)
{
  proj_config_ = YAML::LoadFile(yaml_directory);

  std::cout << "YAML file loaded successfully." << std::endl;
  std::cout << "Values:" << std::endl;
  std::cout << proj_config_ << std::endl;
}

bool Lightcrafter4500::Init()
{
  bool success = true;
  int status = projector_.Init();

  if (status < 0)
  {
    std::cout << "Error, failed to initialise projector" << std::endl;
    success = false;
  }

  return success;
}

bool Lightcrafter4500::Close()
{
  this->DisplayBlack();
  return (projector_.Close() < 0) ? false : true;
}

unsigned int Lightcrafter4500::GetExposurePeriod()
{
  return static_cast<unsigned int>(proj_config_["defaultExposureUs"] ? proj_config_["defaultExposureUs"].as<int>() :
                                                                       backup_exposure_period_us_);
}

int Lightcrafter4500::GetTriggerType()
{
  return static_cast<int>(proj_config_["properties"]["triggerType"] ?
                              proj_config_["properties"]["triggerType"].as<int>() :
                              backup_single_pattern_.trigger_type);
}

bool Lightcrafter4500::DisplayWhite()
{
  SetLed();

  std::vector<LightcrafterSinglePattern> pattern_vec = {};

  // Settings for white pattern (pattern number 24, bit depth 1, invert pattern)
  LightcrafterSinglePattern white_pattern;
  white_pattern.trigger_type = proj_config_["properties"]["triggerType"] ?
                                   proj_config_["properties"]["triggerType"].as<int>() :
                                   backup_single_pattern_.trigger_type;
  white_pattern.pattern_number = 24;
  white_pattern.bit_depth = 1;
  white_pattern.led_select = 7;
  white_pattern.image_indice = 0;
  white_pattern.invert_pattern = true;
  white_pattern.insert_black_frame = false;
  white_pattern.buffer_swap = true;
  white_pattern.trigger_out_prev = false;

  unsigned int exposure_period, frame_period;
  exposure_period = frame_period = GetExposurePeriod();

  pattern_vec.push_back(white_pattern);

  int status = projector_.PlayPatternSequence(pattern_vec, exposure_period, frame_period);

  return (status < 0) ? false : true;
}

bool Lightcrafter4500::DisplayBlack()
{
  SetLed();

  std::vector<LightcrafterSinglePattern> pattern_vec = {};

  // Settings for black pattern (pattern number 24, bit depth 1)
  LightcrafterSinglePattern black_pattern;
  black_pattern.trigger_type = proj_config_["properties"]["triggerType"] ?
                                   proj_config_["properties"]["triggerType"].as<int>() :
                                   backup_single_pattern_.trigger_type;
  black_pattern.pattern_number = 24;
  black_pattern.bit_depth = 1;
  black_pattern.led_select = 7;
  black_pattern.image_indice = 0;
  black_pattern.invert_pattern = false;
  black_pattern.insert_black_frame = false;
  black_pattern.buffer_swap = true;
  black_pattern.trigger_out_prev = false;

  unsigned int exposure_period, frame_period;
  exposure_period = frame_period = GetExposurePeriod();

  pattern_vec.push_back(black_pattern);

  int status = projector_.PlayPatternSequence(pattern_vec, exposure_period, frame_period);

  return (status < 0) ? false : true;
}

bool Lightcrafter4500::PatternExists(const std::string& pattern_name)
{
  if (!proj_config_["patterns"])
  {
    std::cout << "The 'patterns' field does not exist in the projector YAML file. Cannot search for pattern "
              << pattern_name << "." << std::endl;
    return false;
  }

  bool pattern_exists = false;

  for (YAML::const_iterator it = proj_config_["patterns"].begin(); it != proj_config_["patterns"].end(); ++it)
  {
    if (it->first.as<std::string>() == it->first.as<std::string>())
    {
      pattern_exists = true;
      break;
    }
  }

  return pattern_exists;
}

bool Lightcrafter4500::SetLed(const std::string& pattern_name)
{
  // Worst case senario we set  value to backup_rgb_ in header file
  unsigned char rgb[3];
  std::copy(std::begin(backup_rgb_), std::end(backup_rgb_), std::begin(rgb));

  // Case where a valid pattern name is entered
  if (!pattern_name.empty())
  {
    // If pattern name does not exist in YAML file we set as fail
    if (!PatternExists(pattern_name))
    {
      return false;
    }

    // If pattern node has the rgb field we set those values, set as fail otherwise
    if (proj_config_["patterns"][pattern_name]["rgb"] && proj_config_["patterns"][pattern_name]["rgb"].size() == 3)
    {
      auto rgb_vec = proj_config_["patterns"][pattern_name]["rgb"].as<std::vector<unsigned char>>();
      std::copy(rgb_vec.begin(), rgb_vec.end(), std::begin(rgb));
    }
    else
    {
      return false;
    }
  }
  // Case where empty pattern name is entered, try searching yaml for default rgb
  else if (proj_config_["defaultSettings"]["defaultRgb"] && proj_config_["defaultSettings"]["defaultRgb"].size() == 3)
  {
    auto rgb_vec = proj_config_["defaultSettings"]["defaultRgb"].as<std::vector<unsigned char>>();
    std::copy(rgb_vec.begin(), rgb_vec.end(), std::begin(rgb));
  }

  int status = projector_.SetLedCurrents(rgb[0], rgb[1], rgb[2]);

  return (status < 0) ? false : true;
}

bool Lightcrafter4500::ProjectSinglePattern(const std::string& pattern_name, int pattern_indice)
{
  if (!PatternExists(pattern_name))
  {
    return false;
  }

  SetLed(pattern_name);

  std::vector<LightcrafterSinglePattern> pattern_vec = {};
  LightcrafterSinglePattern single_pattern = GetSinglePattern(pattern_name, pattern_indice);
  pattern_vec.push_back(single_pattern);

  unsigned int exposure_period, frame_period;
  exposure_period = frame_period = proj_config_["patterns"][pattern_name]["exposureUs"] ?
                                       proj_config_["patterns"][pattern_name]["exposureUs"].as<unsigned int>() :
                                       backup_exposure_period_us_;

  // std::cout << "Exposure: " << exposure_period << std::endl;

  int status = projector_.PlayPatternSequence(pattern_vec, exposure_period, frame_period);

  return (status < 0) ? false : true;
};

bool Lightcrafter4500::ProjectFullPattern(const std::string& pattern_name)
{
  return false;
}

std::vector<LightcrafterSinglePattern> Lightcrafter4500::GetPatternSequence(const std::string& pattern_name)
{
  std::vector<LightcrafterSinglePattern> temp;
  return temp;
}

LightcrafterSinglePattern Lightcrafter4500::GetSinglePattern(const std::string& pattern_name, int pattern_indice)
{
  LightcrafterSinglePattern single_pattern;

  single_pattern.trigger_type = GetTriggerType();
  single_pattern.pattern_number =
      proj_config_["patterns"][pattern_name]["channelNumbers"][pattern_indice] ?
          proj_config_["patterns"][pattern_name]["channelNumbers"][pattern_indice].as<int>() :
          backup_single_pattern_.pattern_number;
  single_pattern.bit_depth = proj_config_["patterns"][pattern_name]["bitDepth"] ?
                                 proj_config_["patterns"][pattern_name]["bitDepth"].as<int>() :
                                 backup_single_pattern_.bit_depth;
  single_pattern.led_select = proj_config_["patterns"][pattern_name]["displayColour"] ?
                                  proj_config_["patterns"][pattern_name]["displayColour"].as<int>() :
                                  backup_single_pattern_.led_select;
  single_pattern.image_indice = proj_config_["patterns"][pattern_name]["imageIndices"][pattern_indice] ?
                                    proj_config_["patterns"][pattern_name]["imageIndices"][pattern_indice].as<int>() :
                                    backup_single_pattern_.image_indice;
  single_pattern.invert_pattern = backup_single_pattern_.invert_pattern;
  single_pattern.insert_black_frame = backup_single_pattern_.insert_black_frame;
  single_pattern.buffer_swap = backup_single_pattern_.buffer_swap;
  single_pattern.trigger_out_prev = backup_single_pattern_.trigger_out_prev;

  // std::cout << single_pattern << std::endl;

  return single_pattern;
}

}  // namespace projector
}  // namespace sl_sensor