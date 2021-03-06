/***************************************************************************************************
 * This file is part of sl_sensor.
 *
 * sl_sensor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * sl_sensor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with sl_sensor.  If not, see <https://www.gnu.org/licenses/>.
 ***************************************************************************************************/

#include "sl_sensor_projector/lightcrafter_4500.hpp"

#include <algorithm>
#include <iostream>
#include <iterator>

namespace sl_sensor {
namespace projector {
Lightcrafter4500::Lightcrafter4500() {}

Lightcrafter4500::~Lightcrafter4500() { this->Close(); }

void Lightcrafter4500::LoadYaml(const std::string& yaml_directory) {
  proj_config_ = YAML::LoadFile(yaml_directory);

  std::cout << "YAML file loaded successfully." << std::endl;
  std::cout << "Values:" << std::endl;
  std::cout << proj_config_ << std::endl;
}

bool Lightcrafter4500::Init() {
  bool success = true;
  int status = projector_.Init();

  if (status < 0) {
    std::cout << "Error, failed to initialise projector" << std::endl;
    success = false;
  }

  return success;
}

bool Lightcrafter4500::Close() {
  this->DisplayBlack();
  return (projector_.Close() < 0) ? false : true;
}

unsigned int Lightcrafter4500::GetDefaultExposurePeriod() {
  return static_cast<unsigned int>(proj_config_["default_exposure_us"]
                                       ? proj_config_["default_exposure_us"].as<int>()
                                       : backup_exposure_period_us_);
}

bool Lightcrafter4500::DisplayWhite() {
  SetLed();

  std::vector<LightcrafterSinglePattern> pattern_vec = {};

  // Settings for white pattern (pattern number 24, bit depth 1, invert pattern)
  LightcrafterSinglePattern white_pattern;
  white_pattern.trigger_type = 0;
  white_pattern.pattern_number = 24;
  white_pattern.bit_depth = 1;
  white_pattern.led_select = 7;
  white_pattern.image_indice = 0;
  white_pattern.invert_pattern = true;
  white_pattern.insert_black_frame = false;
  white_pattern.buffer_swap = true;
  white_pattern.trigger_out_prev = false;

  unsigned int exposure_period, frame_period;
  exposure_period = frame_period = GetDefaultExposurePeriod();

  pattern_vec.push_back(white_pattern);

  int status = projector_.PlayPatternSequence(pattern_vec, exposure_period, frame_period);

  return (status < 0) ? false : true;
}

bool Lightcrafter4500::DisplayBlack() {
  SetLed();

  std::vector<LightcrafterSinglePattern> pattern_vec = {};

  // Settings for black pattern (pattern number 24, bit depth 1)
  LightcrafterSinglePattern black_pattern;
  black_pattern.trigger_type = 0;
  black_pattern.pattern_number = 24;
  black_pattern.bit_depth = 1;
  black_pattern.led_select = 7;
  black_pattern.image_indice = 0;
  black_pattern.invert_pattern = false;
  black_pattern.insert_black_frame = false;
  black_pattern.buffer_swap = true;
  black_pattern.trigger_out_prev = false;

  unsigned int exposure_period, frame_period;
  exposure_period = frame_period = GetDefaultExposurePeriod();

  pattern_vec.push_back(black_pattern);

  int status = projector_.PlayPatternSequence(pattern_vec, exposure_period, frame_period);

  return (status < 0) ? false : true;
}

bool Lightcrafter4500::PatternExists(const YAML::Node& config, const std::string& pattern_name) {
  if (!config["patterns"]) {
    std::cout << "The 'patterns' field does not exist in the projector YAML file. Cannot search "
                 "for pattern "
              << pattern_name << "." << std::endl;
    return false;
  }

  bool pattern_exists = false;

  for (YAML::const_iterator it = config["patterns"].begin(); it != config["patterns"].end(); ++it) {
    if (pattern_name == it->first.as<std::string>()) {
      pattern_exists = true;
      break;
    }
  }

  return pattern_exists;
}

bool Lightcrafter4500::PatternExists(const std::string& pattern_name) {
  return PatternExists(proj_config_, pattern_name);
}

bool Lightcrafter4500::SetLed(const std::string& pattern_name) {
  // Worst case senario we set  value to backup_rgb_ in header file
  unsigned char rgb[3];
  std::copy(std::begin(backup_rgb_), std::end(backup_rgb_), std::begin(rgb));

  // Case where a valid pattern name is entered
  if (!pattern_name.empty()) {
    // If pattern name does not exist in YAML file we set as fail
    if (!PatternExists(pattern_name)) {
      return false;
    }

    // If pattern node has the rgb field we set those values, set as fail otherwise
    if (proj_config_["patterns"][pattern_name]["rgb"] &&
        proj_config_["patterns"][pattern_name]["rgb"].size() == 3) {
      rgb[0] = (unsigned char)proj_config_["patterns"][pattern_name]["rgb"][0].as<int>();
      rgb[1] = (unsigned char)proj_config_["patterns"][pattern_name]["rgb"][1].as<int>();
      rgb[2] = (unsigned char)proj_config_["patterns"][pattern_name]["rgb"][2].as<int>();
    } else {
      return false;
    }
  } else if (proj_config_["default_settings"]["default_rgb"] &&
             proj_config_["default_settings"]["default_rgb"].size() == 3) {
    rgb[0] = (unsigned char)proj_config_["default_settings"]["default_rgb"][0].as<int>();
    rgb[1] = (unsigned char)proj_config_["default_settings"]["default_rgb"][1].as<int>();
    rgb[2] = (unsigned char)proj_config_["default_settings"]["default_rgb"][2].as<int>();
  }

  int status = projector_.SetLedCurrents(rgb[0], rgb[1], rgb[2]);

  return (status < 0) ? false : true;
}

bool Lightcrafter4500::ProjectSinglePattern(const std::string& pattern_name, int pattern_indice) {
  if (!PatternExists(pattern_name)) {
    return false;
  }

  SetLed(pattern_name);

  std::vector<LightcrafterSinglePattern> pattern_vec = {};
  LightcrafterSinglePattern single_pattern = GetSinglePattern(pattern_name, pattern_indice);
  pattern_vec.push_back(single_pattern);

  unsigned int exposure_period, frame_period;
  exposure_period = frame_period =
      proj_config_["patterns"][pattern_name]["exposure_us"]
          ? proj_config_["patterns"][pattern_name]["exposure_us"].as<unsigned int>()
          : backup_exposure_period_us_;

  int status = projector_.PlayPatternSequence(pattern_vec, exposure_period, frame_period);

  return (status < 0) ? false : true;
};

bool Lightcrafter4500::ProjectFullPattern(const std::string& pattern_name) {
  if (!PatternExists(pattern_name)) {
    return false;
  }

  SetLed(pattern_name);

  std::vector<LightcrafterSinglePattern> pattern_vec = GetPatternSequence(pattern_name);

  unsigned int exposure_period, frame_period;
  exposure_period = frame_period =
      proj_config_["patterns"][pattern_name]["exposure_us"]
          ? proj_config_["patterns"][pattern_name]["exposure_us"].as<unsigned int>()
          : backup_exposure_period_us_;

  int status = projector_.PlayPatternSequence(pattern_vec, exposure_period, frame_period);

  return (status < 0) ? false : true;
}

int Lightcrafter4500::GetNumberProjections(const YAML::Node& config,
                                           const std::string& pattern_name) {
  return (PatternExists(config, pattern_name))
             ? static_cast<int>(config["patterns"][pattern_name]["channel_number"].size())
             : -1;
}

std::vector<LightcrafterSinglePattern> Lightcrafter4500::GetPatternSequence(
    const std::string& pattern_name) {
  std::vector<LightcrafterSinglePattern> pattern_vec = {};

  int prev_image_indice = -1;
  int number_projections = GetNumberProjections(proj_config_, pattern_name);

  for (int i = 0; i < number_projections; i++) {
    int image_indice = proj_config_["patterns"][pattern_name]["image_index"][i]
                           ? proj_config_["patterns"][pattern_name]["image_index"][i].as<int>()
                           : backup_single_pattern_.image_indice;

    int bit_depth = proj_config_["patterns"][pattern_name]["bit_depth"][i]
                        ? proj_config_["patterns"][pattern_name]["bit_depth"][i].as<int>()
                        : backup_single_pattern_.bit_depth;

    int pattern_number = proj_config_["patterns"][pattern_name]["channel_number"][i]
                             ? proj_config_["patterns"][pattern_name]["channel_number"][i].as<int>()
                             : backup_single_pattern_.pattern_number;

    int led_select = proj_config_["patterns"][pattern_name]["display_colour"]
                         ? proj_config_["patterns"][pattern_name]["display_colour"].as<int>()
                         : backup_single_pattern_.led_select;

    bool invert_pattern =
        (bool)proj_config_["patterns"][pattern_name]["invert_pattern"][i]
            ? proj_config_["patterns"][pattern_name]["invert_pattern"][i].as<int>()
            : backup_single_pattern_.invert_pattern;

    LightcrafterSinglePattern temp1;
    temp1.trigger_type =
        (i == 0) ? 1 : 3;  // First pattern is hardware triggered, the rest are internal timer based
    temp1.pattern_number = pattern_number;
    temp1.bit_depth = bit_depth;
    temp1.led_select = led_select;
    temp1.image_indice = image_indice;
    temp1.invert_pattern = invert_pattern;
    temp1.insert_black_frame = false;
    temp1.buffer_swap = (prev_image_indice != image_indice) ? true : false;
    temp1.trigger_out_prev = false;
    pattern_vec.push_back(temp1);

    LightcrafterSinglePattern temp2;
    temp2.trigger_type = 3;
    temp2.pattern_number = pattern_number;
    temp2.bit_depth = bit_depth;
    temp2.led_select = led_select;
    temp2.image_indice = image_indice;
    temp2.invert_pattern = invert_pattern;
    temp2.insert_black_frame = false;
    temp2.buffer_swap = false;
    temp2.trigger_out_prev = false;
    pattern_vec.push_back(temp2);

    prev_image_indice = image_indice;
  }

  return pattern_vec;
}

LightcrafterSinglePattern Lightcrafter4500::GetSinglePattern(const std::string& pattern_name,
                                                             int pattern_indice) {
  LightcrafterSinglePattern single_pattern;

  single_pattern.trigger_type = 0;
  single_pattern.pattern_number =
      proj_config_["patterns"][pattern_name]["channel_number"][pattern_indice]
          ? proj_config_["patterns"][pattern_name]["channel_number"][pattern_indice].as<int>()
          : backup_single_pattern_.pattern_number;

  single_pattern.bit_depth =
      proj_config_["patterns"][pattern_name]["bit_depth"][pattern_indice]
          ? proj_config_["patterns"][pattern_name]["bit_depth"][pattern_indice].as<int>()
          : backup_single_pattern_.bit_depth;

  single_pattern.led_select =
      proj_config_["patterns"][pattern_name]["display_colour"]
          ? proj_config_["patterns"][pattern_name]["display_colour"].as<int>()
          : backup_single_pattern_.led_select;

  single_pattern.image_indice =
      proj_config_["patterns"][pattern_name]["image_index"][pattern_indice]
          ? proj_config_["patterns"][pattern_name]["image_index"][pattern_indice].as<int>()
          : backup_single_pattern_.image_indice;

  single_pattern.invert_pattern =
      (bool)proj_config_["patterns"][pattern_name]["invert_pattern"][pattern_indice]
          ? proj_config_["patterns"][pattern_name]["invert_pattern"][pattern_indice].as<int>()
          : backup_single_pattern_.image_indice;

  single_pattern.insert_black_frame = backup_single_pattern_.insert_black_frame;
  single_pattern.buffer_swap = backup_single_pattern_.buffer_swap;
  single_pattern.trigger_out_prev = backup_single_pattern_.trigger_out_prev;

  // std::cout << single_pattern << std::endl;

  return single_pattern;
}

}  // namespace projector
}  // namespace sl_sensor