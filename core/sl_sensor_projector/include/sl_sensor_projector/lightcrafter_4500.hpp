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

#ifndef SL_SENSOR_PROJECTOR_LIGHTCRAFTER_4500_HPP_
#define SL_SENSOR_PROJECTOR_LIGHTCRAFTER_4500_HPP_

#include "sl_sensor_projector/lightcrafter_4500_api.hpp"
#include "sl_sensor_projector/lightcrafter_single_pattern.hpp"

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

namespace sl_sensor {
namespace projector {
/**
 * @brief Class object that manages the Lightcrafter 4500 used by the Lightcrafter 4500 ROS Node.
 * Note: For functions that return a boolean, true means success and false means failure
 *
 */
class Lightcrafter4500 {
 public:
  Lightcrafter4500();
  ~Lightcrafter4500();

  /**
   * @brief Load projector's YAML config file
   *
   * @param yaml_directory - YAML file directory
   */
  void LoadYaml(const std::string& yaml_directory);

  /**
   * @brief Init projector
   *
   * @return true
   * @return false
   */
  bool Init();

  /**
   * @brief Close projector
   *
   * @return true
   * @return false
   */
  bool Close();

  /**
   * @brief Display a white image
   *
   * @return true
   * @return false
   */
  bool DisplayWhite();

  /**
   * @brief Display a dark screen
   *
   * @return true
   * @return false
   */
  bool DisplayBlack();

  /**
   * @brief Check if a pattern exists the projector's YAML config file
   *
   * @param pattern_name - Name of pattern sequence
   * @return true
   * @return false
   */
  bool PatternExists(const std::string& pattern_name);

  /**
   * @brief Project a single pattern image
   *
   * @param pattern_name - Name of pattern sequence
   * @param pattern_indice - Indice of the pattern to be projection within the pattern sequence
   * @return true
   * @return false
   */
  bool ProjectSinglePattern(const std::string& pattern_name, int pattern_indice);

  /**
   * @brief Project enture pattern sequence
   *
   * @param pattern_name - Name of pattern sequence
   * @return true
   * @return false
   */
  bool ProjectFullPattern(const std::string& pattern_name);

  /**
   * @brief Generate a vector of LightcrafterSinglePattern based of the desired pattern
   *
   * @param - Name of pattern sequence
   * @return std::vector<LightcrafterSinglePattern>
   */
  std::vector<LightcrafterSinglePattern> GetPatternSequence(const std::string& pattern_name);

  /**
   * @brief Get LightcrafterSinglePattern based of a pattern in a se
   *
   * @param - Name of pattern sequence
   * @return std::vector<LightcrafterSinglePattern>
   */

  /**
   * @brief Get LightcrafterSinglePattern based of a pattern in a sequence
   *
   * @param pattern_name - Name of pattern sequence
   * @param pattern_indice - Indice of the pattern to be projection within the pattern sequence
   * @return LightcrafterSinglePattern
   */
  LightcrafterSinglePattern GetSinglePattern(const std::string& pattern_name, int pattern_indice);

  /**
   * @brief Get the Default Exposure Period from YAML file
   *
   * @return unsigned int - Exposure period in micro seconds
   */
  unsigned int GetDefaultExposurePeriod();

  /**
   * @brief Set the Led brightness for a particular pattern
   *
   * @param pattern_name
   * @return true
   * @return false
   */
  bool SetLed(const std::string& pattern_name = "");

  /**
   * @brief Check if a pattern exists in the YAML config file
   *
   * @param config - YAML node
   * @param pattern_name - Pattern name to be checked
   * @return true
   * @return false
   */
  static bool PatternExists(const YAML::Node& config, const std::string& pattern_name);

  /**
   * @brief Get the Number Projections there are in a pattern sequence
   *
   * @param config - YAML node
   * @param pattern_name - Pattern name to be checked
   * @return int - Number of projections
   */
  static int GetNumberProjections(const YAML::Node& config, const std::string& pattern_name);

 private:
  Lightcrafter4500Api projector_;
  YAML::Node proj_config_;
  unsigned int backup_exposure_period_us_ = 1000000;
  LightcrafterSinglePattern backup_single_pattern_ =
      LightcrafterSinglePattern(0, 0, 8, 7, 0, false, false, true, false);
  unsigned char backup_rgb_[3] = {
      104, 135, 130};  // This is the default LED settings from the official TI Lightcrafter GUI
};

}  // namespace projector
}  // namespace sl_sensor

#endif  // SL_SENSOR_PROJECTOR_LIGHTCRAFTER_4500_HPP_