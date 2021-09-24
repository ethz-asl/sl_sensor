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

#include "sl_sensor_projector/lightcrafter_single_pattern.hpp"

namespace sl_sensor {
namespace projector {
std::ostream& operator<<(std::ostream& out, const LightcrafterSinglePattern& single_pattern) {
  out << "trigger_type: " << single_pattern.trigger_type << "\n"
      << "pattern_number: " << single_pattern.pattern_number << "\n"
      << "bit_depth: " << single_pattern.bit_depth << "\n"
      << "led_select: " << single_pattern.led_select << "\n"
      << "image_indice: " << single_pattern.image_indice << "\n"
      << "invert_pattern: " << single_pattern.invert_pattern << "\n"
      << "insert_black_frame: " << single_pattern.insert_black_frame << "\n"
      << "buffer_swap: " << single_pattern.buffer_swap << "\n"
      << "trigger_out_prev: " << single_pattern.trigger_out_prev << "\n";
  return out;
};

}  // namespace projector
}  // namespace sl_sensor