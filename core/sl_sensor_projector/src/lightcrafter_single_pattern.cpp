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