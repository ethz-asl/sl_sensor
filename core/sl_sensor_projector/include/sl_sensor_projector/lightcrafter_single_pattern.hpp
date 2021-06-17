#pragma once
#include <iostream>

namespace sl_sensor
{
namespace projector
{
/**
 * @brief Struct that contains the relevant information to project a pattern on the projector
 *
 */
struct LightcrafterSinglePattern
{
  int trigger_type = 0;
  int pattern_number = 0;
  int bit_depth = 0;
  int led_select = 0;
  int image_indice = 0;
  bool invert_pattern = false;
  bool insert_black_frame = false;
  bool buffer_swap = false;
  bool trigger_out_prev = false;

  LightcrafterSinglePattern(int trigger_type_ = 0, int pattern_number_ = 0, int bit_depth_ = 0, int led_select_ = 0,
                            int image_indice_ = 0, bool invert_pattern_ = false, bool insert_black_frame_ = false,
                            bool buffer_swap_ = false, bool trigger_out_prev_ = false)
    : trigger_type(trigger_type_)
    , pattern_number(pattern_number_)
    , bit_depth(bit_depth_)
    , led_select(led_select_)
    , image_indice(image_indice_)
    , invert_pattern(invert_pattern_)
    , insert_black_frame(insert_black_frame_)
    , buffer_swap(buffer_swap_)
    , trigger_out_prev(trigger_out_prev_){};
};

std::ostream& operator<<(std::ostream& out, const LightcrafterSinglePattern& single_pattern);

}  // namespace projector

}  // namespace sl_sensor