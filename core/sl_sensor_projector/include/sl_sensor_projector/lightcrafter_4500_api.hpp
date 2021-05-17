#pragma once

#include "sl_sensor_projector/lightcrafter_single_pattern.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sl_sensor
{
namespace projector
{
class Lightcrafter4500Api
{
public:
  Lightcrafter4500Api();
  ~Lightcrafter4500Api();

  int Init();
  int Close();
  int AppendPatternSequence(const LightcrafterSinglePattern& pattern);
  int SetPatternSequence(const std::vector<LightcrafterSinglePattern>& pattern_vec);
  int PlayPatternSequence(const std::vector<LightcrafterSinglePattern>& pattern_vec, unsigned int exposure_period_us,
                          unsigned int frame_period_us);
  int ClearPatternSequence();
  int SendPatternSequence(unsigned int exposure_period_us, unsigned int frame_period_us);
  int SetLedCurrents(unsigned char r, unsigned char g, unsigned char b);
  int GetLedCurrents(unsigned char& r, unsigned char& g, unsigned char& b);
  void PrintProjectorInfo();
  int SetPatternMode();
  int SetVideoMode();
  int SetPatternSequenceStart();
  int SetPatternSequencePause();
  int SetPatternSequenceStop();
  void SleepMs(int ms);

private:
  void ShowError(const std::string& err);
  const int max_entries_ = 10;
  int set_pat_seq_mode(unsigned int desired_mode);
  std::vector<LightcrafterSinglePattern> m_pattern_store = {};
  void check_and_fix_buffer_swaps();
  int validate_pattern();
};

}  // namespace projector

}  // namespace sl_sensor
