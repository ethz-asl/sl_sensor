#pragma once

#include "sl_sensor_projector/lightcrafter_single_pattern.hpp"

#include <memory>
#include <string>
#include <vector>

namespace sl_sensor {
namespace projector {
/**
 * @brief Interface to control Lightcrafter 4500. Note: Most functions return an int. int < 0 means
 * that the action has failed
 *
 */
class Lightcrafter4500Api {
 public:
  Lightcrafter4500Api();
  ~Lightcrafter4500Api();

  /**
   * @brief Initialise connection with Lightcrafter. Call at the start of operation. Do not call
   * more than once
   *
   * @return int
   */
  int Init();

  /**
   * @brief Disconnect Lightcrafter, automatically called upon object destruction
   *
   * @return int
   */
  int Close();

  /**
   * @brief Add a pattern to the pattern sequence to be projected
   *
   * @param pattern
   * @return int
   */
  int AppendPatternSequence(const LightcrafterSinglePattern& pattern);

  /**
   * @brief Set the Pattern Sequence to be projected
   *
   * @param pattern_vec
   * @return int
   */
  int SetPatternSequence(const std::vector<LightcrafterSinglePattern>& pattern_vec);

  /**
   * @brief Display pattern sequence
   *
   * @param pattern_vec - Pattern to be displayed
   * @param exposure_period_us - Exposure time for each pattern in microseconds
   * @param frame_period_us - Frame period in microseconds (Exposure period + time to load image)
   * @note Usually we just set exposure_period_us = frame_period_us
   * @return int
   */
  int PlayPatternSequence(const std::vector<LightcrafterSinglePattern>& pattern_vec,
                          unsigned int exposure_period_us, unsigned int frame_period_us);

  /**
   * @brief Clear class object's internally stored pattern sequence
   *
   * @return int
   */
  int ClearPatternSequence();

  /**
   * @brief Send class object's internally stored pattern sequence to the projector
   *
   * @param exposure_period_us - Exposure time for each pattern in microseconds
   * @param frame_period_us - Frame period in microseconds (Exposure period + time to load image)
   * @note Usually we just set exposure_period_us = frame_period_us
   * @return int
   */
  int SendPatternSequence(unsigned int exposure_period_us, unsigned int frame_period_us);

  /**
   * @brief Set the Led Currents object
   *
   * @param r - 0-255 value for red LED, with 255 being the brightest value
   * @param g - 0-255 value for green LED, with 255 being the brightest value
   * @param b - 0-255 value for blue LED, with 255 being the brightest value
   * @return int
   */
  int SetLedCurrents(unsigned char r, unsigned char g, unsigned char b);

  /**
   * @brief Get the Led Currents of the projector
   *
   * @param r - 0-255 value for red LED, with 255 being the brightest value
   * @param g - 0-255 value for green LED, with 255 being the brightest value
   * @param b - 0-255 value for blue LED, with 255 being the brightest value
   * @return int
   */
  int GetLedCurrents(unsigned char& r, unsigned char& g, unsigned char& b);

  /**
   * @brief Prints Projector information on console
   *
   */
  void PrintProjectorInfo();

  /**
   * @brief Set projector to Pattern Mode
   *
   * @return int
   */
  int SetPatternMode();

  /**
   * @brief Set projector to Video Mode (input from HDMI port)
   *
   * @return int
   */
  int SetVideoMode();

  /**
   * @brief Start displaying the pattern sequence
   *
   * @return int
   */
  int SetPatternSequenceStart();

  /**
   * @brief Pause the displaying of the pattern sequence
   *
   * @return int
   */
  int SetPatternSequencePause();

  /**
   * @brief Stop
   *
   * @return int
   */
  int SetPatternSequenceStop();

 private:
  const int max_entries_ = 10;
  std::vector<LightcrafterSinglePattern> pattern_store_ = {};

  /**
   * @brief Display error
   *
   * @param err
   */
  void ShowError(const std::string& err);

  /**
   * @brief Set the Pattern Sequence Mode object
   *
   * @param desired_mode
   * @return int
   */
  int SetPatternSequenceMode(unsigned int desired_mode);

  /**
   * @brief Fix any issues with buffer swaps with the internally stored pattern sequence
   *
   */
  void CheckAndFixBufferSwaps();

  /**
   * @brief Validate that internally stored pattern sequence can be displayed by the projector
   *
   * @return int
   */
  int ValidatePattern();

  /**
   * @brief Convenience function to sleep for a small amount of time
   *
   * @param ms
   */
  void SleepMs(int ms);
};

}  // namespace projector
}  // namespace sl_sensor
