#include <iostream>
#include "sl_sensor_projector/lightcrafter_4500_api.hpp"

#include <dlpc350_api.h>
#include <dlpc350_firmware.h>
#include <dlpc350_usb.h>
#include <dlpc350_version.h>
#include <usb.h>

#include <chrono>
#include <thread>

namespace sl_sensor
{
namespace projector
{
Lightcrafter4500Api::Lightcrafter4500Api()
{
}

Lightcrafter4500Api::~Lightcrafter4500Api()
{
  this->Close();
}

void Lightcrafter4500Api::ShowError(const std::string& err)
{
  std::cerr << "Lightcrafter Error: " << err << std::endl;
}

int Lightcrafter4500Api::Init()
{
  // Initialize usb connection
  if (DLPC350_USB_Init() < 0)
  {
    ShowError("Could not Init USB!");
    return -1;
  }

  if (DLPC350_USB_Open() < 0)
  {
    ShowError("Could not connect!");
    return -1;
  }

  // Make sure LC is not in standby
  bool is_standby;
  DLPC350_GetPowerMode(&is_standby);
  if (is_standby)
  {
    DLPC350_SetPowerMode(0);
    SleepMs(5000);
    std::cout << "Lightcrafter is in Standby Mode, will take ~5 second to power up" << std::endl;
  }

  for (int i = 0; i < max_entries_; i++)
  {
    DLPC350_GetPowerMode(&is_standby);

    if (is_standby)
    {
      std::cout << "Lightcrafter is still in Standby Mode, will check status "
                   "in another 3 seconds"
                << std::endl;
      SleepMs(3000);
    }
    else
    {
      return 0;
    }
  }

  std::cout << "Initialised and opened USB connection to Lightcrafter!" << std::endl;

  return -1;
}

int Lightcrafter4500Api::Close()
{
  if (DLPC350_USB_IsConnected())
  {
    // Check if it is in Pattern Mode
    // If it is, we stop pattern playing
    bool is_in_pattern_mode;
    DLPC350_GetMode(&is_in_pattern_mode);
    if (is_in_pattern_mode)
    {
      SetPatternSequenceStop();
    }

    if (DLPC350_USB_Close() < 0)
    {
      ShowError("Could not Close!");
      return -1;
    }

    if (DLPC350_USB_Exit() < 0)
    {
      ShowError("Could not exit!");
      return -1;
    }
  }

  return 0;
}

int Lightcrafter4500Api::SendPatternSequence(unsigned int exposure_period_us, unsigned int frame_period_us)
{
  int i, num_lut_entries = 0;
  // unsigned int status;
  char error_str[256];
  unsigned char splash_lut[64];
  int num_splash_lut_entries = 0;
  int num_pats_in_exposure = 1;
  unsigned int min_pat_exposure[8] = { 235, 700, 1570, 1700, 2000, 2500, 4500, 8333 };
  unsigned int worst_case_bit_depth = 0;
  // unsigned int num_patterns;
  int trig_mode = 0;

  // Pattern store cannot be empty
  if (m_pattern_store.size() == 0)
  {
    ShowError("Pattern sequence is empty, cannnot send to Lightcrafter");
    return -1;
  }

  // Fix Buffer Swaps
  check_and_fix_buffer_swaps();

  // Make sure the Pattern Exposure and Pattern Period timings are within the
  // spec

  // Pattern Exposure > Pattern Period not a valid settings
  if (exposure_period_us > frame_period_us)
  {
    ShowError("Pattern exposure setting voilation, it should be, Pattern Exposure = "
              "Pattern Period or (Pattern Period - Pattern Exposure) > 230us");
    return -1;
  }

  // If Pattern Exposure != Pattern Period then (Pattern Period - Pattern
  // Exposure) > 230us
  if ((exposure_period_us != frame_period_us) && ((frame_period_us - exposure_period_us) <= 230))
  {
    ShowError("Pattern exposure setting voilation, it should be, Pattern Exposure = "
              "Pattern Period or (Pattern Period - Pattern Exposure) > 230us");
    return -1;
  }

  // Clear existing pattern in LC
  DLPC350_ClearPatLut();

  // Go through all patterns, check exposure timings for patterns that share the
  // same exposure period
  for (i = 0; i < (int)m_pattern_store.size(); i++)
  {
    auto curr_pattern = m_pattern_store[i];

    // If first pattern, it is an invalid pattern sequence if it has no trigger
    if (i == 0 && curr_pattern.trigger_type == 3)
    {
      ShowError("First Item must be triggered. Please select a Trigger_In_Type "
                "other than No Trigger");
      return -1;
    }

    // If exposure time for this pattern is not shared with previous patterns
    if (curr_pattern.trigger_out_prev == false)
    {
      // If the previous patterns before this share their exposure times
      if (num_pats_in_exposure != 1)
      {
        // Check if exposure time is above the minimum requirement
        if (exposure_period_us / num_pats_in_exposure < min_pat_exposure[worst_case_bit_depth])
        {
          sprintf(error_str, "Exposure time %d < Minimum Exposure time %d for bit depth %d",
                  exposure_period_us / num_pats_in_exposure, min_pat_exposure[worst_case_bit_depth],
                  worst_case_bit_depth + 1);
          ShowError(error_str);
          return -1;
        }
      }

      // Reset value of these temp vars
      num_pats_in_exposure = 1;
      worst_case_bit_depth = 0;
    }
    else
    {
      // Else if the current pattern shares exposure time with the previous
      num_pats_in_exposure++;  // increment count

      // Update worst case bit depth (largest bit depth used in the
      // sequence)
      if ((int)curr_pattern.bit_depth - 1 > (int)worst_case_bit_depth)
      {
        worst_case_bit_depth = curr_pattern.bit_depth - 1;
      }
    }

    /**
    std::cout << "Pattern "
              << "[" << i << "] "
              << "TrigType = " << curr_pattern.trigger_type << ","
              << "PatNum = " << curr_pattern.pattern_number << ","
              << "BitDepth = " << curr_pattern.bit_depth << ","
              << "LEDSelect = " << curr_pattern.led_select << ","
              << "InvertPattern = " << curr_pattern.invert_pattern << ","
              << "Insert Black = " << curr_pattern.insert_black_frame << ","
              << "BufSwap = " << curr_pattern.buffer_swap << ","
              << "TrigOutPrev = " << curr_pattern.trigger_out_prev << std::endl;
    **/

    // If everything checks out, add pattern
    if (DLPC350_AddToPatLut(curr_pattern.trigger_type, curr_pattern.pattern_number, curr_pattern.bit_depth,
                            curr_pattern.led_select, curr_pattern.invert_pattern, curr_pattern.insert_black_frame,
                            curr_pattern.buffer_swap, curr_pattern.trigger_out_prev) < 0)
    {
      ShowError("Error Updating LUT");
      return -1;
    }

    // If there is a buffer swap or if this is the first pattern (image needs
    // to be changed for this pattern)
    if (curr_pattern.buffer_swap || num_splash_lut_entries == 0)
    {
      if (num_splash_lut_entries >= 64)
      {
        ShowError("Image LUT entries(64) reached maximum. Will not add anymore "
                  "entries\n");
      }
      else
        // We add the image indice to splash_lut array
        splash_lut[num_splash_lut_entries++] = (unsigned char)curr_pattern.image_indice;
    }

    // Increment number of patterns inserted
    num_lut_entries++;
  }

  if (num_pats_in_exposure != 1)
  {
    // Check if expsoure time is above the minimum requirement
    if (exposure_period_us / num_pats_in_exposure < min_pat_exposure[worst_case_bit_depth])
    {
      sprintf(error_str, "Exposure time %d < Minimum Exposure time %d for bit depth %d",
              exposure_period_us / num_pats_in_exposure, min_pat_exposure[worst_case_bit_depth],
              worst_case_bit_depth + 1);
      ShowError(error_str);
      return -1;
    }
  }

  // Start pattern mode
  this->SetPatternMode();

  // Config pattern mode
  unsigned int num_patterns_trigout2 = 1;  // Does not matter a lot since we are not using trigout
  bool repeat_mode = true;
  if (DLPC350_SetPatternConfig(num_lut_entries, repeat_mode, num_patterns_trigout2, num_splash_lut_entries) < 0)
  {
    ShowError("Error Sending Pattern Config");
    return -1;
  }

  // Set exposure and fram period
  if (DLPC350_SetExposure_FramePeriod(exposure_period_us, frame_period_us) < 0)
  {
    ShowError("Error Sending Exposure period");
    return -1;
  }

  trig_mode = 1;  // Internal trigger
  // Configure Trigger Mode - 0 or 1
  if (DLPC350_SetPatternTriggerMode(trig_mode) < 0)
  {
    ShowError("Error Sending trigger Mode");
    return -1;
  }

  // Send Pattern LUT
  if (DLPC350_SendPatLut() < 0)
  {
    ShowError("Error Sending Pattern LUT");
    return -1;
  }

  /**
  std::cout << "num_lut_entries: " << num_lut_entries << std::endl;
  std::cout << "num_patterns_trigout2: " << num_patterns_trigout2 << std::endl;
  std::cout << "num_splash_lut_entries: " << num_splash_lut_entries
            << std::endl;
  std::cout << "splash_lut[]: ";
  for (const unsigned char splash : splash_lut) {
    std::cout << (unsigned int)splash << " ";
  }
  std::cout << std::endl;
  **/

  // Send Image LUT
  if (DLPC350_SendImageLut(&splash_lut[0], num_splash_lut_entries) < 0)
  {
    ShowError("Error Sending Image LUT");
    return -1;
  }

  return validate_pattern();
}

void Lightcrafter4500Api::PrintProjectorInfo()
{
  char version_str[255];
  unsigned int API_ver, App_ver, SWConfig_ver, SeqConfig_ver;
  unsigned int num_img_in_flash = 0;

  if (DLPC350_USB_IsConnected())
  {
    std::cout << "<<< Projector information >>>" << std::endl;

    // Version Information
    if (DLPC350_GetVersion(&App_ver, &API_ver, &SWConfig_ver, &SeqConfig_ver) == 0)
    {
      sprintf(version_str, "%d.%d.%d", (App_ver >> 24), ((App_ver << 8) >> 24), ((App_ver << 16) >> 16));
      std::cout << "Version: " << version_str << std::endl;
    }

    // Read firmware tag information
    unsigned char firmwareTag[33];
    if (DLPC350_GetFirmwareTagInfo(&firmwareTag[0]) == 0)
    {
      std::cout << "Firmware Tag: " << firmwareTag << std::endl;
    }

    // Retrieve the total number of Images in the firmware info
    if (DLPC350_GetNumImagesInFlash(&num_img_in_flash) == 0)
    {
      std::cout << "Number of images in flash: " << num_img_in_flash << std::endl;
    }
  }
}

int Lightcrafter4500Api::SetPatternMode()
{
  int result = -1;

  // Check if it is in Pattern Mode
  bool is_in_pattern_mode;
  DLPC350_GetMode(&is_in_pattern_mode);

  if (is_in_pattern_mode)
  {
    // Stop pattern sequence
    unsigned int pattern_mode;
    DLPC350_GetPatternDisplay(&pattern_mode);
    if (pattern_mode != 0)
    {
      result = SetPatternSequenceStop();
    }
  }
  else
  {
    // Switch to Pattern Mode
    DLPC350_SetMode(true);
    SleepMs(100);

    for (int i = 0; i < max_entries_; i++)
    {
      DLPC350_GetMode(&is_in_pattern_mode);
      if (is_in_pattern_mode)
      {
        result = 0;
        break;
      }
      else
      {
        DLPC350_SetMode(true);
        SleepMs(100);
      }
    }
  }

  return result;
}

int Lightcrafter4500Api::SetVideoMode()
{
  int result = -1;

  // Check if it is in Pattern Mode
  bool is_in_pattern_mode;
  DLPC350_GetMode(&is_in_pattern_mode);

  if (is_in_pattern_mode)
  {
    // First stop pattern sequence
    unsigned int pattern_mode;
    DLPC350_GetPatternDisplay(&pattern_mode);
    if (pattern_mode != 0)
    {
      SetPatternSequenceStop();
      DLPC350_GetPatternDisplay(&pattern_mode);
    }

    // If we stopped pattern sequence successfully, switch to video mode
    if (pattern_mode == 0)
    {
      DLPC350_SetMode(false);
      SleepMs(100);

      for (int i = 0; i < max_entries_; i++)
      {
        DLPC350_GetMode(&is_in_pattern_mode);
        if (!is_in_pattern_mode)
        {
          result = 0;
          break;
        }
        else
        {
          DLPC350_SetMode(false);
          SleepMs(100);
        }
      }
    }
  }
  else
  {
    result = 0;
  }

  return result;
}

int Lightcrafter4500Api::SetPatternSequenceStart()
{
  return set_pat_seq_mode(2);
}

int Lightcrafter4500Api::SetPatternSequencePause()
{
  return set_pat_seq_mode(1);
}

int Lightcrafter4500Api::SetPatternSequenceStop()
{
  return set_pat_seq_mode(0);
}

void Lightcrafter4500Api::SleepMs(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int Lightcrafter4500Api::set_pat_seq_mode(unsigned int desired_mode)
{
  int result = -1;

  // Check if it is in Pattern Mode
  bool is_in_pattern_mode;
  DLPC350_GetMode(&is_in_pattern_mode);

  if (is_in_pattern_mode)
  {
    unsigned int current_pat_mode;
    DLPC350_PatternDisplay(desired_mode);
    SleepMs(100);

    for (int i = 0; i < max_entries_; i++)
    {
      DLPC350_GetPatternDisplay(&current_pat_mode);

      if (current_pat_mode == desired_mode)
      {
        // Mode change successful
        result = 0;
        break;
      }
      else
      {
        // Else try change mode again
        DLPC350_PatternDisplay(desired_mode);
        SleepMs(100);
      }
    }
  }
  else
  {
    // If not in pattern mode, we display error message
    ShowError("Warning: Attempted to set pattern sequence (Start/Stop/Pause) mode "
              "when projector is not in Pattern mode. Ignoring command.");
  }

  return result;
}

int Lightcrafter4500Api::AppendPatternSequence(const LightcrafterSinglePattern& pattern)
{
  m_pattern_store.push_back(pattern);
  return 0;
}

int Lightcrafter4500Api::ClearPatternSequence()
{
  m_pattern_store.clear();
  return 0;
}

void Lightcrafter4500Api::check_and_fix_buffer_swaps()
{
  int prev_img_indice = -1;  // Negative to ensure that first image indice is always differnt

  for (int i = 0; i < (int)m_pattern_store.size(); i++)
  {
    int curr_img_indice = m_pattern_store[i].image_indice;
    // If image indice has changed, we make sure that buffer swap is enabled
    if (prev_img_indice != curr_img_indice && !m_pattern_store[i].buffer_swap)
    {
      m_pattern_store[i].buffer_swap = true;
      std::cout << "Pattern with indice " << i << ": Change Buffer Swap to true since image indice " << curr_img_indice
                << " is different from the previous indice " << prev_img_indice << std::endl;
    }
    // Update prev image indice
    prev_img_indice = curr_img_indice;
  }
}

int Lightcrafter4500Api::validate_pattern()
{
  int i = 0;
  unsigned int status;
  bool ready;

  bool is_pattern_mode = false;
  DLPC350_GetMode(&is_pattern_mode);

  if (!is_pattern_mode)
  {
    ShowError("Please change operating mode to Pattern Sequence before validating "
              "sequence");
    return -1;
  }

  // if pattern sequence is already running it must be stopped first
  SetPatternSequenceStop();

  if (DLPC350_StartPatLutValidate())
  {
    ShowError("Error validating LUT data");
    return -1;
  }

  do
  {
    if (DLPC350_CheckPatLutValidate(&ready, &status) < 0)
    {
      ShowError("Error validating LUT data");
      return -1;
    }

    if (ready)
    {
      break;
    }
    else
    {
      SleepMs(200);
    }

    if (i++ > max_entries_)
      break;
  } while (1);

  // Print out any possible warnings and errors
  if (status & BIT0)
  {
    ShowError("Selected exposure or frame period settings are invalid");
  }

  if (status & BIT1)
  {
    ShowError("Selected pattern numbers in LUT are invalid");
  }

  if (status & BIT2)
  {
    ShowError("Warning, continuous Trigger Out1 request or overlapping black "
              "sectors (Not critical)");
  }

  if (status & BIT3)
  {
    ShowError("Warning, post vector was not inserted prior to external triggered "
              "vector (Not critical)");
  }

  if (status & BIT4)
  {
    ShowError("Warning, frame period or exposure difference is less than 230usec "
              "(Not critical)");
  }

  // If bit 0 or bit 1 are value 1, validation is unsuccessful
  if ((status & BIT0) || (status & BIT1))
  {
    ShowError("Pattern validation unsuccessful!");

    return -1;
  }

  return 0;
}

int Lightcrafter4500Api::SetLedCurrents(unsigned char r, unsigned char g, unsigned char b)
{
  // r,g and b values are the ones displayed on the GUI
  return DLPC350_SetLedCurrents(255 - r, 255 - g, 255 - b);
}

int Lightcrafter4500Api::GetLedCurrents(unsigned char& r, unsigned char& g, unsigned char& b)
{
  unsigned char red_current, green_current, blue_current;

  int status = DLPC350_GetLedCurrents(&red_current, &green_current, &blue_current);

  if (status == 0)
  {
    // Convert rgb values as displayed on the GUI
    r = 255 - red_current;
    g = 255 - green_current;
    b = 255 - blue_current;
  }

  return status;
}

int Lightcrafter4500Api::SetPatternSequence(const std::vector<LightcrafterSinglePattern>& pattern_vec)
{
  m_pattern_store.clear();
  m_pattern_store = pattern_vec;
  return 0;
}

int Lightcrafter4500Api::PlayPatternSequence(const std::vector<LightcrafterSinglePattern>& pattern_vec,
                                             unsigned int exposure_period_us, unsigned int frame_period_us)
{
  // Set pattern
  SetPatternSequence(pattern_vec);

  // Send patterns (This will automatically stop previous pattern)
  // std::cout << "Sending patterns" << std::endl;
  if (SendPatternSequence(exposure_period_us, frame_period_us) < 0)
  {
    std::cout << "Failed to send pattern" << std::endl;
    return -1;
  }

  // Start playing pattern
  // std::cout << "Start playing pattern" << std::endl;
  if (SetPatternSequenceStart() < 0)
  {
    std::cout << "Failed to start playing pattern" << std::endl;
    return -1;
  }

  return 0;
}

}  // namespace projector

}  // namespace sl_sensor
