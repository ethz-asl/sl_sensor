// Code adapted from SLStudio https://github.com/jakobwilm/slstudio

#ifndef SL_SENSOR_CODEC_CODEC_COMMON_HPP_
#define SL_SENSOR_CODEC_CODEC_COMMON_HPP_

#include <yaml-cpp/yaml.h>

namespace sl_sensor {
namespace codec {
enum class CodecDirection {
  kNone = 0,
  kHorizontal = 1 << 0,
  kVertical = 1 << 1,
  kBoth = kHorizontal | kVertical
};

std::tuple<unsigned int, unsigned int, CodecDirection> GetBasicCodecInformationFromYAMLNode(
    const YAML::Node& node);

}  // namespace codec
}  // namespace sl_sensor

#endif  // SL_SENSOR_CODEC_CODEC_COMMON_HPP_