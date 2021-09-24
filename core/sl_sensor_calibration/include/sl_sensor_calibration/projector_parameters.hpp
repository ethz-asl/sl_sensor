#ifndef PROJECTOR_PARAMETERS_HPP_
#define PROJECTOR_PARAMETERS_HPP_

#include "sl_sensor_calibration/intrinsic_parameters.hpp"

namespace sl_sensor {
namespace calibration {

/**
 * @brief Projector parameters only contain camera intrinsics for the projector at the moment, hence
 * this is just a typedef for now
 *
 */
typedef IntrinsicParameters ProjectorParameters;
}  // namespace calibration
}  // namespace sl_sensor

#endif  // PROJECTOR_PARAMETERS_HPP_