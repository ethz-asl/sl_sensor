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

#ifndef SL_SENSOR_CALIBRATION_PROJECTOR_PARAMETERS_HPP_
#define SL_SENSOR_CALIBRATION_PROJECTOR_PARAMETERS_HPP_

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

#endif  // SL_SENSOR_CALIBRATION_PROJECTOR_PARAMETERS_HPP_