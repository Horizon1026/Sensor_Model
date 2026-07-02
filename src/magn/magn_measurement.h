#ifndef _SENSOR_MODEL_MAGN_MEASUREMENT_H_
#define _SENSOR_MODEL_MAGN_MEASUREMENT_H_

#include "basic_type.h"

namespace sensor_model {

/* Measurement of magnetometer. */
struct MagnMeasurement {
    double time_stamp_s = 0.0;
    Vec3 magn_mG = Vec3::Zero();
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_MAGN_MEASUREMENT_H_
