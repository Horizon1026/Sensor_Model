#ifndef _SENSOR_MODEL_BARO_MEASUREMENT_H_
#define _SENSOR_MODEL_BARO_MEASUREMENT_H_

#include "basic_type.h"

namespace sensor_model {

/* Measurement of barometer. */
struct BaroMeasurement {
    double time_stamp_s = 0.0;
    float temperature_C = 0.0f;
    float pressure_Kpa = 0.0f;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_BARO_MEASUREMENT_H_
