#ifndef _SENSOR_MODEL_GNSS_MEASUREMENT_H_
#define _SENSOR_MODEL_GNSS_MEASUREMENT_H_

#include "datatype_basic.h"

namespace SENSOR_MODEL {

/* Measurement of GNSS. */
struct GnssMeasurement {
    float time_stamp_s = 0.0f;

    double longitude_deg = 0.0;
    double latitude_deg = 0.0;
    double altitude_m = 0.0;
    double yaw_rad = 0.0;
};

}

#endif // end of _SENSOR_MODEL_GNSS_MEASUREMENT_H_
