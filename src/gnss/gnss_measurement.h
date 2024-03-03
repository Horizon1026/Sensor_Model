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

    double yaw_north_rad = 0.0;

    // Velocity with unit : m/s.
    float vel_east_mps = 0.0f;
    float vel_north_mps = 0.0f;
    float vel_up_mps = 0.0f;

    // Quality of gnss.
    bool is_fixed = false;
    float hacc = 0.0f;
    float hdop = 0.0f;
    float pdop = 0.0f;
};

}

#endif // end of _SENSOR_MODEL_GNSS_MEASUREMENT_H_
