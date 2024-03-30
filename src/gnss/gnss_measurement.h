#ifndef _SENSOR_MODEL_GNSS_MEASUREMENT_H_
#define _SENSOR_MODEL_GNSS_MEASUREMENT_H_

#include "datatype_basic.h"

namespace SENSOR_MODEL {

/* Measurement of GNSS. */
struct GnssMeasurement {
    float time_stamp_s = 0.0f;

    // LLA observations.
    bool is_lla_valid = false;
    double longitude_deg = 0.0; // 经度
    double latitude_deg = 0.0;  // 纬度
    double altitude_m = 0.0;    // 椭球高度

    // Yaw defined in ENU frame.
    bool is_yaw_valid = false;
    double yaw_ned_deg = 0.0;

    // Velocity defined in ENU frame with unit : m/s.
    bool is_vel_valid = false;
    float vel_east_mps = 0.0f;
    float vel_north_mps = 0.0f;
    float vel_up_mps = 0.0f;
};

}

#endif // end of _SENSOR_MODEL_GNSS_MEASUREMENT_H_
