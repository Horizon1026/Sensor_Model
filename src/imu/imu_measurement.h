#ifndef _SENSOR_MODEL_IMU_MEASUREMENT_H_
#define _SENSOR_MODEL_IMU_MEASUREMENT_H_

#include "datatype_basic.h"
#include "math_kinematics.h"

namespace SENSOR_MODEL {

/* Measurement of imu. */
struct ImuMeasurement {
    Vec3 accel = Vec3::Zero();
    Vec3 gyro = Vec3::Zero();
    float time_stamp_s = 0.0f;
};

}

#endif // end of _SENSOR_MODEL_IMU_MEASUREMENT_H_
