#ifndef _SENSOR_MODEL_IMU_MEASUREMENT_H_
#define _SENSOR_MODEL_IMU_MEASUREMENT_H_

#include "basic_type.h"
#include "slam_basic_math.h"

namespace sensor_model {

/* Measurement of imu. */
struct ImuMeasurement {
    Vec3 accel = Vec3::Zero();
    Vec3 gyro = Vec3::Zero();
    float time_stamp_s = -1.0f;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_IMU_MEASUREMENT_H_
