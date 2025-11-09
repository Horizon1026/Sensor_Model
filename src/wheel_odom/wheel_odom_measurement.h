#ifndef _SENSOR_MODEL_WHEEL_ODOM_MEASUREMENT_H_
#define _SENSOR_MODEL_WHEEL_ODOM_MEASUREMENT_H_

#include "basic_type.h"

namespace sensor_model {

/* Measurement of Wheel Odom. */
struct WheelOdomMeasurement {
    float time_stamp_s = 0.0f;

    // Velocity of wheel encoder. (Unit : count / s)
    bool is_wheel_velocity_valid = false;
    float encoder_left_cnt = 0;
    float encoder_right_cnt = 0;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_WHEEL_ODOM_MEASUREMENT_H_
