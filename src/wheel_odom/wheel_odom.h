#ifndef _SENSOR_MODEL_WHEEL_ODOM_BASIC_H_
#define _SENSOR_MODEL_WHEEL_ODOM_BASIC_H_

#include "basic_type.h"
#include "slam_basic_math.h"

#include "wheel_odom_measurement.h"

namespace SENSOR_MODEL {

/* Class Wheel Odom Model Declaration. */
class WheelOdom {

public:
    /* Options of Wheel Odom Model. */
    struct Options {
        float kWheelRadiusInMeter = 0.0f;
        float kEncoderCountInOneCircle = 0.0f;
        float kEncoderSamplePeriodInSecond = 0.0f;
        float kVelocityNoiseSigma = 0.0f;
    };

public:
    WheelOdom() = default;
    virtual ~WheelOdom() = default;

    // Process measurements to be state observations.
    Vec3 ConvertEncoderCountToVelocity(const WheelOdomMeasurement &measure);
    Vec3 ConvertLeftEncoderCountToVelocity(const WheelOdomMeasurement &measure);
    Vec3 ConvertRightEncoderCountToVelocity(const WheelOdomMeasurement &measure);

    // Reference for member variables.
    Options &options() { return options_; }

    // Const reference for member variables.
    const Options &options() const { return options_; }

private:
    Options options_;
};

}  // namespace SENSOR_MODEL

#endif  // end of _SENSOR_MODEL_WHEEL_ODOM_BASIC_H_