#ifndef _SENSOR_MODEL_MAGNETOMETER_MODEL_H_
#define _SENSOR_MODEL_MAGNETOMETER_MODEL_H_

#include "basic_type.h"
#include "slam_basic_math.h"

#include "magn_measurement.h"

namespace sensor_model {

/* Class Magnetometer Model Declaration. */
class Magnetometer {

public:
    /* Options of Magnetometer Model. */
    struct Options {
        float kNoiseSigma = 0.15f;
    };

public:
    Magnetometer() = default;
    virtual ~Magnetometer() = default;

    // Convert magnetometer measurement to yaw (heading) angle.
    float ConvertMagnToYaw(const MagnMeasurement &magn, const float roll_deg = 0.0, const float pitch_deg = 0.0);

    // Format angle to [-180, 180] degrees.
    float FormatDegree(const float abnormal_degree);

    // Reference for member variables.
    Options &options() { return options_; }
    // Const reference for member variables.
    const Options &options() const { return options_; }

private:
    Options options_;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_MAGNETOMETER_MODEL_H_
