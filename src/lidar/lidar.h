#ifndef _SENSOR_MODEL_LIDAR_MODEL_H_
#define _SENSOR_MODEL_LIDAR_MODEL_H_

#include "basic_type.h"
#include "lidar_measurement.h"
#include "slam_basic_math.h"

namespace SENSOR_MODEL {

/* Class Lidar Model Declaration. */
class Lidar {

public:
    /* Options of Lidar Model. */
    struct Options {
        float kPositionNoiseSigma = 1.0f;
    };

public:
    Lidar() = default;
    virtual ~Lidar() = default;

    // Reference for member variables.
    Options &options() { return options_; }

    // Const reference for member variables.
    const Options &options() const { return options_; }

private:
    Options options_;
};

}  // namespace SENSOR_MODEL

#endif  // end of _SENSOR_MODEL_LIDAR_MODEL_H_
