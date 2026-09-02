#ifndef _SENSOR_MODEL_BAROMETER_MODEL_H_
#define _SENSOR_MODEL_BAROMETER_MODEL_H_

#include "baro_measurement.h"

#include "basic_type.h"
#include "slam_basic_math.h"

namespace sensor_model {

/* Class Barometer Model Declaration. */
class Barometer {

public:
    /* Options of Barometer Model. */
    struct Options {
        float kNoiseSigma = 0.25f;
    };

public:
    Barometer() = default;
    virtual ~Barometer() = default;

    // Convert measure of baro to altitude.
    float ConvertBaroToAltitude(const BaroMeasurement &measure);
    // Convert measure of baro to altitude with temperature compensation.
    float ConvertBaroWithTemperatureToAltitude(const BaroMeasurement &measure);

    // Reference for member variables.
    Options &options() { return options_; }
    // Const reference for member variables.
    const Options &options() const { return options_; }

private:
    Options options_;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_BAROMETER_MODEL_H_
