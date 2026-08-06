#include "barometer.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"

namespace sensor_model {

namespace {
    constexpr float kPressureOfSeaLevel = 101.325f;
}

float Barometer::ConvertBaroToAltitude(const BaroMeasurement &measure) {
    const float ratio = measure.pressure_Kpa / kPressureOfSeaLevel;
    if (ratio <= 0) {
        return 0.0f;
    }
    return static_cast<float>(44330.0 * (1.0 - std::pow(static_cast<double>(ratio), 1.0 / 5.255)));
}

}  // namespace sensor_model
