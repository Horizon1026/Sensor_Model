#include "barometer.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"

namespace sensor_model {

namespace {
    // ISA standard atmosphere constants of the troposphere.
    constexpr float kPressureOfSeaLevel = 101.325f;                                           // kPa.
    constexpr float kTemperatureOfSeaLevel = 288.15f;                                         // K, i.e. 15 degC.
    constexpr float kTemperatureLapseRate = 0.0065f;                                          // K/m.
    constexpr float kScaleHeightOfSeaLevel = kTemperatureOfSeaLevel / kTemperatureLapseRate;  // m.
    constexpr float kBaroExponent = 1.0f / 5.255f;                                            // R * L / (g * M).
}  // namespace

float Barometer::ConvertBaroToAltitude(const BaroMeasurement &measure) {
    const float ratio = measure.pressure_Kpa / kPressureOfSeaLevel;
    if (ratio <= 0) {
        return 0.0f;
    }
    return kScaleHeightOfSeaLevel * (1.0f - std::pow(ratio, kBaroExponent));
}

float Barometer::ConvertBaroWithTemperatureToAltitude(const BaroMeasurement &measure) {
    const float ratio = measure.pressure_Kpa / kPressureOfSeaLevel;
    if (ratio <= 0) {
        return 0.0f;
    }

    // The standard barometric formula bakes the fixed ISA sea-level temperature into its scale height T / L.
    const float temperature_K = measure.temperature_degC + 273.15f;
    const float scale_height = temperature_K / kTemperatureLapseRate;

    return scale_height * (1.0f - std::pow(ratio, kBaroExponent));
}

}  // namespace sensor_model
