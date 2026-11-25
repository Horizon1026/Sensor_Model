#include "barometer.h"

#include "slam_operations.h"

#include "slam_basic_math.h"
#include "slam_log_reporter.h"

#include "cstdio"
#include "fstream"

namespace sensor_model {

bool Barometer::ParseMeasuresFromFile(const std::string &file_path, const double &time_stamp_offset_s, std::vector<BaroMeasurement> &measures) {
    std::ifstream file(file_path);
    std::string line;
    RETURN_FALSE_IF(!file.is_open() || !std::getline(file, line));

    measures.clear();
    while (std::getline(file, line)) {
        double data[3] = {};
        CONTINUE_IF(std::sscanf(line.c_str(), "%lf,%lf,%lf", &data[0], &data[1], &data[2]) != 3);
        BaroMeasurement measure;
        measure.time_stamp_s = data[0] * 1e-9 - time_stamp_offset_s;
        measure.pressure_Kpa = static_cast<float>(data[1]);
        measure.temperature_degC = static_cast<float>(data[2]);
        measures.emplace_back(measure);
    }
    return !measures.empty();
}

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
