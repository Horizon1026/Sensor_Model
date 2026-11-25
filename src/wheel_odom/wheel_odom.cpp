#include "wheel_odom.h"

#include "slam_operations.h"

#include "slam_basic_math.h"
#include "slam_log_reporter.h"

#include "cstdio"
#include "fstream"

namespace sensor_model {

bool WheelOdom::ParseMeasuresFromFile(const std::string &file_path, const double &time_stamp_offset_s, std::vector<WheelOdomMeasurement> &measures) {
    std::ifstream file(file_path);
    std::string line;
    RETURN_FALSE_IF(!file.is_open() || !std::getline(file, line));

    measures.clear();
    while (std::getline(file, line)) {
        double data[4] = {};
        CONTINUE_IF(std::sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &data[0], &data[1], &data[2], &data[3]) != 4);
        WheelOdomMeasurement measure;
        measure.time_stamp_s = data[0] * 1e-9 - time_stamp_offset_s;
        measure.is_wheel_velocity_valid = data[1] == 1;
        measure.encoder_left_cnt = static_cast<float>(data[2]);
        measure.encoder_right_cnt = static_cast<float>(data[3]);
        measures.emplace_back(measure);
    }
    return !measures.empty();
}

Vec3 WheelOdom::ConvertEncoderCountToVelocity(const WheelOdomMeasurement &measure) {
    if (!measure.is_wheel_velocity_valid) {
        return Vec3::Zero();
    }

    const float scale = options_.kWheelRadiusInMeter / options_.kEncoderCountInOneCircle / options_.kEncoderSamplePeriodInSecond * 2.0f * kPai;
    const float velocity_left_mps = scale * measure.encoder_left_cnt;
    const float velocity_right_mps = scale * measure.encoder_right_cnt;
    const float velocity = (velocity_left_mps + velocity_right_mps) * 0.5f;

    return Vec3(velocity, 0, 0);
}

Vec3 WheelOdom::ConvertLeftEncoderCountToVelocity(const WheelOdomMeasurement &measure) {
    if (!measure.is_wheel_velocity_valid) {
        return Vec3::Zero();
    }

    const float velocity_left_mps =
        options_.kWheelRadiusInMeter * measure.encoder_left_cnt / options_.kEncoderCountInOneCircle / options_.kEncoderSamplePeriodInSecond * 2.0f * kPai;

    return Vec3(velocity_left_mps, 0, 0);
}

Vec3 WheelOdom::ConvertRightEncoderCountToVelocity(const WheelOdomMeasurement &measure) {
    if (!measure.is_wheel_velocity_valid) {
        return Vec3::Zero();
    }

    const float velocity_right_mps =
        options_.kWheelRadiusInMeter * measure.encoder_right_cnt / options_.kEncoderCountInOneCircle / options_.kEncoderSamplePeriodInSecond * 2.0f * kPai;

    return Vec3(velocity_right_mps, 0, 0);
}

}  // namespace sensor_model
