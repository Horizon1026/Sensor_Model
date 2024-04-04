#include "wheel_odom.h"
#include "math_kinematics.h"
#include "log_report.h"

namespace SENSOR_MODEL {

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

    const float velocity_left_mps = options_.kWheelRadiusInMeter * measure.encoder_left_cnt /
        options_.kEncoderCountInOneCircle / options_.kEncoderSamplePeriodInSecond * 2.0f * kPai;

    return Vec3(velocity_left_mps, 0, 0);
}

Vec3 WheelOdom::ConvertRightEncoderCountToVelocity(const WheelOdomMeasurement &measure) {
    if (!measure.is_wheel_velocity_valid) {
        return Vec3::Zero();
    }

    const float velocity_right_mps = options_.kWheelRadiusInMeter * measure.encoder_right_cnt /
        options_.kEncoderCountInOneCircle / options_.kEncoderSamplePeriodInSecond * 2.0f * kPai;

    return Vec3(velocity_right_mps, 0, 0);
}

}
