#include "wheel_odom.h"
#include "math_kinematics.h"
#include "log_report.h"

namespace SENSOR_MODEL {

Vec3 WheelOdom::ConvertEncoderCountToVelocity(const WheelOdomMeasurement &measure) {
    if (!measure.is_wheel_velocity_valid) {
        return Vec3::Zero();
    }

    const float velocity_left_mps = measure.encoder_left_cnt / options_.kEncoderCountInOneCircle;
    const float velocity_right_mps = measure.encoder_right_cnt / options_.kEncoderCountInOneCircle;
    const float velocity = (velocity_left_mps + velocity_right_mps) * 0.5f;

    return Vec3(velocity, 0, 0);
}

}
