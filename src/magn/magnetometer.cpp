#include "magnetometer.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"

namespace sensor_model {

float Magnetometer::FormatDegree(const float abnormal_degree) {
    if (abnormal_degree < -180.0f) {
        return abnormal_degree + 180.0f * 2.0f;
    } else if (abnormal_degree > 180.0f) {
        return abnormal_degree - 180.0f * 2.0f;
    }

    return abnormal_degree;
}

float Magnetometer::ConvertMagnToYaw(const MagnMeasurement &magn, const float roll_deg, const float pitch_deg) {
    // Extract magnetic readings in body frame (x-forward, y-right, z-down, NED convention).
    const float mx = magn.magn_mG.x();
    const float my = magn.magn_mG.y();
    const float mz = magn.magn_mG.z();

    // Convert roll and pitch from degrees to radians.
    const float roll = roll_deg * kDegToRad;
    const float pitch = pitch_deg * kDegToRad;

    const float cos_roll = std::cos(roll);
    const float sin_roll = std::sin(roll);
    const float cos_pitch = std::cos(pitch);
    const float sin_pitch = std::sin(pitch);

    // Tilt compensation: project magnetic field from body frame onto the
    // horizontal plane, removing the effect of roll and pitch.
    // Ref: Madgwick et al. "Estimation of IMU and MARG orientation using
    //       a gradient descent algorithm", 2011.
    //
    //   Hx = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch)
    //   Hy = my * cos(roll) - mz * sin(roll)
    const float x_h = mx * cos_pitch +
                      my * sin_roll * sin_pitch +
                      mz * cos_roll * sin_pitch;
    const float y_h = my * cos_roll - mz * sin_roll;

    // Degenerate case: negligible horizontal magnetic field (e.g. at magnetic poles).
    if (std::abs(x_h) < kZeroFloat && std::abs(y_h) < kZeroFloat) {
        ReportWarn("[Magnetometer] Negligible horizontal magnetic field, "
                   << "unable to compute heading. Returning 0.0.");
        return 0.0f;
    }

    // Compute heading (clockwise from North) using atan2(-Hy, Hx).
    // atan2(y, x) gives counter-clockwise from +x; heading is clockwise
    // from North (+x), so the sign of y is flipped.
    float yaw_rad = std::atan2(-y_h, x_h);

    // Convert to degrees and wrap to [-180, 180].
    float yaw_deg = yaw_rad * kRadToDeg;
    return FormatDegree(yaw_deg);
}

}  // namespace sensor_model
