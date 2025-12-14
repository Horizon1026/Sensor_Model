#include "camera_pinhole_equidistant.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"

namespace sensor_model {

bool CameraPinholeEquidistant::DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) const {
    const float r = undistort_xy.norm();
    if (r < kZeroFloat) {
        distort_xy = undistort_xy;
        return true;
    }
    const float theta = std::atan(r);
    const float theta2 = theta * theta;
    const float theta4 = theta2 * theta2;
    const float theta6 = theta4 * theta2;
    const float theta8 = theta6 * theta2;
    const float theta10 = theta8 * theta2;
    const float thetad = theta * (1 + k_[0] * theta2 + k_[1] * theta4 + k_[2] * theta6 + k_[3] * theta8 + k_[4] * theta10);

    distort_xy = thetad * undistort_xy / r;

    return true;
}

bool CameraPinholeEquidistant::UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) const {
    switch (options().kUndistortMethod) {
        case UndistortMethod::kFixedPointIteration:
        default:
            return UndistortByFixePointIteration(distort_xy, undistort_xy);
    }
}


void CameraPinholeEquidistant::SetDistortionParameter(const std::vector<float> &params) {
    for (uint32_t i = 0; i < 5; ++i) {
        k_[i] = params[i];
    }
}

void CameraPinholeEquidistant::GetDistortionParameter(std::vector<float> &params) const {
    params.clear();
    params.reserve(5);
    for (uint32_t i = 0; i < 5; ++i) {
        params.emplace_back(k_[i]);
    }
}

bool CameraPinholeEquidistant::UndistortByFixePointIteration(const Vec2 &distort_xy, Vec2 &undistort_xy) const {
    // Set initial value.
    undistort_xy = distort_xy;

    // Iterate to solve it;
    for (uint32_t iter = 0; iter < options().kMaxIterationForUndistortion; ++iter) {
        // Compute f(x) = residual.
        Vec2 predict_distort_xy;
        DistortOnNormalizedPlane(undistort_xy, predict_distort_xy);
        const Vec2 f = distort_xy - predict_distort_xy;

        // Compute g(x) = x + k * f(x)
        const float k = 1.f;
        const Vec2 g = undistort_xy + k * f;

        // Solve step.
        const Vec2 delta = g - undistort_xy;
        const float step_len = delta.squaredNorm();

        // Update undistort_xy.
        if (std::isnan(step_len) || std::isinf(step_len)) {
            return false;
        }
        undistort_xy = g;

        // Check if converged.
        if (std::sqrt(step_len) < options().kMaxIterateStepLengthForUndistortion) {
            break;
        }
    }

    return true;
}

}  // namespace sensor_model
