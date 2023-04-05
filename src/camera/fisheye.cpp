#include "fisheye.h"
#include "math_kinematics.h"

namespace SensorModel {

// Lift 3d point in camera frame on normalized plane.
void Fisheye::LiftToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) {
    // TODO：
}

// Lift 3d point in camera frame on normalized plane, and do undistortion.
bool Fisheye::LiftToNormalizedPlaneAndUndistort(const Vec2 pixel_uv, Vec2 &undistort_xy) {
    Vec2 distort_xy;
    LiftBackToNormalizedPlane(pixel_uv, distort_xy);
    return UndistortOnNormalizedPlane(distort_xy, undistort_xy);
}

bool Fisheye::DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) {
    const float r = undistort_xy.norm();
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

bool Fisheye::UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) {
    switch (options().kUndistortMethod) {
        case UndistortMethod::kGradientDesent:
            return UndistortByGradienDesent(distort_xy, undistort_xy);
        case UndistortMethod::kFixePointIteration:
            return UndistortByFixePointIteration(distort_xy, undistort_xy);
        default:
            return false;
    }
}

void Fisheye::SetDistortionParameter(float k1, float k2, float k3, float k4, float k5) {
    k_[0] = k1;
    k_[1] = k2;
    k_[2] = k3;
    k_[3] = k4;
    k_[4] = k5;
}

bool Fisheye::UndistortByGradienDesent(const Vec2 &distort_xy, Vec2 &undistort_xy) {
    // Set initial value.
    undistort_xy = distort_xy;

    // Iterate to solve it;
    for (uint32_t iter = 0; iter < options().kMaxIterationForUndistortion; ++iter) {
        // TODO:
    }

    return true;
}

bool Fisheye::UndistortByFixePointIteration(const Vec2 &distort_xy, Vec2 &undistort_xy) {
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


}