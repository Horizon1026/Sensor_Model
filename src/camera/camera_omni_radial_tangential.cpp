#include "camera_omni_radial_tangential.h"

#include "cmath"
#include "stdexcept"

namespace sensor_model {

bool CameraOmniRadialTangential::DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) const {
    const float x = undistort_xy(0);
    const float y = undistort_xy(1);
    const float xy = x * y;
    const float x2 = x * x;
    const float y2 = y * y;
    const float r2 = x2 + y2;
    const float r4 = r2 * r2;
    const float r6 = r4 * r2;
    const float temp = 1.0f + k_[0] * r2 + k_[1] * r4 + k_[2] * r6;
    distort_xy(0) = x * temp + 2.0f * p_[0] * xy + p_[1] * (r2 + 2.0f * x2);
    distort_xy(1) = y * temp + 2.0f * p_[1] * xy + p_[0] * (r2 + 2.0f * y2);

    return distort_xy.allFinite();
}

bool CameraOmniRadialTangential::UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) const {
    switch (options().kUndistortMethod) {
        case UndistortMethod::kGradientDesent:
        case UndistortMethod::kNewtonIteration:
            return UndistortIteratively(distort_xy, undistort_xy, true);
        case UndistortMethod::kFixedPointIteration:
        case UndistortMethod::kBisection:  // No scalar bisection for 2D tangential distortion.
        default:
            return UndistortIteratively(distort_xy, undistort_xy, false);
    }
}

void CameraOmniRadialTangential::SetDistortionParameter(const std::vector<float> &params) {
    if (params.size() != 5) {
        throw std::invalid_argument("CameraOmniRadialTangential expects {k1, k2, k3, p1, p2}");
    }
    for (const float param: params) {
        if (!std::isfinite(param)) {
            throw std::invalid_argument("CameraOmniRadialTangential parameters must be finite");
        }
    }
    k_[0] = params[0];
    k_[1] = params[1];
    k_[2] = params[2];
    p_[0] = params[3];
    p_[1] = params[4];
}

void CameraOmniRadialTangential::GetDistortionParameter(std::vector<float> &params) const {
    params.reserve(5);
    params.clear();
    params.emplace_back(k_[0]);
    params.emplace_back(k_[1]);
    params.emplace_back(k_[2]);
    params.emplace_back(p_[0]);
    params.emplace_back(p_[1]);
}

bool CameraOmniRadialTangential::UndistortIteratively(const Vec2 &distort_xy, Vec2 &undistort_xy, bool use_newton) const {
    undistort_xy = distort_xy;
    const float tolerance = options().kMaxIterateStepLengthForUndistortion;
    if (!distort_xy.allFinite() || !std::isfinite(tolerance) || tolerance <= 0.0f) {
        return false;
    }

    for (uint32_t iter = 0; iter < options().kMaxIterationForUndistortion; ++iter) {
        Vec2 predicted = Vec2::Zero();
        if (!DistortOnNormalizedPlane(undistort_xy, predicted)) {
            return false;
        }
        const Vec2 residual = distort_xy - predicted;
        if (residual.norm() <= tolerance) {
            return true;
        }
        Vec2 delta = residual;
        if (use_newton) {
            const float x = undistort_xy.x();
            const float y = undistort_xy.y();
            const float r2 = undistort_xy.squaredNorm();
            const float r4 = r2 * r2;
            const float radial = 1.0f + k_[0] * r2 + k_[1] * r4 + k_[2] * r4 * r2;
            const float derivative = 2.0f * k_[0] + 4.0f * k_[1] * r2 + 6.0f * k_[2] * r4;
            Mat2 jacobian;
            jacobian << radial + x * x * derivative + 2.0f * p_[0] * y + 6.0f * p_[1] * x, x * y * derivative + 2.0f * p_[0] * x + 2.0f * p_[1] * y,
                x * y * derivative + 2.0f * p_[0] * x + 2.0f * p_[1] * y, radial + y * y * derivative + 6.0f * p_[0] * y + 2.0f * p_[1] * x;
            if (!jacobian.allFinite()) {
                return false;
            }
            const auto solver = jacobian.fullPivLu();
            if (!solver.isInvertible()) {
                return false;
            }
            delta = solver.solve(residual);
        }
        if (!delta.allFinite()) {
            return false;
        }
        undistort_xy += delta;
    }
    Vec2 predicted = Vec2::Zero();
    return DistortOnNormalizedPlane(undistort_xy, predicted) && (predicted - distort_xy).norm() <= tolerance;
}

}  // namespace sensor_model
