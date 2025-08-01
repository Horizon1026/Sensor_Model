#include "pinhole.h"
#include "slam_basic_math.h"

namespace SENSOR_MODEL {

bool Pinhole::DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) {
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

    return true;
}

bool Pinhole::UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) {
    switch (options().kUndistortMethod) {
        case UndistortMethod::kGradientDesent:
            return UndistortByGradienDesent(distort_xy, undistort_xy);
        case UndistortMethod::kFixedPointIteration:
        default:
            return UndistortByFixePointIteration(distort_xy, undistort_xy);
    }
}

void Pinhole::SetDistortionParameter(const std::vector<float> &params) {
    k_[0] = params[0];
    k_[1] = params[1];
    k_[2] = params[2];
    p_[0] = params[3];
    p_[1] = params[4];
}

bool Pinhole::UndistortByGradienDesent(const Vec2 &distort_xy, Vec2 &undistort_xy) {
    // Set initial value.
    undistort_xy = distort_xy;

    // Iterate to solve it;
    for (uint32_t iter = 0; iter < options().kMaxIterationForUndistortion; ++iter) {
        // Compute temp value.
        float &x = undistort_xy.x();
        float &y = undistort_xy.y();
        const float xy = x * y;
        const float x2 = x * x;
        const float y2 = y * y;
        const float r2 = x2 + y2;
        const float r4 = r2 * r2;
        const float r6 = r4 * r2;
        const float temp_xy = 1.0f + k_[0] * r2 + k_[1] * r4 + k_[2] * r6;
        const float temp_x = 2.0f * k_[0] * x + 4.0f * k_[1] * r2 * x + 6.0f * k_[2] * r4 * x;
        const float temp_y = 2.0f * k_[0] * y + 4.0f * k_[1] * r2 * y + 6.0f * k_[2] * r4 * y;

        // Compute residual.
        const Vec2 residual = distort_xy - Vec2(x * temp_xy + 2.0f * p_[0] * xy + p_[1] * (r2 + 2.0f * x2),
                                                y * temp_xy + 2.0f * p_[1] * xy + p_[0] * (r2 + 2.0f * y2));

        // Compute jacobian, d_r / d_distort
        Mat2 jacobian;
        jacobian << temp_xy + x * temp_x + 2.0f * p_[0] * y + p_[1] * 6.0f * x,
                    x * temp_y + 2.0f * p_[0] * x + 2.0f * p_[1] * y,
                    y * temp_x + 2.0f * p_[0] * x + 2.0f * p_[1] * y,
                    temp_xy + y * temp_y + p_[0] * 6.0f * y + 2.0f * p_[1] * x;

        if (std::fabs(jacobian.determinant()) < kZeroFloat) {
            return false;
        }

        // Solve step.
        const Vec2 delta = jacobian.ldlt().solve(residual);
        const float step_len = delta.squaredNorm();

        // Update undistort_xy.
        if (std::isnan(step_len) || std::isinf(step_len)) {
            return false;
        }
        undistort_xy += delta;

        // Check if converged.
        if (std::sqrt(step_len) < options().kMaxIterateStepLengthForUndistortion) {
            break;
        }
    }

    return true;
}

bool Pinhole::UndistortByFixePointIteration(const Vec2 &distort_xy, Vec2 &undistort_xy) {
    // Set initial value.
    undistort_xy = distort_xy;

    // Iterate to solve it;
    for (uint32_t iter = 0; iter < options().kMaxIterationForUndistortion; ++iter) {
        // Compute temp value.
        float &x = undistort_xy.x();
        float &y = undistort_xy.y();
        const float xy = x * y;
        const float x2 = x * x;
        const float y2 = y * y;
        const float r2 = x2 + y2;
        const float r4 = r2 * r2;
        const float r6 = r4 * r2;
        const float temp_xy = 1.0f + k_[0] * r2 + k_[1] * r4 + k_[2] * r6;

        // Compute f(x) = residual.
        const Vec2 f = distort_xy - Vec2(x * temp_xy + 2.0f * p_[0] * xy + p_[1] * (r2 + 2.0f * x2),
                                      y * temp_xy + 2.0f * p_[1] * xy + p_[0] * (r2 + 2.0f * y2));

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
