#include "pinhole.h"
#include "math_kinematics.h"
#include "slam_operations.h"

namespace SensorModel {

// Lift 3d point in camera frame on normalized plane.
void Pinhole::LiftToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) {
    if (p_c.z() < kZero) {
        norm_xy.setZero();
    } else {
        norm_xy.x() = p_c.x() / p_c.z();
        norm_xy.y() = p_c.y() / p_c.z();
    }
}

bool Pinhole::DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv) {
    Vec2 undistort_xy = Vec2((undistort_uv(0) - cx()) / fx(),
                             (undistort_uv(1) - cy()) / fy());

    Vec2 distort_xy;
    RETURN_FALSE_IF_FALSE(DistortOnNormalizedPlane(undistort_xy, distort_xy));

    distort_uv(0) = distort_xy(0) * fx() + cx();
    distort_uv(1) = distort_xy(1) * fy() + cy();

    return true;
}

bool Pinhole::UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv) {
    Vec2 distort_xy = Vec2((distort_uv(0) - cx()) / fx(),
                           (distort_uv(1) - cy()) / fy());

    Vec2 undistort_xy;
    RETURN_FALSE_IF_FALSE(UndistortOnNormalizedPlane(distort_xy, undistort_xy));

    undistort_uv(0) = undistort_xy(0) * fx() + cx();
    undistort_uv(1) = undistort_xy(1) * fy() + cy();

    return true;
}

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
        case UndistortMethod::kFixePointIteration:
            return UndistortByFixePointIteration(distort_xy, undistort_xy);
        default:
            return false;
    }
}

void Pinhole::SetDistortionParameter(const float k1, const float k2, const float k3, const float p1, const float p2) {
    k_[0] = k1;
    k_[1] = k2;
    k_[2] = k3;
    p_[0] = p1;
    p_[1] = p2;
}

bool Pinhole::CorrectDistortedImage(const Image &raw_image, Image &corrected_image) {
    if (raw_image.image_data() == nullptr || corrected_image.image_data() == nullptr) {
        return false;
    }

    if (raw_image.cols() != corrected_image.cols() || raw_image.rows() != corrected_image.rows()) {
        return false;
    }

    const int32_t rows = corrected_image.rows();
    const int32_t cols = corrected_image.cols();

    Vec2 distort = Vec2::Zero();
    for (int32_t u = 0; u < cols; ++u) {
        for (int32_t v = 0; v < rows; ++v) {
            DistortOnImagePlane(Vec2(u, v), distort);

            float pixel_value = 0.0f;
            if (raw_image.GetPixelValue(distort.y(), distort.x(), &pixel_value)) {
                corrected_image.SetPixelValue(v, u, static_cast<uint8_t>(pixel_value));
            }
        }
    }

    return true;
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

        if (std::fabs(jacobian.determinant()) < kZero) {
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