#include "camera_pinhole_kannala_brandt.h"

#include "slam_basic_math.h"
#include "slam_log_reporter.h"

namespace sensor_model {

/* Specialized Template Class Declaration. */
template class CameraPinholeKannalaBrandt<float>;
template class CameraPinholeKannalaBrandt<double>;

template <typename Scalar>
bool CameraPinholeKannalaBrandt<Scalar>::DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const {
    const Scalar r = undistort_xy.norm();
    if (r < kZeroFloat) {
        distort_xy = undistort_xy;
        return true;
    }
    const Scalar theta = std::atan(r);
    const Scalar theta2 = theta * theta;
    const Scalar theta3 = theta2 * theta;
    const Scalar theta4 = theta3 * theta;
    const Scalar theta5 = theta4 * theta;
    const Scalar theta6 = theta5 * theta;
    const Scalar theta_d = theta * k_[0] + theta2 * k_[1] + theta3 * k_[2] + theta4 * k_[3] + theta5 * k_[4] + theta6 * k_[5];

    distort_xy = theta_d * undistort_xy / r;
    return true;
}

template <typename Scalar>
bool CameraPinholeKannalaBrandt<Scalar>::UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const {
    switch (this->options().kUndistortMethod) {
        case BasicCameraModel<Scalar>::UndistortMethod::kFixedPointIteration:
        default:
            return UndistortByFixePointIteration(distort_xy, undistort_xy);
    }
}


template <typename Scalar>
void CameraPinholeKannalaBrandt<Scalar>::SetDistortionParameter(const std::vector<Scalar> &params) {
    for (uint32_t i = 0; i < 6; ++i) {
        k_[i] = params[i];
    }
}

template <typename Scalar>
void CameraPinholeKannalaBrandt<Scalar>::GetDistortionParameter(std::vector<Scalar> &params) const {
    params.clear();
    params.reserve(5);
    for (uint32_t i = 0; i < 6; ++i) {
        params.emplace_back(k_[i]);
    }
}

template <typename Scalar>
bool CameraPinholeKannalaBrandt<Scalar>::UndistortByFixePointIteration(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy) const {
    // Set initial value.
    undistort_xy = distort_xy;

    // Iterate to solve it;
    for (uint32_t iter = 0; iter < this->options().kMaxIterationForUndistortion; ++iter) {
        // Compute f(x) = residual.
        TVec2<Scalar> predict_distort_xy;
        DistortOnNormalizedPlane(undistort_xy, predict_distort_xy);
        const TVec2<Scalar> f = distort_xy - predict_distort_xy;

        // Compute g(x) = x + k * f(x)
        const Scalar k = 1;
        const TVec2<Scalar> g = undistort_xy + k * f;

        // Solve step.
        const TVec2<Scalar> delta = g - undistort_xy;
        const Scalar step_len = delta.squaredNorm();

        // Update undistort_xy.
        if (std::isnan(step_len) || std::isinf(step_len)) {
            return false;
        }
        undistort_xy = g;

        // Check if converged.
        if (std::sqrt(step_len) < this->options().kMaxIterateStepLengthForUndistortion) {
            break;
        }
    }

    return true;
}

}  // namespace sensor_model
