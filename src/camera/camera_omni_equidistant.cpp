#include "camera_omni_equidistant.h"

#include "slam_basic_math.h"
#include "stdexcept"

namespace sensor_model {

/* Specialized Template Class Declaration. */
template class CameraOmniEquidistant<float>;
template class CameraOmniEquidistant<double>;

template <typename Scalar>
bool CameraOmniEquidistant<Scalar>::DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const {
    const Scalar r = undistort_xy.norm();
    if (r < kZeroFloat) {
        distort_xy = undistort_xy;
        return true;
    }
    const Scalar theta = std::atan(r);
    const Scalar theta2 = theta * theta;
    const Scalar theta4 = theta2 * theta2;
    const Scalar theta6 = theta4 * theta2;
    const Scalar theta8 = theta6 * theta2;
    const Scalar theta10 = theta8 * theta2;
    const Scalar theta_d = theta * (1 + k_[0] * theta2 + k_[1] * theta4 + k_[2] * theta6 + k_[3] * theta8 + k_[4] * theta10);

    distort_xy = theta_d * undistort_xy / r;
    return distort_xy.allFinite();
}

template <typename Scalar>
bool CameraOmniEquidistant<Scalar>::UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const {
    switch (this->options().kUndistortMethod) {
        case BasicCameraModel<Scalar>::UndistortMethod::kFixedPointIteration:
        default:
            return UndistortByFixePointIteration(distort_xy, undistort_xy);
    }
}


template <typename Scalar>
void CameraOmniEquidistant<Scalar>::SetDistortionParameter(const std::vector<Scalar> &params) {
    if (params.size() != 5) {
        throw std::invalid_argument("CameraOmniEquidistant expects 5 distortion parameters");
    }
    for (const Scalar param: params) {
        if (!std::isfinite(param)) {
            throw std::invalid_argument("CameraOmniEquidistant parameters must be finite");
        }
    }
    for (uint32_t i = 0; i < 5; ++i) {
        k_[i] = params[i];
    }
}

template <typename Scalar>
void CameraOmniEquidistant<Scalar>::GetDistortionParameter(std::vector<Scalar> &params) const {
    params.clear();
    params.reserve(5);
    for (uint32_t i = 0; i < 5; ++i) {
        params.emplace_back(k_[i]);
    }
}

template <typename Scalar>
bool CameraOmniEquidistant<Scalar>::UndistortByFixePointIteration(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy) const {
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

template <typename Scalar>
bool CameraOmniEquidistant<Scalar>::JacobiansForDistortedImagePlaneResidual(const TVec2<Scalar> distorted_pixel_uv, TMat<Scalar> &jacobian_dr_dprojection,
                                                                            TMat<Scalar> &jacobian_dr_ddistortion) const {
    if (!BasicCameraModel<Scalar>::JacobiansForDistortedImagePlaneResidual(distorted_pixel_uv, jacobian_dr_dprojection, jacobian_dr_ddistortion)) {
        return false;
    }
    TVec2<Scalar> undistort_xy, distort_xy, predicted_xy;
    const Scalar tolerance = this->options().kMaxIterateStepLengthForUndistortion;
    if (!std::isfinite(tolerance) || tolerance <= 0 || !this->LiftFromRawImagePlaneToUndistortedNormalizedPlane(distorted_pixel_uv, undistort_xy) ||
        !undistort_xy.allFinite()) {
        return false;
    }
    this->LiftFromImagePlaneToNormalizedPlane(distorted_pixel_uv, distort_xy);
    const Scalar allowed_error = std::max(tolerance, 8 * std::numeric_limits<Scalar>::epsilon() * (1 + distort_xy.norm()));
    if (!DistortOnNormalizedPlane(undistort_xy, predicted_xy) || !predicted_xy.allFinite() || (predicted_xy - distort_xy).norm() > allowed_error) {
        return false;
    }
    jacobian_dr_ddistortion.setZero(2, 5);
    const Scalar r = undistort_xy.norm();
    // Match the small-radius identity branch in DistortOnNormalizedPlane.
    Scalar radial_derivative = 1;
    if (r == kZeroFloat) {
        return false;
    }
    if (r >= kZeroFloat) {
        const Scalar theta = std::atan(r);
        Scalar power = theta * theta * theta;
        for (uint32_t i = 0; i < 5; ++i) {
            jacobian_dr_ddistortion(0, i) = this->fx() * undistort_xy.x() * power / r;
            jacobian_dr_ddistortion(1, i) = this->fy() * undistort_xy.y() * power / r;
            power *= theta * theta;
        }
        const Scalar theta2 = theta * theta;
        const Scalar theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 = theta6 * theta2, theta10 = theta8 * theta2;
        radial_derivative = (1 + 3 * k_[0] * theta2 + 5 * k_[1] * theta4 + 7 * k_[2] * theta6 + 9 * k_[3] * theta8 + 11 * k_[4] * theta10) / (1 + r * r);
    }
    TVec3<Scalar> bearing_vector;
    this->LiftFromNormalizedPlaneToBearingVector(undistort_xy, bearing_vector);
    const Scalar denominator = bearing_vector.z() + this->xi();
    if (!bearing_vector.allFinite() || !(denominator > 0)) {
        return false;
    }
    jacobian_dr_dprojection.conservativeResize(2, 5);
    // The xi increment changes norm_xy along its radial direction.
    jacobian_dr_dprojection(0, 4) = -this->fx() * radial_derivative * undistort_xy.x() / denominator;
    jacobian_dr_dprojection(1, 4) = -this->fy() * radial_derivative * undistort_xy.y() / denominator;
    return jacobian_dr_dprojection.allFinite() && jacobian_dr_ddistortion.allFinite();
}

}  // namespace sensor_model
