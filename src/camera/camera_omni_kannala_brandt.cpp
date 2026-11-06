#include "camera_omni_kannala_brandt.h"

#include "algorithm"
#include "cmath"
#include "limits"
#include "slam_basic_math.h"

namespace sensor_model {

/* Specialized Template Class Declaration. */
template class CameraOmniKannalaBrandt<float>;
template class CameraOmniKannalaBrandt<double>;

template <typename Scalar>
bool CameraOmniKannalaBrandt<Scalar>::DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const {
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
    const Scalar theta12 = theta10 * theta2;
    const Scalar theta_d = theta * (1 + k_[0] * theta2 + k_[1] * theta4 + k_[2] * theta6 + k_[3] * theta8 + k_[4] * theta10 + k_[5] * theta12);

    distort_xy = theta_d * undistort_xy / r;
    return distort_xy.allFinite();
}

template <typename Scalar>
bool CameraOmniKannalaBrandt<Scalar>::UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const {
    switch (this->options().kUndistortMethod) {
        case BasicCameraModel<Scalar>::UndistortMethod::kNewtonIteration:
        default:
            return UndistortByNewtonIteration(distort_xy, undistort_xy);
    }
}

template <typename Scalar>
void CameraOmniKannalaBrandt<Scalar>::SetDistortionParameter(const std::vector<Scalar> &params) {
    for (uint32_t i = 0; i < 6; ++i) {
        k_[i] = params[i];
    }
}

template <typename Scalar>
void CameraOmniKannalaBrandt<Scalar>::GetDistortionParameter(std::vector<Scalar> &params) const {
    params.clear();
    params.reserve(6);
    for (uint32_t i = 0; i < 6; ++i) {
        params.emplace_back(k_[i]);
    }
}

template <typename Scalar>
bool CameraOmniKannalaBrandt<Scalar>::UndistortByNewtonIteration(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy) const {
    const uint32_t max_iteration = this->options().kMaxIterationForUndistortion;
    Scalar tolerance = this->options().kMaxIterateStepLengthForUndistortion;
    if (!distort_xy.allFinite() || max_iteration == 0 || !std::isfinite(tolerance) || tolerance <= 0) {
        return false;
    }
    const Scalar radius = distort_xy.stableNorm();
    if (radius == 0) {
        undistort_xy.setZero();
        return true;
    }
    Scalar lower = 0;
    Scalar upper = std::nextafter(std::acos(Scalar(-1)) / 2, Scalar(0));
    const Scalar upper2 = upper * upper;
    Scalar polynomial = k_[5];
    for (int32_t i = 4; i >= 0; --i) {
        polynomial = k_[i] + upper2 * polynomial;
    }
    const Scalar limit = upper * (1 + upper2 * polynomial);
    if (!std::isfinite(radius) || !std::isfinite(limit) || radius >= limit) {
        return false;
    }
    Scalar theta = std::min(radius, upper);
    tolerance = std::max(tolerance, 4 * std::numeric_limits<Scalar>::epsilon());
    for (uint32_t iteration = 0; iteration < max_iteration; ++iteration) {
        const Scalar theta2 = theta * theta;
        Scalar radial_polynomial = k_[5];
        Scalar derivative_polynomial = 13 * k_[5];
        for (int32_t i = 4; i >= 0; --i) {
            radial_polynomial = k_[i] + theta2 * radial_polynomial;
            derivative_polynomial = (2 * i + 3) * k_[i] + theta2 * derivative_polynomial;
        }
        const Scalar residual = theta * (1 + theta2 * radial_polynomial) - radius;
        const Scalar derivative = 1 + theta2 * derivative_polynomial;
        if (!std::isfinite(residual) || !std::isfinite(derivative) || derivative <= 0) {
            return false;
        }
        if (std::fabs(residual) <= tolerance * std::max(Scalar(1), radius)) {
            const TVec2<Scalar> result = distort_xy * (std::tan(theta) / radius);
            if (!result.allFinite()) {
                return false;
            }
            undistort_xy = result;
            return true;
        }
        if (residual > 0) {
            upper = theta;
        } else {
            lower = theta;
        }
        const Scalar candidate = theta - residual / derivative;
        theta = candidate > lower && candidate < upper ? candidate : (lower + upper) / 2;
    }
    return false;
}

template <typename Scalar>
bool CameraOmniKannalaBrandt<Scalar>::JacobiansForDistortedImagePlaneResidual(const TVec3<Scalar> prediction_undistorted_bearing_xyz,
                                                                              TMat<Scalar> &jacobian_dr_dprojection,
                                                                              TMat<Scalar> &jacobian_dr_ddistortion) const {
    if (!BasicCameraModel<Scalar>::JacobiansForDistortedImagePlaneResidual(prediction_undistorted_bearing_xyz, jacobian_dr_dprojection,
                                                                           jacobian_dr_ddistortion)) {
        return false;
    }
    const TVec3<Scalar> bearing_vector = prediction_undistorted_bearing_xyz / prediction_undistorted_bearing_xyz.stableNorm();
    TVec2<Scalar> undistort_xy;
    this->LiftFromCameraFrameToNormalizedPlane(bearing_vector, undistort_xy);
    jacobian_dr_ddistortion.setZero(2, 6);
    const Scalar r = undistort_xy.norm();
    Scalar radial_derivative = 1;
    if (r == kZeroFloat) {
        return false;
    }
    if (r >= kZeroFloat) {
        const Scalar theta = std::atan(r);
        Scalar power = theta * theta * theta;
        for (uint32_t i = 0; i < 6; ++i) {
            jacobian_dr_ddistortion(0, i) = this->fx() * undistort_xy.x() * power / r;
            jacobian_dr_ddistortion(1, i) = this->fy() * undistort_xy.y() * power / r;
            power *= theta * theta;
        }
        const Scalar theta2 = theta * theta;
        const Scalar theta4 = theta2 * theta2;
        const Scalar theta6 = theta4 * theta2;
        const Scalar theta8 = theta6 * theta2;
        const Scalar theta10 = theta8 * theta2;
        const Scalar theta12 = theta10 * theta2;
        radial_derivative =
            (1 + 3 * k_[0] * theta2 + 5 * k_[1] * theta4 + 7 * k_[2] * theta6 + 9 * k_[3] * theta8 + 11 * k_[4] * theta10 + 13 * k_[5] * theta12) / (1 + r * r);
    }
    const Scalar denominator = bearing_vector.z() + this->xi();
    if (!bearing_vector.allFinite() || !(denominator > 0)) {
        return false;
    }
    jacobian_dr_dprojection.conservativeResize(2, 5);
    jacobian_dr_dprojection(0, 4) = -this->fx() * radial_derivative * undistort_xy.x() / denominator;
    jacobian_dr_dprojection(1, 4) = -this->fy() * radial_derivative * undistort_xy.y() / denominator;
    return jacobian_dr_dprojection.allFinite() && jacobian_dr_ddistortion.allFinite();
}

}  // namespace sensor_model
