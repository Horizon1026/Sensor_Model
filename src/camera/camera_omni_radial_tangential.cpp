#include "camera_omni_radial_tangential.h"

#include "cmath"
#include "stdexcept"

#include "algorithm"
#include "limits"

namespace sensor_model {

/* Specialized Template Class Declaration. */
template class CameraOmniRadialTangential<float>;
template class CameraOmniRadialTangential<double>;

template <typename Scalar>
bool CameraOmniRadialTangential<Scalar>::DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const {
    const Scalar x = undistort_xy(0);
    const Scalar y = undistort_xy(1);
    const Scalar xy = x * y;
    const Scalar x2 = x * x;
    const Scalar y2 = y * y;
    const Scalar r2 = x2 + y2;
    const Scalar r4 = r2 * r2;
    const Scalar r6 = r4 * r2;
    const Scalar temp = 1 + k_[0] * r2 + k_[1] * r4 + k_[2] * r6;
    distort_xy(0) = x * temp + 2 * p_[0] * xy + p_[1] * (r2 + 2 * x2);
    distort_xy(1) = y * temp + 2 * p_[1] * xy + p_[0] * (r2 + 2 * y2);

    return distort_xy.allFinite();
}

template <typename Scalar>
bool CameraOmniRadialTangential<Scalar>::UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const {
    switch (this->options().kUndistortMethod) {
        case BasicCameraModel<Scalar>::UndistortMethod::kGradientDesent:
        case BasicCameraModel<Scalar>::UndistortMethod::kNewtonIteration:
            return UndistortIteratively(distort_xy, undistort_xy, true);
        case BasicCameraModel<Scalar>::UndistortMethod::kFixedPointIteration:
        case BasicCameraModel<Scalar>::UndistortMethod::kBisection:  // No scalar bisection for 2D tangential distortion.
        default:
            return UndistortIteratively(distort_xy, undistort_xy, false);
    }
}

template <typename Scalar>
void CameraOmniRadialTangential<Scalar>::SetDistortionParameter(const std::vector<Scalar> &params) {
    if (params.size() != 5) {
        throw std::invalid_argument("CameraOmniRadialTangential expects {k1, k2, k3, p1, p2}");
    }
    for (const Scalar param: params) {
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

template <typename Scalar>
void CameraOmniRadialTangential<Scalar>::GetDistortionParameter(std::vector<Scalar> &params) const {
    params.reserve(5);
    params.clear();
    params.emplace_back(k_[0]);
    params.emplace_back(k_[1]);
    params.emplace_back(k_[2]);
    params.emplace_back(p_[0]);
    params.emplace_back(p_[1]);
}

template <typename Scalar>
bool CameraOmniRadialTangential<Scalar>::UndistortIteratively(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy, bool use_newton) const {
    undistort_xy = distort_xy;
    const Scalar tolerance = this->options().kMaxIterateStepLengthForUndistortion;
    if (!distort_xy.allFinite() || !std::isfinite(tolerance) || tolerance <= 0) {
        return false;
    }

    for (uint32_t iter = 0; iter < this->options().kMaxIterationForUndistortion; ++iter) {
        TVec2<Scalar> predicted = TVec2<Scalar>::Zero();
        if (!DistortOnNormalizedPlane(undistort_xy, predicted)) {
            return false;
        }
        const TVec2<Scalar> residual = distort_xy - predicted;
        if (residual.norm() <= tolerance) {
            return true;
        }
        TVec2<Scalar> delta = residual;
        if (use_newton) {
            const Scalar x = undistort_xy.x();
            const Scalar y = undistort_xy.y();
            const Scalar r2 = undistort_xy.squaredNorm();
            const Scalar r4 = r2 * r2;
            const Scalar radial = 1 + k_[0] * r2 + k_[1] * r4 + k_[2] * r4 * r2;
            const Scalar derivative = 2 * k_[0] + 4 * k_[1] * r2 + 6 * k_[2] * r4;
            TMat2<Scalar> jacobian;
            jacobian << radial + x * x * derivative + 2 * p_[0] * y + 6 * p_[1] * x, x * y * derivative + 2 * p_[0] * x + 2 * p_[1] * y,
                x * y * derivative + 2 * p_[0] * x + 2 * p_[1] * y, radial + y * y * derivative + 6 * p_[0] * y + 2 * p_[1] * x;
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
    TVec2<Scalar> predicted = TVec2<Scalar>::Zero();
    return DistortOnNormalizedPlane(undistort_xy, predicted) && (predicted - distort_xy).norm() <= tolerance;
}

template <typename Scalar>
bool CameraOmniRadialTangential<Scalar>::JacobiansForDistortedImagePlaneResidual(const TVec3<Scalar> prediction_undistorted_bearing_xyz,
                                                                                 TMat<Scalar> &jacobian_dr_dprojection,
                                                                                 TMat<Scalar> &jacobian_dr_ddistortion) const {
    if (!BasicCameraModel<Scalar>::JacobiansForDistortedImagePlaneResidual(prediction_undistorted_bearing_xyz, jacobian_dr_dprojection,
                                                                           jacobian_dr_ddistortion)) {
        return false;
    }
    const TVec3<Scalar> bearing_vector = prediction_undistorted_bearing_xyz / prediction_undistorted_bearing_xyz.stableNorm();
    TVec2<Scalar> undistort_xy;
    this->LiftFromCameraFrameToNormalizedPlane(bearing_vector, undistort_xy);
    const Scalar x = undistort_xy.x(), y = undistort_xy.y();
    const Scalar r2 = undistort_xy.squaredNorm(), r4 = r2 * r2, r6 = r4 * r2;
    jacobian_dr_ddistortion.resize(2, 5);
    jacobian_dr_ddistortion << this->fx() * x * r2, this->fx() * x * r4, this->fx() * x * r6, this->fx() * 2 * x * y, this->fx() * (r2 + 2 * x * x),
        this->fy() * y * r2, this->fy() * y * r4, this->fy() * y * r6, this->fy() * (r2 + 2 * y * y), this->fy() * 2 * x * y;
    const Scalar denominator = bearing_vector.z() + this->xi();
    if (!bearing_vector.allFinite() || !(denominator > 0)) {
        return false;
    }
    const Scalar radial = 1 + k_[0] * r2 + k_[1] * r4 + k_[2] * r6;
    const Scalar derivative = 2 * k_[0] + 4 * k_[1] * r2 + 6 * k_[2] * r4;
    const Scalar jxx = radial + x * x * derivative + 2 * p_[0] * y + 6 * p_[1] * x;
    const Scalar jxy = x * y * derivative + 2 * p_[0] * x + 2 * p_[1] * y;
    const Scalar jyy = radial + y * y * derivative + 6 * p_[0] * y + 2 * p_[1] * x;
    jacobian_dr_dprojection.conservativeResize(2, 5);
    // Chain d pixel / d norm_xy with d norm_xy / d xi at a fixed physical ray.
    jacobian_dr_dprojection(0, 4) = -this->fx() * (jxx * x + jxy * y) / denominator;
    jacobian_dr_dprojection(1, 4) = -this->fy() * (jxy * x + jyy * y) / denominator;
    return jacobian_dr_dprojection.allFinite() && jacobian_dr_ddistortion.allFinite();
}

}  // namespace sensor_model
