#include "camera_pinhole_equidistant.h"
#include "slam_basic_math.h"

#include "algorithm"
#include "cmath"
#include "limits"

namespace sensor_model {

/* Specialized Template Class Declaration. */
template class CameraPinholeEquidistant<float>;
template class CameraPinholeEquidistant<double>;

template <typename Scalar>
void CameraPinholeEquidistant<Scalar>::EvaluateDistortion(Scalar theta, Scalar &radius, Scalar &derivative) const {
    const Scalar theta2 = theta * theta;
    radius = theta;
    derivative = 1;
    Scalar power = theta2;
    for (uint32_t i = 0; i < 5; ++i) {
        radius += k_[i] * theta * power;
        derivative += (2 * i + 3) * k_[i] * power;
        power *= theta2;
    }
}

template <typename Scalar>
void CameraPinholeEquidistant<Scalar>::LiftFromBearingVectorToImagePlane(const TVec3<Scalar> bearing_vector, TVec2<Scalar> &pixel_uv) const {
    const Scalar radius = bearing_vector.template head<2>().norm();
    if (radius == 0) {
        if (bearing_vector.z() > 0) {
            pixel_uv << this->cx(), this->cy();
        } else {
            pixel_uv.setConstant(std::numeric_limits<Scalar>::quiet_NaN());
        }
        return;
    }
    Scalar rho = 0;
    Scalar derivative = 0;
    EvaluateDistortion(std::atan2(radius, bearing_vector.z()), rho, derivative);
    pixel_uv << this->fx() * rho * bearing_vector.x() / radius + this->cx(), this->fy() * rho * bearing_vector.y() / radius + this->cy();
}

template <typename Scalar>
void CameraPinholeEquidistant<Scalar>::LiftFromImagePlaneToBearingVector(const TVec2<Scalar> pixel_uv, TVec3<Scalar> &bearing_vector) const {
    const TVec2<Scalar> distorted_xy((pixel_uv.x() - this->cx()) / this->fx(), (pixel_uv.y() - this->cy()) / this->fy());
    const Scalar radius = distorted_xy.norm();
    Scalar theta = 0;
    if (!UndistortAngle(radius, theta)) {
        bearing_vector.setConstant(std::numeric_limits<Scalar>::quiet_NaN());
        return;
    }
    bearing_vector << 0, 0, std::cos(theta);
    if (radius > 0) {
        bearing_vector.template head<2>() = (std::sin(theta) / radius) * distorted_xy;
    }
}

template <typename Scalar>
bool CameraPinholeEquidistant<Scalar>::DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const {
    const Scalar radius = undistort_xy.norm();
    if (radius < kZeroFloat) {
        distort_xy = undistort_xy;
        return true;
    }
    Scalar rho = 0;
    Scalar derivative = 0;
    EvaluateDistortion(std::atan(radius), rho, derivative);
    distort_xy = (rho / radius) * undistort_xy;
    return distort_xy.allFinite();
}

template <typename Scalar>
bool CameraPinholeEquidistant<Scalar>::UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const {
    const Scalar radius = distort_xy.norm();
    Scalar theta = 0;
    if (!UndistortAngle(radius, theta) || theta >= std::acos(Scalar(-1)) / 2) {
        return false;
    }
    undistort_xy.setZero();
    if (radius > 0) {
        undistort_xy = (std::tan(theta) / radius) * distort_xy;
    }
    return true;
}

template <typename Scalar>
void CameraPinholeEquidistant<Scalar>::SetDistortionParameter(const std::vector<Scalar> &params) {
    for (uint32_t i = 0; i < 5; ++i) {
        k_[i] = params[i];
    }
}

template <typename Scalar>
void CameraPinholeEquidistant<Scalar>::GetDistortionParameter(std::vector<Scalar> &params) const {
    params.clear();
    params.reserve(5);
    for (uint32_t i = 0; i < 5; ++i) {
        params.emplace_back(k_[i]);
    }
}


template <typename Scalar>
bool CameraPinholeEquidistant<Scalar>::UndistortAngle(Scalar radius, Scalar &theta) const {
    if (!std::isfinite(radius)) {
        return false;
    }
    theta = 0;
    if (radius == 0) {
        return true;
    }
    const Scalar pi = std::acos(Scalar(-1));
    Scalar lower = 0;
    Scalar upper = 0;
    Scalar rho = 0;
    Scalar derivative = 0;
    for (uint32_t i = 1; i <= 64; ++i) {
        upper = pi * i / 64;
        EvaluateDistortion(upper, rho, derivative);
        if (rho >= radius) {
            break;
        }
        if (derivative <= 0 || i == 64) {
            return false;
        }
        lower = upper;
    }
    theta = (lower + upper) / 2;
    const Scalar tolerance = std::max(this->options().kMaxIterateStepLengthForUndistortion, 4 * std::numeric_limits<Scalar>::epsilon());
    for (uint32_t iter = 0; iter < this->options().kMaxIterationForUndistortion; ++iter) {
        EvaluateDistortion(theta, rho, derivative);
        const Scalar residual = rho - radius;
        if (std::fabs(residual) <= tolerance * std::max(Scalar(1), radius)) {
            return derivative > 0 && theta < pi;
        }
        if (residual > 0) {
            upper = theta;
        } else {
            lower = theta;
        }
        const Scalar candidate = theta - residual / derivative;
        theta = derivative > 0 && candidate >= lower && candidate <= upper ? candidate : (lower + upper) / 2;
    }
    return false;
}

template <typename Scalar>
bool CameraPinholeEquidistant<Scalar>::JacobiansForDistortedImagePlaneResidual(const TVec3<Scalar> prediction_undistorted_bearing_xyz,
                                                                               TMat<Scalar> &jacobian_dr_dprojection,
                                                                               TMat<Scalar> &jacobian_dr_ddistortion) const {
    TMat<Scalar> jacobian_dr_dp_c;
    return JacobiansForDistortedImagePlaneResidual(prediction_undistorted_bearing_xyz, jacobian_dr_dprojection, jacobian_dr_ddistortion, jacobian_dr_dp_c);
}

template <typename Scalar>
bool CameraPinholeEquidistant<Scalar>::JacobiansForDistortedImagePlaneResidual(const TVec3<Scalar> prediction_undistorted_bearing_xyz,
                                                                               TMat<Scalar> &jacobian_dr_dprojection, TMat<Scalar> &jacobian_dr_ddistortion,
                                                                               TMat<Scalar> &jacobian_dr_dp_c) const {
    const auto &p_c = prediction_undistorted_bearing_xyz;
    const Scalar radius2 = p_c.template head<2>().squaredNorm();
    const Scalar radius = std::sqrt(radius2);
    const Scalar norm2 = p_c.squaredNorm();
    if (!p_c.allFinite() || norm2 == 0 || (radius == 0 && p_c.z() <= 0)) {
        return false;
    }
    const Scalar theta = std::atan2(radius, p_c.z());
    const Scalar theta2 = theta * theta;
    Scalar rho = 0;
    Scalar derivative = 0;
    EvaluateDistortion(theta, rho, derivative);
    TVec2<Scalar> direction_xy = TVec2<Scalar>::Zero();
    if (radius > 0) {
        direction_xy = p_c.template head<2>() / radius;
    }
    const TVec2<Scalar> distorted_xy = rho * direction_xy;
    jacobian_dr_dprojection.resize(2, 4);
    jacobian_dr_dprojection << distorted_xy.x(), 0, 1, 0, 0, distorted_xy.y(), 0, 1;
    jacobian_dr_ddistortion.resize(2, 5);
    Scalar power = theta * theta2;
    for (uint32_t i = 0; i < 5; ++i) {
        jacobian_dr_ddistortion(0, i) = this->fx() * direction_xy.x() * power;
        jacobian_dr_ddistortion(1, i) = this->fy() * direction_xy.y() * power;
        power *= theta2;
    }
    jacobian_dr_dp_c.setZero(2, 3);
    if (p_c.z() <= 0 || radius > kZeroFloat * std::sqrt(norm2)) {
        const Scalar scale = rho / radius;
        jacobian_dr_dp_c.template leftCols<2>() = scale * TMat2<Scalar>::Identity() + ((derivative * p_c.z() / norm2 - scale) / radius2) *
                                                                                          p_c.template head<2>() * p_c.template head<2>().transpose();
        jacobian_dr_dp_c.col(2) = -(derivative / norm2) * p_c.template head<2>();
    } else {
        const Scalar inverse_z = 1 / p_c.z();
        jacobian_dr_dp_c << inverse_z, 0, -p_c.x() * inverse_z * inverse_z, 0, inverse_z, -p_c.y() * inverse_z * inverse_z;
    }
    jacobian_dr_dp_c.row(0) *= this->fx();
    jacobian_dr_dp_c.row(1) *= this->fy();
    return true;
}

}  // namespace sensor_model
