#include "camera_pinhole_radial_tangential.h"

#include "slam_basic_math.h"

namespace sensor_model {

/* Specialized Template Class Declaration. */
template class CameraPinholeRadialTangential<float>;
template class CameraPinholeRadialTangential<double>;

template <typename Scalar>
bool CameraPinholeRadialTangential<Scalar>::DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const {
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

    return true;
}

template <typename Scalar>
bool CameraPinholeRadialTangential<Scalar>::UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const {
    switch (this->options().kUndistortMethod) {
        case BasicCameraModel<Scalar>::UndistortMethod::kGradientDesent:
            return UndistortByGradienDesent(distort_xy, undistort_xy);
        case BasicCameraModel<Scalar>::UndistortMethod::kFixedPointIteration:
        default:
            return UndistortByFixePointIteration(distort_xy, undistort_xy);
    }
}

template <typename Scalar>
void CameraPinholeRadialTangential<Scalar>::SetDistortionParameter(const std::vector<Scalar> &params) {
    k_[0] = params[0];
    k_[1] = params[1];
    k_[2] = params[2];
    p_[0] = params[3];
    p_[1] = params[4];
}

template <typename Scalar>
void CameraPinholeRadialTangential<Scalar>::GetDistortionParameter(std::vector<Scalar> &params) const {
    params.reserve(5);
    params.clear();
    params.emplace_back(k_[0]);
    params.emplace_back(k_[1]);
    params.emplace_back(k_[2]);
    params.emplace_back(p_[0]);
    params.emplace_back(p_[1]);
}

template <typename Scalar>
bool CameraPinholeRadialTangential<Scalar>::UndistortByGradienDesent(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy) const {
    // Set initial value.
    undistort_xy = distort_xy;

    // Iterate to solve it;
    for (uint32_t iter = 0; iter < this->options().kMaxIterationForUndistortion; ++iter) {
        // Compute temp value.
        Scalar &x = undistort_xy.x();
        Scalar &y = undistort_xy.y();
        const Scalar xy = x * y;
        const Scalar x2 = x * x;
        const Scalar y2 = y * y;
        const Scalar r2 = x2 + y2;
        const Scalar r4 = r2 * r2;
        const Scalar r6 = r4 * r2;
        const Scalar temp_xy = 1 + k_[0] * r2 + k_[1] * r4 + k_[2] * r6;
        const Scalar temp_x = 2 * k_[0] * x + 4 * k_[1] * r2 * x + 6 * k_[2] * r4 * x;
        const Scalar temp_y = 2 * k_[0] * y + 4 * k_[1] * r2 * y + 6 * k_[2] * r4 * y;

        // Compute residual.
        const TVec2<Scalar> residual =
            distort_xy - TVec2<Scalar>(x * temp_xy + 2 * p_[0] * xy + p_[1] * (r2 + 2 * x2), y * temp_xy + 2 * p_[1] * xy + p_[0] * (r2 + 2 * y2));

        // Compute jacobian, d_r / d_distort
        TMat2<Scalar> jacobian;
        jacobian << temp_xy + x * temp_x + 2 * p_[0] * y + p_[1] * 6 * x, x * temp_y + 2 * p_[0] * x + 2 * p_[1] * y,
            y * temp_x + 2 * p_[0] * x + 2 * p_[1] * y, temp_xy + y * temp_y + p_[0] * 6 * y + 2 * p_[1] * x;

        if (std::fabs(jacobian.determinant()) < kZeroFloat) {
            return false;
        }

        // Solve step.
        const TVec2<Scalar> delta = jacobian.ldlt().solve(residual);
        const Scalar step_len = delta.squaredNorm();

        // Update undistort_xy.
        if (std::isnan(step_len) || std::isinf(step_len)) {
            return false;
        }
        undistort_xy += delta;

        // Check if converged.
        if (std::sqrt(step_len) < this->options().kMaxIterateStepLengthForUndistortion) {
            break;
        }
    }

    return true;
}

template <typename Scalar>
bool CameraPinholeRadialTangential<Scalar>::UndistortByFixePointIteration(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy) const {
    // Set initial value.
    undistort_xy = distort_xy;

    // Iterate to solve it;
    for (uint32_t iter = 0; iter < this->options().kMaxIterationForUndistortion; ++iter) {
        // Compute temp value.
        Scalar &x = undistort_xy.x();
        Scalar &y = undistort_xy.y();
        const Scalar xy = x * y;
        const Scalar x2 = x * x;
        const Scalar y2 = y * y;
        const Scalar r2 = x2 + y2;
        const Scalar r4 = r2 * r2;
        const Scalar r6 = r4 * r2;
        const Scalar temp_xy = 1 + k_[0] * r2 + k_[1] * r4 + k_[2] * r6;

        // Compute f(x) = residual.
        const TVec2<Scalar> f =
            distort_xy - TVec2<Scalar>(x * temp_xy + 2 * p_[0] * xy + p_[1] * (r2 + 2 * x2), y * temp_xy + 2 * p_[1] * xy + p_[0] * (r2 + 2 * y2));

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
bool CameraPinholeRadialTangential<Scalar>::JacobiansForDistortedImagePlaneResidual(const TVec3<Scalar> prediction_undistorted_bearing_xyz,
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
    return jacobian_dr_dprojection.allFinite() && jacobian_dr_ddistortion.allFinite();
}

}  // namespace sensor_model
