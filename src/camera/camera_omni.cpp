#include "camera_omni.h"

#include "slam_operations.h"

#include "algorithm"
#include "cmath"
#include "limits"
#include "stdexcept"

namespace sensor_model {

/* Specialized Template Class Declaration. */
template class CameraOmni<float>;
template class CameraOmni<double>;

template <typename Scalar>
CameraOmni<Scalar>::CameraOmni(Scalar fx, Scalar fy, Scalar cx, Scalar cy, Scalar xi): CameraOmni(fx, fy, cx, cy, xi, 0, 0) {}

template <typename Scalar>
CameraOmni<Scalar>::CameraOmni(Scalar fx, Scalar fy, Scalar cx, Scalar cy, Scalar xi, int32_t image_rows, int32_t image_cols):
    BasicCameraModel<Scalar>(fx, fy, cx, cy, image_rows, image_cols) {
    SetIntrinsicParameter(fx, fy, cx, cy, xi);
}

// Lift 2d point in normalized plane to other coordinate systems.
template <typename Scalar>
void CameraOmni<Scalar>::LiftFromNormalizedPlaneToBearingVector(const TVec2<Scalar> norm_xy, TVec3<Scalar> &bearing_vector) const {
    const Scalar r2 = norm_xy.squaredNorm();
    const Scalar xi = xi_;
    const Scalar radicand = 1 + (1 - xi * xi) * r2;
    if (!norm_xy.allFinite() || radicand < 0) {
        bearing_vector.setConstant(std::numeric_limits<Scalar>::quiet_NaN());
        return;
    }
    const Scalar lambda = (xi + std::sqrt(radicand)) / (1 + r2);
    bearing_vector << lambda * norm_xy.x(), lambda * norm_xy.y(), lambda - xi;
    // The boundary is singular for xi > 1 and is excluded from projection.
    if (xi > 1 && radicand <= 0) {
        bearing_vector.setConstant(std::numeric_limits<Scalar>::quiet_NaN());
    }
}

// Lift 3d point in bearing vector to other coordinate systems.
template <typename Scalar>
void CameraOmni<Scalar>::LiftFromBearingVectorToNormalizedPlane(const TVec3<Scalar> bearing_vector, TVec2<Scalar> &norm_xy) const {
    LiftFromCameraFrameToNormalizedPlane(bearing_vector, norm_xy);
}

// Lift 2d point in image plane back on normalized plane.
template <typename Scalar>
void CameraOmni<Scalar>::LiftFromImagePlaneToNormalizedPlane(const TVec2<Scalar> pixel_uv, TVec2<Scalar> &norm_xy) const {
    if (!pixel_uv.allFinite() || !std::isfinite(this->fx()) || !std::isfinite(this->fy()) || this->fx() == 0 || this->fy() == 0) {
        norm_xy.setConstant(std::numeric_limits<Scalar>::quiet_NaN());
        return;
    }
    norm_xy(0) = (pixel_uv(0) - this->cx()) / this->fx();
    norm_xy(1) = (pixel_uv(1) - this->cy()) / this->fy();
}

// Lift 3d point in camera frame on normalized plane.
template <typename Scalar>
void CameraOmni<Scalar>::LiftFromCameraFrameToNormalizedPlane(const TVec3<Scalar> p_c, TVec2<Scalar> &norm_xy) const {
    const Scalar length = p_c.norm();
    const Scalar z = length > 0 ? p_c.z() / length : -1;
    const Scalar limit = xi_ <= 1 ? -xi_ : -1 / xi_;
    if (!p_c.allFinite() || length == 0 || z <= limit) {
        norm_xy.setConstant(std::numeric_limits<Scalar>::quiet_NaN());
        return;
    }
    const Scalar denominator = p_c.z() + xi_ * length;
    norm_xy = p_c.template head<2>() / denominator;
}

// Distort on normalized plane.
template <typename Scalar>
bool CameraOmni<Scalar>::DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const {
    // No lens distortion in the base omni model.
    distort_xy = undistort_xy;
    return distort_xy.allFinite();
}

// Undistort on normalized plane.
template <typename Scalar>
bool CameraOmni<Scalar>::UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const {
    // No lens distortion in the base omni model.
    undistort_xy = distort_xy;
    return undistort_xy.allFinite();
}

// Lift 3d point in image plane on normalized plane, and do undistortion.
template <typename Scalar>
bool CameraOmni<Scalar>::LiftFromRawImagePlaneToUndistortedNormalizedPlane(const TVec2<Scalar> pixel_uv, TVec2<Scalar> &undistort_xy) const {
    TVec2<Scalar> distort_xy = TVec2<Scalar>::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, distort_xy);
    RETURN_FALSE_IF_FALSE(UndistortOnNormalizedPlane(distort_xy, undistort_xy));
    TVec3<Scalar> bearing;
    LiftFromNormalizedPlaneToBearingVector(undistort_xy, bearing);
    return bearing.allFinite();
}

template <typename Scalar>
bool CameraOmni<Scalar>::DistortOnImagePlane(const TVec2<Scalar> undistort_uv, TVec2<Scalar> &distort_uv) const {
    TVec2<Scalar> undistort_xy;
    LiftFromImagePlaneToNormalizedPlane(undistort_uv, undistort_xy);
    TVec2<Scalar> distort_xy = TVec2<Scalar>::Zero();
    RETURN_FALSE_IF_FALSE(DistortOnNormalizedPlane(undistort_xy, distort_xy));
    this->LiftFromNormalizedPlaneToImagePlane(distort_xy, distort_uv);
    return distort_uv.allFinite();
}

template <typename Scalar>
bool CameraOmni<Scalar>::UndistortOnImagePlane(const TVec2<Scalar> distort_uv, TVec2<Scalar> &undistort_uv) const {
    TVec2<Scalar> distort_xy;
    LiftFromImagePlaneToNormalizedPlane(distort_uv, distort_xy);
    TVec2<Scalar> undistort_xy = TVec2<Scalar>::Zero();
    RETURN_FALSE_IF_FALSE(UndistortOnNormalizedPlane(distort_xy, undistort_xy));
    this->LiftFromNormalizedPlaneToImagePlane(undistort_xy, undistort_uv);
    return undistort_uv.allFinite();
}

// Undistort image.
template <typename Scalar>
bool CameraOmni<Scalar>::CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, Scalar scale) const {
    RETURN_FALSE_IF(raw_image.data() == nullptr || corrected_image.data() == nullptr);
    RETURN_FALSE_IF(raw_image.cols() != corrected_image.cols() || raw_image.rows() != corrected_image.rows());

    RETURN_FALSE_IF(raw_image.data() == corrected_image.data());
    RETURN_FALSE_IF(!std::isfinite(scale) || !std::isfinite(this->fx()) || !std::isfinite(this->fy()) || !std::isfinite(this->cx()) ||
                    !std::isfinite(this->cy()) || this->fx() == 0 || this->fy() == 0);

    const int32_t rows = corrected_image.rows();
    const int32_t cols = corrected_image.cols();
    scale = std::max(static_cast<Scalar>(1e0), scale);

    TVec2<Scalar> distort_uv = TVec2<Scalar>::Zero();
    for (int32_t u = 0; u < cols; ++u) {
        for (int32_t v = 0; v < rows; ++v) {
            TVec2<Scalar> undistort_uv = TVec2<Scalar>(u, v);
            const TVec2<Scalar> mid_uv = TVec2<Scalar>(this->cx(), this->cy());
            undistort_uv = scale * (undistort_uv - mid_uv) + mid_uv;

            TVec3<Scalar> bearing;
            this->LiftFromImagePlaneToBearingVector(undistort_uv, bearing);
            if (!bearing.allFinite() || !DistortOnImagePlane(undistort_uv, distort_uv)) {
                corrected_image.SetPixelValue(v, u, 0);
                continue;
            }

            float pixel_value = 0.0f;
            if (raw_image.GetPixelValue(static_cast<float>(distort_uv.y()), static_cast<float>(distort_uv.x()), &pixel_value)) {
                corrected_image.SetPixelValue(v, u, static_cast<uint8_t>(pixel_value));
            } else {
                corrected_image.SetPixelValue(v, u, 0);
            }
        }
    }

    return true;
}

template <typename Scalar>
void CameraOmni<Scalar>::SetIntrinsicParameter(Scalar fx, Scalar fy, Scalar cx, Scalar cy, Scalar xi) {
    if (!std::isfinite(xi) || xi < 0) {
        throw std::invalid_argument("CameraOmni xi must be finite and nonnegative");
    }
    xi_ = xi;
    this->SetCommonIntrinsicParameter(fx, fy, cx, cy);
}

template <typename Scalar>
bool CameraOmni<Scalar>::JacobiansForDistortedImagePlaneResidual(const TVec2<Scalar> distorted_pixel_uv, TMat<Scalar> &jacobian_dr_dprojection,
                                                                 TMat<Scalar> &jacobian_dr_ddistortion) const {
    if (!BasicCameraModel<Scalar>::JacobiansForDistortedImagePlaneResidual(distorted_pixel_uv, jacobian_dr_dprojection, jacobian_dr_ddistortion)) {
        return false;
    }
    TVec2<Scalar> norm_xy;
    LiftFromImagePlaneToNormalizedPlane(distorted_pixel_uv, norm_xy);
    TVec3<Scalar> bearing_vector;
    LiftFromNormalizedPlaneToBearingVector(norm_xy, bearing_vector);
    const Scalar denominator = bearing_vector.z() + xi_;
    if (!bearing_vector.allFinite() || !(denominator > 0)) {
        return false;
    }
    jacobian_dr_dprojection.conservativeResize(2, 5);
    // At a fixed physical ray, d norm_xy / d xi = -norm_xy / (bearing.z + xi).
    jacobian_dr_dprojection(0, 4) = -this->fx() * norm_xy.x() / denominator;
    jacobian_dr_dprojection(1, 4) = -this->fy() * norm_xy.y() / denominator;
    return jacobian_dr_dprojection.allFinite();
}

}  // namespace sensor_model
