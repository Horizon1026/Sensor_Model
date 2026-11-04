#include "camera_pinhole.h"

#include "slam_basic_math.h"
#include "slam_operations.h"

namespace sensor_model {

/* Specialized Template Class Declaration. */
template class CameraPinhole<float>;
template class CameraPinhole<double>;

// Lift 2d point in normalized plane to other coordinate systems.
template <typename Scalar>
void CameraPinhole<Scalar>::LiftFromNormalizedPlaneToUnitSphere(const TVec2<Scalar> norm_xy, TVec3<Scalar> &sphere_xyz) const {
    const Scalar yita = 2 / (1 + norm_xy.squaredNorm());
    sphere_xyz.template head<2>() = norm_xy * yita;
    sphere_xyz.z() = yita - 1;
}

template <typename Scalar>
void CameraPinhole<Scalar>::LiftFromNormalizedPlaneToBearingVector(const TVec2<Scalar> norm_xy, TVec3<Scalar> &bearing_vector) const {
    bearing_vector.template head<2>() = norm_xy;
    bearing_vector.z() = 1;
    bearing_vector.normalize();
}

// Lift 3d point in unit sphere to other coordinate systems.
template <typename Scalar>
void CameraPinhole<Scalar>::LiftFromUnitSphereToNormalizedPlane(const TVec3<Scalar> sphere_xyz, TVec2<Scalar> &norm_xy) const {
    const Scalar yita = sphere_xyz.z() + 1;
    norm_xy = sphere_xyz.template head<2>() / yita;
}

// Lift 3d point in bearing vector to other coordinate systems.
template <typename Scalar>
void CameraPinhole<Scalar>::LiftFromBearingVectorToNormalizedPlane(const TVec3<Scalar> bearing_vector, TVec2<Scalar> &norm_xy) const {
    norm_xy = bearing_vector.template head<2>() / bearing_vector.z();
}

// Lift 2d point in image plane back on normalized plane.
template <typename Scalar>
void CameraPinhole<Scalar>::LiftFromImagePlaneToNormalizedPlane(const TVec2<Scalar> pixel_uv, TVec2<Scalar> &norm_xy) const {
    norm_xy(0) = (pixel_uv(0) - this->cx()) / this->fx();
    norm_xy(1) = (pixel_uv(1) - this->cy()) / this->fy();
}

// Lift 3d point in camera frame on normalized plane.
template <typename Scalar>
void CameraPinhole<Scalar>::LiftFromCameraFrameToNormalizedPlane(const TVec3<Scalar> p_c, TVec2<Scalar> &norm_xy) const {
    if (p_c.z() < kZeroFloat) {
        norm_xy.setZero();
    } else {
        norm_xy.x() = p_c.x() / p_c.z();
        norm_xy.y() = p_c.y() / p_c.z();
    }
}

// Distort on normalized plane.
template <typename Scalar>
bool CameraPinhole<Scalar>::DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const {
    // For camera basic, view it as rectify image, so no distortion.
    distort_xy = undistort_xy;
    return true;
}

// Undistort on normalized plane.
template <typename Scalar>
bool CameraPinhole<Scalar>::UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const {
    // For camera basic, view it as rectify image, so no undistortion.
    undistort_xy = distort_xy;
    return true;
}

// Lift 3d point in image plane on normalized plane, and do undistortion.
template <typename Scalar>
bool CameraPinhole<Scalar>::LiftFromRawImagePlaneToUndistortedNormalizedPlane(const TVec2<Scalar> pixel_uv, TVec2<Scalar> &undistort_xy) const {
    TVec2<Scalar> distort_xy = TVec2<Scalar>::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, distort_xy);
    return UndistortOnNormalizedPlane(distort_xy, undistort_xy);
}

template <typename Scalar>
bool CameraPinhole<Scalar>::DistortOnImagePlane(const TVec2<Scalar> undistort_uv, TVec2<Scalar> &distort_uv) const {
    const TVec2<Scalar> undistort_xy = TVec2<Scalar>((undistort_uv(0) - this->cx()) / this->fx(), (undistort_uv(1) - this->cy()) / this->fy());
    TVec2<Scalar> distort_xy = TVec2<Scalar>::Zero();
    RETURN_FALSE_IF_FALSE(DistortOnNormalizedPlane(undistort_xy, distort_xy));
    this->LiftFromNormalizedPlaneToImagePlane(distort_xy, distort_uv);
    return true;
}

template <typename Scalar>
bool CameraPinhole<Scalar>::UndistortOnImagePlane(const TVec2<Scalar> distort_uv, TVec2<Scalar> &undistort_uv) const {
    const TVec2<Scalar> distort_xy = TVec2<Scalar>((distort_uv(0) - this->cx()) / this->fx(), (distort_uv(1) - this->cy()) / this->fy());
    TVec2<Scalar> undistort_xy = TVec2<Scalar>::Zero();
    RETURN_FALSE_IF_FALSE(UndistortOnNormalizedPlane(distort_xy, undistort_xy));
    this->LiftFromNormalizedPlaneToImagePlane(undistort_xy, undistort_uv);
    return true;
}

// Undistort image.
template <typename Scalar>
bool CameraPinhole<Scalar>::CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, Scalar scale) const {
    RETURN_FALSE_IF(raw_image.data() == nullptr || corrected_image.data() == nullptr);
    RETURN_FALSE_IF(raw_image.cols() != corrected_image.cols() || raw_image.rows() != corrected_image.rows());

    const int32_t rows = corrected_image.rows();
    const int32_t cols = corrected_image.cols();
    scale = std::max(static_cast<Scalar>(1e0), scale);

    TVec2<Scalar> distort_uv = TVec2<Scalar>::Zero();
    for (int32_t u = 0; u < cols; ++u) {
        for (int32_t v = 0; v < rows; ++v) {
            TVec2<Scalar> undistort_uv = TVec2<Scalar>(u, v);
            const TVec2<Scalar> mid_uv = TVec2<Scalar>(this->cx(), this->cy());
            undistort_uv = scale * (undistort_uv - mid_uv) + mid_uv;

            DistortOnImagePlane(undistort_uv, distort_uv);

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
void CameraPinhole<Scalar>::SetIntrinsicParameter(Scalar fx, Scalar fy, Scalar cx, Scalar cy) {
    this->SetCommonIntrinsicParameter(fx, fy, cx, cy);
}

}  // namespace sensor_model
