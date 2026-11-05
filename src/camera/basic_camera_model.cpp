#include "basic_camera_model.h"

#include "cmath"

namespace sensor_model {

/* Specialized Template Class Declaration. */
template class BasicCameraModel<float>;
template class BasicCameraModel<double>;

template <typename Scalar>
BasicCameraModel<Scalar>::BasicCameraModel(Scalar fx, Scalar fy, Scalar cx, Scalar cy, int32_t image_rows, int32_t image_cols):
    fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_rows_(image_rows > 0 ? image_rows : static_cast<int32_t>(cy * 2)),
    image_cols_(image_cols > 0 ? image_cols : static_cast<int32_t>(cx * 2)) {}

template <typename Scalar>
void BasicCameraModel<Scalar>::LiftFromNormalizedPlaneToImagePlane(const TVec2<Scalar> norm_xy, TVec2<Scalar> &pixel_uv) const {
    pixel_uv(0) = fx_ * norm_xy(0) + cx_;
    pixel_uv(1) = fy_ * norm_xy(1) + cy_;
}

template <typename Scalar>
void BasicCameraModel<Scalar>::LiftFromBearingVectorToImagePlane(const TVec3<Scalar> bearing_vector, TVec2<Scalar> &pixel_uv) const {
    TVec2<Scalar> norm_xy = TVec2<Scalar>::Zero();
    LiftFromBearingVectorToNormalizedPlane(bearing_vector, norm_xy);
    LiftFromNormalizedPlaneToImagePlane(norm_xy, pixel_uv);
}

template <typename Scalar>
void BasicCameraModel<Scalar>::LiftFromImagePlaneToBearingVector(const TVec2<Scalar> pixel_uv, TVec3<Scalar> &bearing_vector) const {
    TVec2<Scalar> norm_xy = TVec2<Scalar>::Zero();
    LiftFromImagePlaneToNormalizedPlane(pixel_uv, norm_xy);
    LiftFromNormalizedPlaneToBearingVector(norm_xy, bearing_vector);
}

template <typename Scalar>
void BasicCameraModel<Scalar>::SetImageSize(int32_t image_rows, int32_t image_cols) {
    image_rows_ = image_rows;
    image_cols_ = image_cols;
}

template <typename Scalar>
void BasicCameraModel<Scalar>::SetCommonIntrinsicParameter(Scalar fx, Scalar fy, Scalar cx, Scalar cy) {
    fx_ = fx;
    fy_ = fy;
    cx_ = cx;
    cy_ = cy;
}

template <typename Scalar>
bool BasicCameraModel<Scalar>::JacobiansForDistortedImagePlaneResidual(const TVec2<Scalar> distorted_pixel_uv, TMat<Scalar> &jacobian_dr_dprojection,
                                                                       TMat<Scalar> &jacobian_dr_ddistortion) const {
    if (!distorted_pixel_uv.allFinite() || !std::isfinite(fx_) || !std::isfinite(fy_) || !std::isfinite(cx_) || !std::isfinite(cy_) || fx_ == 0 || fy_ == 0) {
        return false;
    }
    TVec2<Scalar> distort_xy;
    LiftFromImagePlaneToNormalizedPlane(distorted_pixel_uv, distort_xy);
    if (!distort_xy.allFinite()) {
        return false;
    }
    jacobian_dr_dprojection.resize(2, 4);
    jacobian_dr_dprojection << distort_xy.x(), 0, 1, 0, 0, distort_xy.y(), 0, 1;
    jacobian_dr_ddistortion.resize(2, 0);
    return true;
}

}  // namespace sensor_model
