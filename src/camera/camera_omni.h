#ifndef _SENSOR_MODEL_CAMERA_OMNI_H_
#define _SENSOR_MODEL_CAMERA_OMNI_H_

#include "basic_camera_model.h"

namespace sensor_model {

// Unified omnidirectional camera model (Mei):
// normalized_xy = p_c.xy / (p_c.z + xi * ||p_c||).
// Bearing vectors represent physical unit rays.
// For xi > 1 only the injective branch z / ||p_c|| > -1 / xi is used.
// Invalid lifts return NaNs; bool operations return false on invalid input.
template <typename Scalar = float>
class CameraOmni: public BasicCameraModel<Scalar> {
public:
    CameraOmni() = default;
    CameraOmni(Scalar fx, Scalar fy, Scalar cx, Scalar cy, Scalar xi = 0);
    CameraOmni(Scalar fx, Scalar fy, Scalar cx, Scalar cy, Scalar xi, int32_t image_rows, int32_t image_cols);
    virtual ~CameraOmni() = default;
    CameraOmni(const CameraOmni &) = delete;

public:
    virtual std::string CameraModelName() const override { return "Omni-Rectify"; }

    // Projection and lens operations use the common interface in BasicCameraModel.
    virtual void LiftFromNormalizedPlaneToBearingVector(const TVec2<Scalar> norm_xy, TVec3<Scalar> &bearing_vector) const override;
    virtual void LiftFromBearingVectorToNormalizedPlane(const TVec3<Scalar> bearing_vector, TVec2<Scalar> &norm_xy) const override;
    virtual void LiftFromImagePlaneToNormalizedPlane(const TVec2<Scalar> pixel_uv, TVec2<Scalar> &norm_xy) const override;
    virtual bool LiftFromRawImagePlaneToUndistortedNormalizedPlane(const TVec2<Scalar> pixel_uv, TVec2<Scalar> &undistort_xy) const override;
    virtual void LiftFromCameraFrameToNormalizedPlane(const TVec3<Scalar> p_c, TVec2<Scalar> &norm_xy) const override;
    virtual bool DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const override;
    virtual bool UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const override;
    virtual bool DistortOnImagePlane(const TVec2<Scalar> undistort_uv, TVec2<Scalar> &distort_uv) const override;
    virtual bool UndistortOnImagePlane(const TVec2<Scalar> distort_uv, TVec2<Scalar> &undistort_uv) const override;
    // Removes lens distortion while retaining the omni projection and xi.
    // Input/output buffers must be distinct and have the same dimensions.
    virtual bool CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, Scalar scale = 1) const override;
    virtual bool JacobiansForDistortedImagePlaneResidual(const TVec2<Scalar> distorted_pixel_uv, TMat<Scalar> &jacobian_dr_dprojection,
                                                         TMat<Scalar> &jacobian_dr_ddistortion) const override;

    // xi must be finite and nonnegative, otherwise throws std::invalid_argument.
    void SetIntrinsicParameter(Scalar fx, Scalar fy, Scalar cx, Scalar cy, Scalar xi);

    // Const reference for member variables.
    const Scalar &xi() const { return xi_; }

private:
    Scalar xi_ = 0;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_OMNI_H_
