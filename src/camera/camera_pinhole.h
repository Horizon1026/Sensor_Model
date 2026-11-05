#ifndef _SENSOR_MODEL_CAMERA_PINHOLE_H_
#define _SENSOR_MODEL_CAMERA_PINHOLE_H_

#include "basic_camera_model.h"

namespace sensor_model {

template <typename Scalar = float>
class CameraPinhole: public BasicCameraModel<Scalar> {
public:
    CameraPinhole() = default;
    CameraPinhole(Scalar fx, Scalar fy, Scalar cx, Scalar cy): BasicCameraModel<Scalar>(fx, fy, cx, cy) {}
    CameraPinhole(Scalar fx, Scalar fy, Scalar cx, Scalar cy, int32_t image_rows, int32_t image_cols):
        BasicCameraModel<Scalar>(fx, fy, cx, cy, image_rows, image_cols) {}
    virtual ~CameraPinhole() = default;
    CameraPinhole(const CameraPinhole &) = delete;

public:
    virtual std::string CameraModelName() const override { return "Pinhole-Rectify"; }

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
    virtual bool CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, Scalar scale = 1) const override;
    void SetIntrinsicParameter(Scalar fx, Scalar fy, Scalar cx, Scalar cy);
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_PINHOLE_H_
