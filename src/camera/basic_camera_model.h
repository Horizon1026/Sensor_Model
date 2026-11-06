#ifndef _SENSOR_MODEL_BASIC_CAMERA_MODEL_H_
#define _SENSOR_MODEL_BASIC_CAMERA_MODEL_H_

#include "basic_type.h"
#include "datatype_image.h"

#include "array"
#include "string"
#include "vector"

namespace sensor_model {

// Shared camera state and projection interface. Valid projection domains
// are defined by each concrete camera model.
template <typename Scalar = float>
class BasicCameraModel {
public:
    enum class UndistortMethod : uint8_t {
        kGradientDesent = 0,
        kFixedPointIteration = 1,
        kBisection = 2,
        kNewtonIteration = 3,
    };

    struct Options {
        UndistortMethod kUndistortMethod = UndistortMethod::kGradientDesent;
        uint32_t kMaxIterationForUndistortion = 10;
        Scalar kMaxIterateStepLengthForUndistortion = static_cast<Scalar>(1e-6);
    };

public:
    virtual ~BasicCameraModel() = default;
    BasicCameraModel(const BasicCameraModel &) = delete;
    virtual std::string CameraModelName() const = 0;

    // Lift 2d point in normalized plane to other coordinate systems.
    void LiftFromNormalizedPlaneToImagePlane(const TVec2<Scalar> norm_xy, TVec2<Scalar> &pixel_uv) const;
    virtual void LiftFromNormalizedPlaneToBearingVector(const TVec2<Scalar> norm_xy, TVec3<Scalar> &bearing_vector) const = 0;

    // Lift 3d point in bearing vector to other coordinate systems.
    virtual void LiftFromBearingVectorToNormalizedPlane(const TVec3<Scalar> bearing_vector, TVec2<Scalar> &norm_xy) const = 0;
    virtual void LiftFromBearingVectorToImagePlane(const TVec3<Scalar> bearing_vector, TVec2<Scalar> &pixel_uv) const;

    // Lift 2d point in image plane to other coordinate systems.
    virtual void LiftFromImagePlaneToNormalizedPlane(const TVec2<Scalar> pixel_uv, TVec2<Scalar> &norm_xy) const = 0;
    virtual void LiftFromImagePlaneToBearingVector(const TVec2<Scalar> pixel_uv, TVec3<Scalar> &bearing_vector) const;

    // Lift 2d point in raw image plane to undistorted normalized plane.
    virtual bool LiftFromRawImagePlaneToUndistortedNormalizedPlane(const TVec2<Scalar> pixel_uv, TVec2<Scalar> &undistort_xy) const = 0;

    // Lift 3d point in camera frame to normalized plane.
    virtual void LiftFromCameraFrameToNormalizedPlane(const TVec3<Scalar> p_c, TVec2<Scalar> &norm_xy) const = 0;

    // Do distortion and undistortion on normalized plane.
    virtual bool DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const = 0;
    virtual bool UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const = 0;

    // Do distortion and undistortion on image plane.
    virtual bool DistortOnImagePlane(const TVec2<Scalar> undistort_uv, TVec2<Scalar> &distort_uv) const = 0;
    virtual bool UndistortOnImagePlane(const TVec2<Scalar> distort_uv, TVec2<Scalar> &undistort_uv) const = 0;

    // Undistort image. GrayImage interpolation uses float coordinates and pixel values.
    virtual bool CorrectDistortedImage(const GrayImage &raw_image, GrayImage &corrected_image, Scalar scale = 1) const = 0;

    // For residual = predicted distorted pixel - observed pixel, holding the camera-frame ray fixed.
    // Input is a finite, nonzero camera-frame ray in the projection domain; unit length is not required.
    // Positive scaling does not change the outputs. Derivatives use forward projection, without undistortion.
    // Projection columns: [fx, fy, cx, cy] (+ xi for omni); distortion columns follow GetDistortionParameter().
    // Both outputs have two rows; no distortion gives zero columns. Outputs are valid only on success.
    virtual bool JacobiansForDistortedImagePlaneResidual(const TVec3<Scalar> prediction_undistorted_bearing_xyz, TMat<Scalar> &jacobian_dr_dprojection,
                                                         TMat<Scalar> &jacobian_dr_ddistortion) const;

    // Also differentiate with respect to the input camera-frame point, without unit-length normalization.
    virtual bool JacobiansForDistortedImagePlaneResidual(const TVec3<Scalar> prediction_undistorted_bearing_xyz, TMat<Scalar> &jacobian_dr_dprojection,
                                                         TMat<Scalar> &jacobian_dr_ddistortion, TMat<Scalar> &jacobian_dr_dp_c) const { return false; }

    // Set intrinsic and distortion parameters.
    void SetImageSize(int32_t image_rows, int32_t image_cols);
    virtual void SetDistortionParameter(const std::vector<Scalar> &params) {};
    virtual void GetDistortionParameter(std::vector<Scalar> &params) const { params.clear(); }

    // Reference for member variables.
    Options &options() { return options_; }

    // Const reference for member variables.
    const Options &options() const { return options_; }
    const Scalar &fx() const { return fx_; }
    const Scalar &fy() const { return fy_; }
    const Scalar &cx() const { return cx_; }
    const Scalar &cy() const { return cy_; }
    const int32_t &image_rows() const { return image_rows_; }
    const int32_t &image_cols() const { return image_cols_; }

protected:
    BasicCameraModel() = default;
    BasicCameraModel(Scalar fx, Scalar fy, Scalar cx, Scalar cy, int32_t image_rows = 0, int32_t image_cols = 0);
    // Concrete models expose their own intrinsic setters (omni also requires xi).
    void SetCommonIntrinsicParameter(Scalar fx, Scalar fy, Scalar cx, Scalar cy);

private:
    Options options_;

    Scalar fx_ = 0;
    Scalar fy_ = 0;
    Scalar cx_ = 0;
    Scalar cy_ = 0;

    int32_t image_rows_ = 0;
    int32_t image_cols_ = 0;
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_BASIC_CAMERA_MODEL_H_
