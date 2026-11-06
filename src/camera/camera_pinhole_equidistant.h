#ifndef _SENSOR_MODEL_CAMERA_PINHOLE_EQUIDISTANT_MODEL_H_
#define _SENSOR_MODEL_CAMERA_PINHOLE_EQUIDISTANT_MODEL_H_

#include "camera_pinhole.h"

namespace sensor_model {

/* Class CameraPinholeEquidistant Declaration. */
template <typename Scalar = float>
class CameraPinholeEquidistant: public CameraPinhole<Scalar> {

public:
    CameraPinholeEquidistant(): CameraPinhole<Scalar>() {}
    CameraPinholeEquidistant(Scalar fx, Scalar fy, Scalar cx, Scalar cy): CameraPinhole<Scalar>(fx, fy, cx, cy) {}
    CameraPinholeEquidistant(Scalar fx, Scalar fy, Scalar cx, Scalar cy, int32_t image_rows, int32_t image_cols):
        CameraPinhole<Scalar>(fx, fy, cx, cy, image_rows, image_cols) {}
    virtual ~CameraPinholeEquidistant() = default;
    CameraPinholeEquidistant(const CameraPinholeEquidistant &pinhole) = delete;

public:
    virtual std::string CameraModelName() const override { return "Pinhole-Equidistant"; }

    /*  Equidistant model.
        r(theta) = theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8 + k5 * theta10) */
    virtual bool DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const override;
    virtual bool UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const override;

    virtual bool JacobiansForDistortedImagePlaneResidual(const TVec3<Scalar> prediction_undistorted_bearing_xyz, TMat<Scalar> &jacobian_dr_dprojection,
                                                         TMat<Scalar> &jacobian_dr_ddistortion) const override;

    virtual void SetDistortionParameter(const std::vector<Scalar> &params) override;
    virtual void GetDistortionParameter(std::vector<Scalar> &params) const override;
    const Scalar &k1() const { return k_[0]; }
    const Scalar &k2() const { return k_[1]; }
    const Scalar &k3() const { return k_[2]; }
    const Scalar &k4() const { return k_[3]; }
    const Scalar &k5() const { return k_[4]; }

private:
    // Different method to do undistortion.
    bool UndistortByFixePointIteration(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy) const;

private:
    // Distortion model parameters.
    std::array<Scalar, 5> k_ = {};
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_PINHOLE_EQUIDISTANT_MODEL_H_
