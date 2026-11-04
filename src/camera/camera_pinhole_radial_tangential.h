#ifndef _SENSOR_MODEL_CAMERA_PINHOLE_RADTAN_MODEL_H_
#define _SENSOR_MODEL_CAMERA_PINHOLE_RADTAN_MODEL_H_

#include "camera_pinhole.h"

namespace sensor_model {

/* Class CameraPinholeRadialTangential Declaration. */
template <typename Scalar = float>
class CameraPinholeRadialTangential: public CameraPinhole<Scalar> {

public:
    CameraPinholeRadialTangential(): CameraPinhole<Scalar>() {}
    CameraPinholeRadialTangential(Scalar fx, Scalar fy, Scalar cx, Scalar cy): CameraPinhole<Scalar>(fx, fy, cx, cy) {}
    CameraPinholeRadialTangential(Scalar fx, Scalar fy, Scalar cx, Scalar cy, int32_t image_rows, int32_t image_cols):
        CameraPinhole<Scalar>(fx, fy, cx, cy, image_rows, image_cols) {}
    virtual ~CameraPinholeRadialTangential() = default;
    CameraPinholeRadialTangential(const CameraPinholeRadialTangential &pinhole) = delete;

public:
    virtual std::string CameraModelName() const override { return "Pinhole-RadialTangential"; }

    /*  Radial-Tangential model:
        x_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * x + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        y_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * y + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y */
    virtual bool DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const override;
    virtual bool UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const override;

    virtual void SetDistortionParameter(const std::vector<Scalar> &params) override;
    virtual void GetDistortionParameter(std::vector<Scalar> &params) const override;
    const Scalar &k1() const { return k_[0]; }
    const Scalar &k2() const { return k_[1]; }
    const Scalar &k3() const { return k_[2]; }
    const Scalar &p1() const { return p_[0]; }
    const Scalar &p2() const { return p_[1]; }

private:
    // Different method to do undistortion.
    bool UndistortByGradienDesent(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy) const;
    bool UndistortByFixePointIteration(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy) const;

private:
    // Distortion model parameters.
    std::array<Scalar, 3> k_ = {};
    std::array<Scalar, 2> p_ = {};
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_PINHOLE_RADTAN_MODEL_H_
