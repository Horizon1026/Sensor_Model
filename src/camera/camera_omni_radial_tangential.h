#ifndef _SENSOR_MODEL_CAMERA_OMNI_RADTAN_MODEL_H_
#define _SENSOR_MODEL_CAMERA_OMNI_RADTAN_MODEL_H_

#include "camera_omni.h"

#include "array"

namespace sensor_model {

/* Class CameraOmniRadialTangential Declaration. */
template <typename Scalar = float>
class CameraOmniRadialTangential: public CameraOmni<Scalar> {

public:
    CameraOmniRadialTangential(): CameraOmni<Scalar>() {}
    CameraOmniRadialTangential(Scalar fx, Scalar fy, Scalar cx, Scalar cy, Scalar xi = 0): CameraOmni<Scalar>(fx, fy, cx, cy, xi) {}
    CameraOmniRadialTangential(Scalar fx, Scalar fy, Scalar cx, Scalar cy, Scalar xi, int32_t image_rows, int32_t image_cols):
        CameraOmni<Scalar>(fx, fy, cx, cy, xi, image_rows, image_cols) {}
    virtual ~CameraOmniRadialTangential() = default;
    CameraOmniRadialTangential(const CameraOmniRadialTangential &omni) = delete;

public:
    virtual std::string CameraModelName() const override { return "Omni-RadialTangential"; }

    /*  Radial-Tangential model:
        x_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * x + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        y_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * y + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y */
    virtual bool DistortOnNormalizedPlane(const TVec2<Scalar> undistort_xy, TVec2<Scalar> &distort_xy) const override;
    virtual bool UndistortOnNormalizedPlane(const TVec2<Scalar> distort_xy, TVec2<Scalar> &undistort_xy) const override;

    // Exactly {k1, k2, k3, p1, p2}; other sizes throw std::invalid_argument.
    virtual void SetDistortionParameter(const std::vector<Scalar> &params) override;
    virtual void GetDistortionParameter(std::vector<Scalar> &params) const override;
    const Scalar &k1() const { return k_[0]; }
    const Scalar &k2() const { return k_[1]; }
    const Scalar &k3() const { return k_[2]; }
    const Scalar &p1() const { return p_[0]; }
    const Scalar &p2() const { return p_[1]; }

private:
    // Different method to do undistortion.
    bool UndistortIteratively(const TVec2<Scalar> &distort_xy, TVec2<Scalar> &undistort_xy, bool use_newton) const;

private:
    // Distortion model parameters.
    std::array<Scalar, 3> k_ = {};
    std::array<Scalar, 2> p_ = {};
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_OMNI_RADTAN_MODEL_H_
