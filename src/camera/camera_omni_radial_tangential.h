#ifndef _SENSOR_MODEL_CAMERA_OMNI_RADTAN_MODEL_H_
#define _SENSOR_MODEL_CAMERA_OMNI_RADTAN_MODEL_H_

#include "camera_omni.h"

#include "array"

namespace sensor_model {

/* Class CameraOmniRadialTangential Declaration. */
class CameraOmniRadialTangential: public CameraOmni {

public:
    CameraOmniRadialTangential(): CameraOmni() {}
    CameraOmniRadialTangential(float fx, float fy, float cx, float cy, float xi = 0.0f): CameraOmni(fx, fy, cx, cy, xi) {}
    CameraOmniRadialTangential(float fx, float fy, float cx, float cy, float xi, int32_t image_rows, int32_t image_cols):
        CameraOmni(fx, fy, cx, cy, xi, image_rows, image_cols) {}
    virtual ~CameraOmniRadialTangential() = default;
    CameraOmniRadialTangential(const CameraOmniRadialTangential &omni) = delete;

public:
    virtual std::string CameraModelName() const override { return "Omni-RadialTangential"; }

    /*  Radial-Tangential model:
        x_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * x + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        y_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * y + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y */
    virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) const override;
    virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) const override;

    // Exactly {k1, k2, k3, p1, p2}; other sizes throw std::invalid_argument.
    virtual void SetDistortionParameter(const std::vector<float> &params) override;
    virtual void GetDistortionParameter(std::vector<float> &params) const override;
    const float &k1() const { return k_[0]; }
    const float &k2() const { return k_[1]; }
    const float &k3() const { return k_[2]; }
    const float &p1() const { return p_[0]; }
    const float &p2() const { return p_[1]; }

private:
    // Different method to do undistortion.
    bool UndistortIteratively(const Vec2 &distort_xy, Vec2 &undistort_xy, bool use_newton) const;

private:
    // Distortion model parameters.
    std::array<float, 3> k_ = {};
    std::array<float, 2> p_ = {};
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_OMNI_RADTAN_MODEL_H_
