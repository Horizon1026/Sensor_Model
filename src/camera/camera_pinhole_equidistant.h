#ifndef _SENSOR_MODEL_CAMERA_PINHOLE_EQUIDISTANT_MODEL_H_
#define _SENSOR_MODEL_CAMERA_PINHOLE_EQUIDISTANT_MODEL_H_

#include "camera_pinhole.h"

namespace sensor_model {

class CameraPinholeEquidistant: public CameraPinhole {

public:
    CameraPinholeEquidistant(): CameraPinhole() {}
    CameraPinholeEquidistant(float fx, float fy, float cx, float cy): CameraPinhole(fx, fy, cx, cy) {}
    CameraPinholeEquidistant(float fx, float fy, float cx, float cy, int32_t image_rows, int32_t image_cols):
        CameraPinhole(fx, fy, cx, cy, image_rows, image_cols) {}
    virtual ~CameraPinholeEquidistant() = default;
    CameraPinholeEquidistant(const CameraPinholeEquidistant &pinhole) = delete;

public:
    /*  Kannala-Brandt(equidistant) model.
        r(theta) = k0 * theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9 + ... */
    virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) override;
    virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) override;

    virtual void SetDistortionParameter(const std::vector<float> &params) override;
    const float &k1() const { return k_[0]; }
    const float &k2() const { return k_[1]; }
    const float &k3() const { return k_[2]; }
    const float &k4() const { return k_[3]; }
    const float &k5() const { return k_[4]; }

private:
    // Different method to do undistortion.
    bool UndistortByFixePointIteration(const Vec2 &distort_xy, Vec2 &undistort_xy);

private:
    // Distortion model parameters.
    std::array<float, 5> k_ = {};
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_PINHOLE_EQUIDISTANT_MODEL_H_
