#ifndef _SENSOR_MODEL_CAMERA_PINHOLE_RADTAN_MODEL_H_
#define _SENSOR_MODEL_CAMERA_PINHOLE_RADTAN_MODEL_H_

#include "camera_pinhole.h"

namespace sensor_model {

class CameraPinholeRadtan: public CameraPinhole {

public:
    CameraPinholeRadtan(): CameraPinhole() {}
    CameraPinholeRadtan(float fx, float fy, float cx, float cy): CameraPinhole(fx, fy, cx, cy) {}
    CameraPinholeRadtan(float fx, float fy, float cx, float cy, int32_t image_rows, int32_t image_cols):
        CameraPinhole(fx, fy, cx, cy, image_rows, image_cols) {}
    virtual ~CameraPinholeRadtan() = default;
    CameraPinholeRadtan(const CameraPinholeRadtan &pinhole) = delete;

public:
    virtual std::string CameraModelName() const override { return "Pinhole-Radtan"; }

    /*  Distortion model:
        x_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * x + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        y_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * y + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y */
    virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) override;
    virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) override;

    virtual void SetDistortionParameter(const std::vector<float> &params) override;
    virtual void GetDistortionParameter(std::vector<float> &params) const override;
    const float &k1() const { return k_[0]; }
    const float &k2() const { return k_[1]; }
    const float &k3() const { return k_[2]; }
    const float &p1() const { return p_[0]; }
    const float &p2() const { return p_[1]; }

private:
    // Different method to do undistortion.
    bool UndistortByGradienDesent(const Vec2 &distort_xy, Vec2 &undistort_xy);
    bool UndistortByFixePointIteration(const Vec2 &distort_xy, Vec2 &undistort_xy);

private:
    // Distortion model parameters.
    std::array<float, 3> k_ = {};
    std::array<float, 2> p_ = {};
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_PINHOLE_RADTAN_MODEL_H_
