#ifndef _SENSOR_MODEL_CAMERA_PINHOLE_KANNALA_BRANDT_MODEL_H_
#define _SENSOR_MODEL_CAMERA_PINHOLE_KANNALA_BRANDT_MODEL_H_

#include "camera_pinhole.h"

namespace sensor_model {

/* Class CameraPinholeKannalaBrandt Declaration. */
class CameraPinholeKannalaBrandt: public CameraPinhole {

public:
    CameraPinholeKannalaBrandt(): CameraPinhole() {}
    CameraPinholeKannalaBrandt(float fx, float fy, float cx, float cy): CameraPinhole(fx, fy, cx, cy) {}
    CameraPinholeKannalaBrandt(float fx, float fy, float cx, float cy, int32_t image_rows, int32_t image_cols):
        CameraPinhole(fx, fy, cx, cy, image_rows, image_cols) {}
    virtual ~CameraPinholeKannalaBrandt() = default;
    CameraPinholeKannalaBrandt(const CameraPinholeKannalaBrandt &pinhole) = delete;

public:
    virtual std::string CameraModelName() const override { return "Pinhole-KannalaBrandt"; }

    /*  Kannala-Brandt model.
        r(theta) = theta * k1 + theta^2 * k2 + theta^3 * k3 + theta^4 * k4 + theta^5 * k5 + ... */
    virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) const override;
    virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) const override;

    virtual void SetDistortionParameter(const std::vector<float> &params) override;
    virtual void GetDistortionParameter(std::vector<float> &params) const override;
    const float &k1() const { return k_[0]; }
    const float &k2() const { return k_[1]; }
    const float &k3() const { return k_[2]; }
    const float &k4() const { return k_[3]; }
    const float &k5() const { return k_[4]; }
    const float &k6() const { return k_[5]; }

private:
    // Different method to do undistortion.
    bool UndistortByFixePointIteration(const Vec2 &distort_xy, Vec2 &undistort_xy) const;

private:
    // Distortion model parameters.
    std::array<float, 6> k_ = {};
};

}  // namespace sensor_model

#endif  // end of _SENSOR_MODEL_CAMERA_PINHOLE_KANNALA_BRANDT_MODEL_H_
