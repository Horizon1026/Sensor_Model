#ifndef _SENSOR_MODEL_FISHEYE_CAMERA_MODEL_H_
#define _SENSOR_MODEL_FISHEYE_CAMERA_MODEL_H_

#include "camera_basic.h"

namespace SensorModel {

class Fisheye : public CameraBasic {

public:
	Fisheye() : CameraBasic() {}
    Fisheye(float fx, float fy, float cx, float cy) : CameraBasic(fx, fy, cx, cy) {}
    virtual ~Fisheye() = default;
    Fisheye(const Fisheye &fisheye) = delete;

public:
    // Lift 3d point in camera frame on normalized plane.
    virtual void LiftToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) override;

    // Lift 3d point in camera frame on normalized plane, and do undistortion.
    virtual bool LiftToNormalizedPlaneAndUndistort(const Vec2 pixel_uv, Vec2 &undistort_xy) override;

    /*
        Distortion model:
        r(theta) = k0 * theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9 + ...
    */
	virtual bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy) override;
	virtual bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy) override;

    void SetDistortionParameter(float k1, float k2, float k3, float k4, float k5);
    const float &k1() const { return k_[0]; }
    const float &k2() const { return k_[1]; }
    const float &k3() const { return k_[2]; }
    const float &k4() const { return k_[3]; }
    const float &k5() const { return k_[4]; }

private:
    // Different method to do undistortion.
    bool UndistortByGradienDesent(const Vec2 &distort_xy, Vec2 &undistort_xy);
    bool UndistortByFixePointIteration(const Vec2 &distort_xy, Vec2 &undistort_xy);

private:
    // Distortion model parameters.
	std::array<float, 5> k_ = {0, 0, 0, 0, 0};

};

}

#endif // end of _SENSOR_MODEL_FISHEYE_CAMERA_MODEL_H_