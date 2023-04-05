#ifndef _SENSOR_MODEL_PINHOLE_CAMERA_MODEL_H_
#define _SENSOR_MODEL_PINHOLE_CAMERA_MODEL_H_

#include "camera_basic.h"
#include "datatype_image.h"

namespace SensorModel {

class Pinhole : public CameraBasic {

public:
	Pinhole() : CameraBasic() {}
    Pinhole(float fx, float fy, float cx, float cy) : CameraBasic(fx, fy, cx, cy) {}
    virtual ~Pinhole() = default;
    Pinhole(const Pinhole &pinhole) = delete;

public:
    // Lift 3d point in camera frame on normalized plane.
    virtual void LiftToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) override;

    /*
        Distortion model:
        x_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * x + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        y_distort = (1 + k1 * r2 + k2 * r4 + k3 * r6) * y + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    */
	bool DistortOnImagePlane(const Vec2 undistort_uv, Vec2 &distort_uv);
	bool UndistortOnImagePlane(const Vec2 distort_uv, Vec2 &undistort_uv);
	bool DistortOnNormalizedPlane(const Vec2 undistort_xy, Vec2 &distort_xy);
	bool UndistortOnNormalizedPlane(const Vec2 distort_xy, Vec2 &undistort_xy);

    void SetDistortionParameter(const float k1, const float k2, const float k3, const float p1, const float p2);

    const float &k1() const { return k_[0]; }
    const float &k2() const { return k_[1]; }
    const float &k3() const { return k_[2]; }
    const float &p1() const { return p_[0]; }
    const float &p2() const { return p_[1]; }

    bool CorrectDistortedImage(const Image &raw_image, Image &corrected_image);

private:
    bool UndistortByGradienDesent(const Vec2 &distort_xy, Vec2 &undistort_xy);
    bool UndistortByFixePointIteration(const Vec2 &distort_xy, Vec2 &undistort_xy);

private:
    // Distortion model parameters.
	std::array<float, 3> k_ = {0, 0, 0};
    std::array<float, 2> p_ = {0, 0};

};

}

#endif