#ifndef _SENSOR_MODEL_PINHOLE_CAMERA_MODEL_H_
#define _SENSOR_MODEL_PINHOLE_CAMERA_MODEL_H_

#include "camera_basic.h"

namespace SensorModel {

class Pinhole : public CameraBasic {

public:
	Pinhole() : CameraBasic() {}
    virtual ~Pinhole() = default;
    Pinhole(const Pinhole &pinhole) = delete;

public:
    // Lift 3d point in camera frame on normalized plane.
    virtual void LiftToNormalizedPlane(const Vec3 p_c, Vec2 &norm_xy) override;

    // Lift 2d point in normalized plane back on camera frame.
    virtual void LiftBackToCameraFrame(const Vec2 norm_xy, Vec3 &p_c) override;

private:
	void Distort(const Vec2 undistort, Vec2 &distort);
	void Undistort(const Vec2 distort, Vec2 &undistort);

private:
	std::array<float, 3> k = {};
    std::array<float, 2> p = {};

};

}

#endif
