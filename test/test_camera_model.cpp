#include "log_api.h"
#include "pinhole.h"

void test_pinhole_camera_model() {
	LogInfo(YELLOW ">> Test pinhole camera model." RESET_COLOR);

    SensorModel::Pinhole camera;
    camera.SetMatrixK(148, 148, 120, 120);
    LogInfo("Set camera params with " << camera.fx() << ", " << camera.fy()
    	<< ", " << camera.cx() << ", " << camera.cy());
}

int main() {
    LogInfo(YELLOW ">> Test camera model." RESET_COLOR);

	test_pinhole_camera_model();
    return 0;
}