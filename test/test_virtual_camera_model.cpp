#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "pinhole.h"
#include "fisheye.h"
#include "virtual_camera.h"
#include "slam_basic_math.h"
#include "visualizor_2d.h"

using namespace SLAM_VISUALIZOR;
using namespace SENSOR_MODEL;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test virtual camera model." RESET_COLOR);

    // Load parameters for fisheye camera.
    const std::string image_filepath = "../examples/fisheye_distorted.bmp";
    const float fx = 348.52f;
    const float fy = 348.52f;
    const float cx = 640.19f;
    const float cy = 358.56f;
    const float k1 = 0.066258f;
    const float k2 = 0.039769f;
    const float k3 = -0.026906f;
    const float k4 = 0.003342f;
    const float k5 = 0.0f;

    // Load image, allocate memory.
    GrayImage raw_image;
    Visualizor2D::LoadImage(image_filepath, raw_image);

    // Initialize virtual camera.
    VirtualCamera virtual_camera;
    virtual_camera.options().kVirtualCameraFocusLength = 200.0f;
    virtual_camera.options().kVirtualImageRows = 500;
    virtual_camera.options().kVirtualImageCols = 500;
    virtual_camera.raw_camera_model() = std::make_unique<Fisheye>(fx, fy, cx, cy);
    virtual_camera.raw_camera_model()->SetDistortionParameter(std::vector<float>{k1, k2, k3, k4, k5});

    // Generate image of virtual camera.
    if (virtual_camera.GenerateMaphex(Quat::Identity(), Quat(Eigen::AngleAxisf(-20.0f * kDegToRad, Vec3::UnitY())))) {
        ReportInfo("Succeed to generate maphex.");
    } else {
        ReportError("Failed to generate maphex.");
    }
    if (virtual_camera.RemapVirtualCameraImage(raw_image)) {
        ReportInfo("Succeed to remap virtual camera image.");
    } else {
        ReportError("Failed to remap virtual camera image.");
    }
    GrayImage virtual_image(virtual_camera.virtual_camera_image().data(),
                            virtual_camera.virtual_camera_image().rows(),
                            virtual_camera.virtual_camera_image().cols(),
                            false);
    GrayImage virtual_mask(virtual_camera.virtual_camera_mask().data(),
                           virtual_camera.virtual_camera_mask().rows(),
                           virtual_camera.virtual_camera_mask().cols(),
                           false);

    // Visualize result.
    Visualizor2D::ShowImage("raw image", raw_image);
    Visualizor2D::ShowImage("virtual image", virtual_image);
    Visualizor2D::ShowImage("virtual mask", virtual_mask);
    Visualizor2D::WaitKey(0);

    return 0;
}
