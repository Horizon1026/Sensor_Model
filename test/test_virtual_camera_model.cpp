#include "basic_type.h"
#include "camera_model.h"
#include "image_painter.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"
#include "virtual_camera.h"
#include "visualizor_2d.h"

#include "enable_stack_backward.h"

using namespace slam_visualizor;
using namespace sensor_model;
using namespace image_painter;

void TestVirtualCameraModel(const std::string &test_name, const GrayImage &raw_image, VirtualCamera &virtual_camera) {
    // Generate image of virtual camera.
    if (virtual_camera.GenerateMaphex(Quat(Eigen::AngleAxisf(-30.0f * kDegToRad, Vec3::UnitY())))) {
        ReportInfo("Succeed to generate maphex.");
    } else {
        ReportError("Failed to generate maphex.");
    }

    // Report parameters of virtual camera.
    ReportInfo("Image rows is " << virtual_camera.GetVirtualCameraImageRows());
    ReportInfo("Image cols is " << virtual_camera.GetVirtualCameraImageCols());
    ReportInfo("Fov(H * V) is " << LogVec(virtual_camera.GetVirtualCameraFov()) << " deg.");

    // Remap virtual camera image.
    if (virtual_camera.RemapVirtualCameraImage(raw_image)) {
        ReportInfo("Succeed to remap virtual camera image.");
    } else {
        ReportError("Failed to remap virtual camera image.");
    }
    GrayImage virtual_image(virtual_camera.virtual_camera_image().data(), virtual_camera.virtual_camera_image().rows(),
                            virtual_camera.virtual_camera_image().cols(), false);
    GrayImage virtual_mask(virtual_camera.virtual_camera_mask().data(), virtual_camera.virtual_camera_mask().rows(),
                           virtual_camera.virtual_camera_mask().cols(), false);

    // Draw range of virtual image in raw image.
    MatImg show_image_mat = MatImg(raw_image.rows(), raw_image.cols());
    raw_image.ToMatImg(show_image_mat);
    GrayImage show_image(show_image_mat.data(), show_image_mat.rows(), show_image_mat.cols(), false);
    uint8_t mark_value = 0;
    Vec2 pixel_uv = Vec2::Zero();
    for (int32_t row = 0; row < virtual_camera.GetVirtualCameraImageRows(); ++row) {
        virtual_camera.RemapPixelUvFromVirtualCameraToRawCamera(Vec2(0, row), pixel_uv);
        ImagePainter::DrawSolidCircle(show_image, pixel_uv.x(), pixel_uv.y(), 2, mark_value);
        virtual_camera.RemapPixelUvFromVirtualCameraToRawCamera(Vec2(virtual_camera.GetVirtualCameraImageCols(), row), pixel_uv);
        ImagePainter::DrawSolidCircle(show_image, pixel_uv.x(), pixel_uv.y(), 2, mark_value);
    }
    for (int32_t col = 0; col < virtual_camera.GetVirtualCameraImageCols(); ++col) {
        virtual_camera.RemapPixelUvFromVirtualCameraToRawCamera(Vec2(col, 0), pixel_uv);
        ImagePainter::DrawSolidCircle(show_image, pixel_uv.x(), pixel_uv.y(), 2, mark_value);
        virtual_camera.RemapPixelUvFromVirtualCameraToRawCamera(Vec2(col, virtual_camera.GetVirtualCameraImageRows()), pixel_uv);
        ImagePainter::DrawSolidCircle(show_image, pixel_uv.x(), pixel_uv.y(), 2, mark_value);
    }

    // Visualize result.
    Visualizor2D::ShowImage(std::string(test_name) + " - virtual image", virtual_image);
    Visualizor2D::ShowImage(std::string(test_name) + " - virtual mask", virtual_mask);
    Visualizor2D::ShowImage(std::string(test_name) + " - virtual image in raw image", show_image);
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test virtual camera model." RESET_COLOR);

    // Load parameters for fisheye camera.
    const std::string image_filepath = "../examples/fisheye_distorted.bmp";

    // Load image, allocate memory.
    GrayImage raw_image;
    Visualizor2D::LoadImage(image_filepath, raw_image);
    Visualizor2D::ShowImage("raw image", raw_image);

    VirtualCamera virtual_camera;

    // Initialize virtual camera to be default pinhole model with no distortion.
    const float cam1_fx = 348.52f;
    const float cam1_fy = 348.52f;
    const float cam1_cx = 640.19f;
    const float cam1_cy = 358.56f;
    const float cam1_k1 = 0.066258f;
    const float cam1_k2 = 0.039769f;
    const float cam1_k3 = -0.026906f;
    const float cam1_k4 = 0.003342f;
    const float cam1_k5 = 0.0f;
    virtual_camera.real_camera_model() = std::make_unique<CameraPinholeEquidistant>(cam1_fx, cam1_fy, cam1_cx, cam1_cy);
    virtual_camera.real_camera_model()->SetDistortionParameter(std::vector<float> {cam1_k1, cam1_k2, cam1_k3, cam1_k4, cam1_k5});
    TestVirtualCameraModel("pinhole-equidistant to virtual rectify", raw_image, virtual_camera);

    // Initialize virtual camera to be pinhole-equidistant camera model.
    virtual_camera.virtual_camera_model() = std::make_unique<CameraPinholeEquidistant>(cam1_fx, cam1_fy, cam1_cx, cam1_cy);
    virtual_camera.virtual_camera_model()->SetDistortionParameter(std::vector<float> {cam1_k1, cam1_k2, cam1_k3, cam1_k4, cam1_k5});
    TestVirtualCameraModel("pinhole-equidistant to pinhole-equidistant", raw_image, virtual_camera);

    // Initialize virtual camera to be pinhole-equidistant camera model.
    const float cam2_fx = 458.654f;
    const float cam2_fy = 457.296f;
    const float cam2_cx = 752.0f / 2.0f;
    const float cam2_cy = 240.0f;
    const float cam2_k1 = -0.28340811f;
    const float cam2_k2 = 0.07395907f;
    const float cam2_k3 = 0.0f;
    const float cam2_p1 = 0.00019359f;
    const float cam2_p2 = 1.76187114e-05f;
    virtual_camera.virtual_camera_model() = std::make_unique<CameraPinholeRadtan>(cam2_fx, cam2_fy, cam2_cx, cam2_cy);
    virtual_camera.virtual_camera_model()->SetDistortionParameter(std::vector<float> {cam2_k1, cam2_k2, cam2_k3, cam2_p1, cam2_p2});
    TestVirtualCameraModel("pinhole-equidistant to pinhole-radtan", raw_image, virtual_camera);

    Visualizor2D::WaitKey(0);
    return 0;
}
