#include "basic_type.h"
#include "slam_log_reporter.h"

#include "pinhole.h"
#include "fisheye.h"

#include "visualizor_2d.h"

#include "feature_point_detector.h"
#include "feature_harris.h"

using namespace SLAM_VISUALIZOR;
using namespace FEATURE_DETECTOR;

void DetectFeaturesInRawImage(const GrayImage &image, std::vector<Vec2> &pixel_uv) {
    // Detect features.
    FeaturePointDetector<HarrisFeature> detector;
    detector.options().kMinFeatureDistance = 25;
    detector.feature().options().kHalfPatchSize = 1;
    detector.feature().options().kMinValidResponse = 40.0f;
    detector.DetectGoodFeatures(image, 20, pixel_uv);
}

void TestPinholeCameraModel() {
    ReportInfo(YELLOW ">> Test pinhole camera model undistortion." RESET_COLOR);

    // Load parameters for pinhole camera.
    const std::string image_filepath = "../examples/pinhole_distorted.png";
    const float fx = 458.654f;
    const float fy = 457.296f;
    const float cx = 752.0f / 2.0f;
    const float cy = 240.0f;
    const float k1 = -0.28340811f;
    const float k2 = 0.07395907f;
    const float k3 = 0.0f;
    const float p1 = 0.00019359f;
    const float p2 = 1.76187114e-05f;

    // Load image, allocate memory.
    GrayImage raw_image;
    GrayImage corr_image;
    Visualizor::LoadImage(image_filepath, raw_image);
    Visualizor::LoadImage(image_filepath, corr_image);
    std::vector<Vec2> distort_features, undistort_features;
    DetectFeaturesInRawImage(raw_image, distort_features);
    undistort_features.reserve(distort_features.size());

    // Initialize pinhole camera.
    SENSOR_MODEL::Pinhole camera;
    camera.SetIntrinsicParameter(fx, fy, cx, cy);
    camera.SetDistortionParameter(std::vector<float>{k1, k2, k3, p1, p2});

    // Undistort the whole image.
    camera.CorrectDistortedImage(raw_image, corr_image);

    // Step 1: use raw image to detect distorted features.
    // Step 2: use ceoff to undistort features.
    // Step 3: use distortion model to distort the result from step 2.
    float average_residual = 0.0f;
    Vec2 undistort = Vec2::Zero();
    Vec2 distort = Vec2::Zero();
    for (uint32_t i = 0; i < distort_features.size(); ++i) {
        const Vec2 &raw_distort = distort_features[i];

        if (camera.UndistortOnImagePlane(raw_distort, undistort)) {
            camera.DistortOnImagePlane(undistort, distort);
            average_residual += (raw_distort - distort).norm();
            undistort_features.emplace_back(undistort);
        } else {
            ReportError("camera.UndistortOnImagePlane(raw_distort, undistort) failed.");
        }
    }
    average_residual /= static_cast<float>(distort_features.size());
    ReportInfo("   Undistortion average residual is " << average_residual);

    // Show undistortion image.
    Visualizor::ShowImageWithDetectedFeatures("Pinhole distorted image with detected features",
        raw_image, distort_features);
    Visualizor::ShowImageWithDetectedFeatures("Pinhole undistorted image with detected features",
        corr_image, undistort_features);
    Visualizor::WaitKey(1);
}

void TestFisheyeCameraModel() {
    ReportInfo(YELLOW ">> Test fisheye camera model undistortion." RESET_COLOR);

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
    GrayImage corr_image;
    Visualizor::LoadImage(image_filepath, raw_image);
    Visualizor::LoadImage(image_filepath, corr_image);
    std::vector<Vec2> distort_features, undistort_features;
    DetectFeaturesInRawImage(raw_image, distort_features);
    undistort_features.reserve(distort_features.size());

    // Initialize fisheye camera.
    SENSOR_MODEL::Fisheye camera;
    camera.SetIntrinsicParameter(fx, fy, cx, cy);
    camera.SetDistortionParameter(std::vector<float>{k1, k2, k3, k4, k5});

    // Undistort the whole image.
    camera.CorrectDistortedImage(raw_image, corr_image);

    // Step 1: use raw image to detect distorted features.
    // Step 2: use ceoff to undistort features.
    // Step 3: use distortion model to distort the result from step 2.
    float average_residual = 0.0f;
    Vec2 undistort = Vec2::Zero();
    Vec2 distort = Vec2::Zero();
    for (uint32_t i = 0; i < distort_features.size(); ++i) {
        const Vec2 &raw_distort = distort_features[i];

        if (camera.UndistortOnImagePlane(raw_distort, undistort)) {
            camera.DistortOnImagePlane(undistort, distort);
            average_residual += (raw_distort - distort).norm();
            undistort_features.emplace_back(undistort);
        } else {
            ReportError("camera.UndistortOnImagePlane(raw_distort, undistort) failed.");
        }
    }
    average_residual /= static_cast<float>(distort_features.size());
    ReportInfo("   Undistortion average residual is " << average_residual);

    // Show undistortion image.
    Visualizor::ShowImageWithDetectedFeatures("Fisheye distorted image with detected features",
        raw_image, distort_features);
    Visualizor::ShowImageWithDetectedFeatures("Fisheye undistorted image with detected features",
        corr_image, undistort_features);
    Visualizor::WaitKey(1);
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test camera model." RESET_COLOR);

    TestPinholeCameraModel();
    TestFisheyeCameraModel();

    Visualizor::WaitKey(0);

    return 0;
}
