#include "log_api.h"
#include "pinhole.h"
#include "fisheye.h"

#include "opencv2/opencv.hpp"

void DetectFeaturesInRawImage(const cv::Mat &cv_raw_image, std::vector<cv::Point2f> &distort_features) {
    cv::goodFeaturesToTrack(cv_raw_image, distort_features, 200, 0.01, 20);

    cv::Mat show_distorted(cv_raw_image.rows, cv_raw_image.cols, CV_8UC3);
    cv::cvtColor(cv_raw_image, show_distorted, cv::COLOR_GRAY2BGR);
    for (uint32_t i = 0; i < distort_features.size(); ++i) {
        cv::circle(show_distorted, distort_features[i], 2, cv::Scalar(255, 255, 0), 3);
    }

    cv::imshow("Distorted image with detected features", show_distorted);
}

void TestPinholeCameraModel() {
	LogInfo(YELLOW ">> Test pinhole camera model undistortion." RESET_COLOR);

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
    cv::Mat cv_raw_image = cv::imread(image_filepath, 0);
    cv::Mat cv_corr_image = cv::imread(image_filepath, 0);
    std::vector<cv::Point2f> distort_features, undistort_features;
    DetectFeaturesInRawImage(cv_raw_image, distort_features);
    undistort_features.reserve(distort_features.size());

    // Initialize pinhole camera.
    SENSOR_MODEL::Pinhole camera;
    camera.SetIntrinsicParameter(fx, fy, cx, cy);
    camera.SetDistortionParameter(Vec5(k1, k2, k3, p1, p2));

    // Undistort the whole image.
    Image raw_image, corr_image;
    raw_image.SetImage(cv_raw_image.data, cv_raw_image.rows, cv_raw_image.cols);
    corr_image.SetImage(cv_corr_image.data, cv_corr_image.rows, cv_corr_image.cols);
    camera.CorrectDistortedImage(raw_image, corr_image);

    // Step 1: use raw image to detect distorted features.
    // Step 2: use ceoff to undistort features.
    // Step 3: use distortion model to distort the result from step 2.
    float average_residual = 0.0f;
    Vec2 undistort = Vec2::Zero();
    Vec2 distort = Vec2::Zero();
    for (uint32_t i = 0; i < distort_features.size(); ++i) {
        Vec2 raw_distort = Vec2(distort_features[i].x, distort_features[i].y);

        if (camera.UndistortOnImagePlane(raw_distort, undistort)) {
            camera.DistortOnImagePlane(undistort, distort);
            average_residual += (raw_distort - distort).norm();
            undistort_features.emplace_back(cv::Point2f(undistort.x(), undistort.y()));
        } else {
            LogError("camera.UndistortOnImagePlane(raw_distort, undistort) failed.");
        }
    }
    average_residual /= static_cast<float>(distort_features.size());
    LogInfo("   Undistortion average residual is " << average_residual);

    // Show undistortion image.
    cv::Mat show_undistorted(cv_corr_image.rows, cv_corr_image.cols, CV_8UC3);
    cv::cvtColor(cv_corr_image, show_undistorted, cv::COLOR_GRAY2BGR);
    for (uint32_t i = 0; i < undistort_features.size(); ++i) {
        cv::circle(show_undistorted, undistort_features[i], 2, cv::Scalar(255, 255, 0), 3);
    }
    cv::imshow("Pinhole undistorted image with detected features", show_undistorted);

    // Draw undistortion result with extra size.
    camera.CorrectDistortedImage(raw_image, corr_image, 1.4f);
    cv::imshow("Pinhole undistorted image", cv_corr_image);
    cv::waitKey();
}

void TestFisheyeCameraModel() {
	LogInfo(YELLOW ">> Test fisheye camera model undistortion." RESET_COLOR);

    // Load parameters for fisheye camera.
    const std::string image_filepath = "../examples/fisheye_distorted.png";
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
    cv::Mat cv_raw_image = cv::imread(image_filepath, 0);
    cv::Mat cv_corr_image = cv::imread(image_filepath, 0);
    std::vector<cv::Point2f> distort_features, undistort_features;
    DetectFeaturesInRawImage(cv_raw_image, distort_features);
    undistort_features.reserve(distort_features.size());

    // Initialize fisheye camera.
    SENSOR_MODEL::Fisheye camera;
    camera.SetIntrinsicParameter(fx, fy, cx, cy);
    camera.SetDistortionParameter(Vec5(k1, k2, k3, k4, k5));

    // Undistort the whole image.
    Image raw_image, corr_image;
    raw_image.SetImage(cv_raw_image.data, cv_raw_image.rows, cv_raw_image.cols);
    corr_image.SetImage(cv_corr_image.data, cv_corr_image.rows, cv_corr_image.cols);
    camera.CorrectDistortedImage(raw_image, corr_image);

    // Step 1: use raw image to detect distorted features.
    // Step 2: use ceoff to undistort features.
    // Step 3: use distortion model to distort the result from step 2.
    float average_residual = 0.0f;
    Vec2 undistort = Vec2::Zero();
    Vec2 distort = Vec2::Zero();
    for (uint32_t i = 0; i < distort_features.size(); ++i) {
        Vec2 raw_distort = Vec2(distort_features[i].x, distort_features[i].y);

        if (camera.UndistortOnImagePlane(raw_distort, undistort)) {
            camera.DistortOnImagePlane(undistort, distort);
            average_residual += (raw_distort - distort).norm();
            undistort_features.emplace_back(cv::Point2f(undistort.x(), undistort.y()));
        } else {
            LogError("camera.UndistortOnImagePlane(raw_distort, undistort) failed.");
        }
    }
    average_residual /= static_cast<float>(distort_features.size());
    LogInfo("   Undistortion average residual is " << average_residual);

    // Show undistortion image.
    cv::Mat show_undistorted(cv_corr_image.rows, cv_corr_image.cols, CV_8UC3);
    cv::cvtColor(cv_corr_image, show_undistorted, cv::COLOR_GRAY2BGR);
    for (uint32_t i = 0; i < undistort_features.size(); ++i) {
        cv::circle(show_undistorted, undistort_features[i], 2, cv::Scalar(255, 255, 0), 3);
    }
    cv::imshow("Fisheye undistorted image with detected features", show_undistorted);

    // Draw undistortion result with extra size.
    camera.CorrectDistortedImage(raw_image, corr_image, 3.0f);
    cv::imshow("Fisheye undistorted image", cv_corr_image);
    cv::waitKey();
}

int main(int argc, char **argv) {
    LogInfo(YELLOW ">> Test camera model." RESET_COLOR);

	TestPinholeCameraModel();
    TestFisheyeCameraModel();
    return 0;
}