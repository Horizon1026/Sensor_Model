#include "lidar.h"

#include "basic_type.h"
#include "slam_log_reporter.h"
#include "visualizor_3d.h"

#include "cctype"
#include "fstream"
#include "iostream"

using namespace slam_utility;
using namespace sensor_model;
using namespace slam_visualizor;

bool LoadLidarMeasurements(const std::string &file_name, std::vector<Vec3> &points) {
    std::ifstream imu_file(file_name.c_str());
    if (!imu_file.is_open()) {
        ReportError("Failed to load lidar data file " << file_name);
        return false;
    }

    ReportInfo(">> Load lidar data from " << file_name);
    points.clear();
    points.reserve(30000);

    std::string oneLine;
    Vec3 position = Vec3::Zero();
    while (std::getline(imu_file, oneLine) && !oneLine.empty()) {
        std::istringstream imuData(oneLine);
        imuData >> position.x() >> position.y() >> position.z();
        points.emplace_back(position);
    }

    return !points.empty();
}

// Show a set of points in a 3D window with the camera auto-fitted to the cloud.
void VisualizePoints3D(const std::string &window_title, const std::vector<Vec3> &points, const RgbPixel &color, const int32_t radius) {
    if (points.empty()) {
        ReportWarn("No points to visualize in window [" << window_title << "].");
        return;
    }

    // Compute the bounding box of the point cloud and auto-fit the camera.
    Vec3 min_p = points[0];
    Vec3 max_p = points[0];
    for (const auto &point: points) {
        min_p = min_p.cwiseMin(point);
        max_p = max_p.cwiseMax(point);
    }
    const Vec3 center = 0.5f * (min_p + max_p);
    const float max_extent = (max_p - min_p).maxCoeff();
    const float view_depth = (max_extent > 1e-6f) ? (max_extent * 4.0f) : 1.0f;

    Visualizor3D::camera_view().q_wc = Quat::Identity();
    Visualizor3D::camera_view().p_wc = center - Vec3(0.0f, 0.0f, view_depth);

    Visualizor3D::Clear();
    for (const auto &point: points) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = point,
            .color = color,
            .radius = radius,
        });
    }

    ReportInfo(">> Show window [" << window_title << "]. Press ESC or close the window to continue.");
    Visualizor3D::Refresh(window_title, 30);
    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh(window_title, 30);
    }
}

int main(int argc, char **argv) {
    // A single input file: a .txt lidar scan (x y z per line) or a .pcd point cloud.
    if (argc != 2) {
        ReportInfo(">> Usage: " << argv[0] << " <point_file>");
        ReportInfo("   point_file is a .txt lidar scan or a .pcd point cloud, e.g. ../examples/ascii_pcd.pcd.");
        return 0;
    }
    const std::string point_file = argv[1];

    ReportInfo(YELLOW ">> Test lidar model." RESET_COLOR);

    // Choose the loader according to the file extension.
    std::vector<Vec3> points;
    Lidar lidar;
    bool parsed = false;
    std::string ext = point_file.substr(point_file.find_last_of('.') + 1);
    for (auto &c: ext) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (ext == "txt") {
        parsed = LoadLidarMeasurements(point_file, points);
    } else if (ext == "pcd") {
        parsed = lidar.ConvertPcdFileToPoints(point_file, points);
    } else {
        ReportError("Unsupported file extension [" << ext << "], expected a .txt or .pcd file.");
        return 0;
    }

    if (!parsed || points.empty()) {
        ReportError(">> Failed to parse point file " << point_file);
        return 0;
    }

    ReportInfo(">> Parse point file " << point_file << " with " << points.size() << " points.");

    // Show the parsed points in 3D.
    VisualizePoints3D("Lidar model", points, RgbColor::kGreen, 2);

    return 0;
}
