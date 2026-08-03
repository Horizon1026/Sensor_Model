#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

#include "fstream"
#include "iostream"

#include "lidar.h"
#include "lidar_measurement.h"

#include "visualizor_3d.h"

using namespace slam_utility;
using namespace sensor_model;
using namespace slam_visualizor;

void LoadLidarMeasurements(const std::string &file_name, std::vector<Vec3> &points) {
    std::ifstream imu_file(file_name.c_str());
    if (!imu_file.is_open()) {
        ReportError("Failed to load lidar data file " << file_name);
        return;
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
}

// Show a set of points in a 3D window with the camera auto-fitted to the cloud.
// If a window with the same title was already opened, it is reused: the first
// Refresh() call also clears the pending "should close" state of the previous
// window, so several datasets can be shown one after another in one window.
void VisualizePoints3D(const std::string &window_title, const std::vector<Vec3> &points, const RgbPixel &color, const int32_t radius) {
    if (points.empty()) {
        ReportWarn("No points to visualize in window [" << window_title << "].");
        return;
    }

    // Compute the bounding box of the point cloud and auto-fit the camera.
    Vec3 min_p = points[0];
    Vec3 max_p = points[0];
    for (const auto &point : points) {
        min_p = min_p.cwiseMin(point);
        max_p = max_p.cwiseMax(point);
    }
    const Vec3 center = 0.5f * (min_p + max_p);
    const float max_extent = (max_p - min_p).maxCoeff();
    const float view_depth = (max_extent > 1e-6f) ? (max_extent * 4.0f) : 1.0f;

    Visualizor3D::camera_view().q_wc = Quat::Identity();
    Visualizor3D::camera_view().p_wc = center - Vec3(0.0f, 0.0f, view_depth);

    Visualizor3D::Clear();
    for (const auto &point : points) {
        Visualizor3D::points().emplace_back(PointType {
            .p_w = point,
            .color = color,
            .radius = radius,
        });
    }

    ReportInfo(">> Show window [" << window_title << "]. Press ESC or close the window to continue.");
    // Refresh once first: for a reused window this also clears the "should
    // close" flag that was set when the window was closed in the previous round.
    Visualizor3D::Refresh(window_title, 30);
    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh(window_title, 30);
    }
}

int main(int argc, char **argv) {
    std::string lidar_scan_file = "../examples/lidar_scan.txt";
    std::string ascii_pcd_file = "../examples/ascii_pcd.pcd";
    if (argc >= 2) {
        lidar_scan_file = argv[1];
    }
    if (argc >= 3) {
        ascii_pcd_file = argv[2];
    }

    ReportInfo(YELLOW ">> Test lidar model." RESET_COLOR);

    // Load the lidar scan.
    std::vector<Vec3> lidar_points;
    // LoadLidarMeasurements(lidar_scan_file, lidar_points);

    // Parse the example ascii pcd file with the lidar model.
    Lidar lidar;
    std::vector<Vec3> pcd_points;
    const bool pcd_parsed = lidar.ConvertPcdFileToPoints(ascii_pcd_file, pcd_points);
    if (pcd_parsed) {
        ReportInfo(">> Parse pcd file " << ascii_pcd_file << " with " << pcd_points.size() << " points.");
    } else {
        ReportError(">> Failed to parse pcd file " << ascii_pcd_file);
    }

    // Show the lidar scan.
    VisualizePoints3D("Lidar model", lidar_points, RgbColor::kCyan, 1);

    // Show the parsed pcd points in 3D.
    if (pcd_parsed) {
        VisualizePoints3D("Lidar model", pcd_points, RgbColor::kGreen, 2);
    } else {
        ReportError(">> No pcd points to visualize.");
    }

    return 0;
}
