#include "basic_type.h"
#include "slam_operations.h"
#include "slam_log_reporter.h"

#include <fstream>
#include <iostream>

#include "lidar.h"
#include "lidar_measurement.h"

#include "visualizor_3d.h"

using namespace SLAM_UTILITY;
using namespace SENSOR_MODEL;
using namespace SLAM_VISUALIZOR;

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

int main(int argc, char **argv) {
    std::string lidar_scan_file = "../examples/lidar_scan.txt";
    if (argc == 2) {
        lidar_scan_file = argv[1];
    }

    ReportInfo(YELLOW ">> Test lidar model." RESET_COLOR);
    std::vector<Vec3> lidar_points;
    LoadLidarMeasurements(lidar_scan_file, lidar_points);

    Visualizor3D::camera_view().p_wc = Vec3(0, 0, -40);
    Visualizor3D::Clear();
    for (const auto &point : lidar_points) {
        Visualizor3D::points().emplace_back(PointType{
            .p_w = point.cast<float>(),
            .color = RgbColor::kCyan,
            .radius = 1,
        });
    }
    while (!Visualizor3D::ShouldQuit()) {
        Visualizor3D::Refresh("Lidar scan", 30);
    }

    return 0;
}
