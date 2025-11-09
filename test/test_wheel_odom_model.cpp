#include "basic_type.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

#include <fstream>
#include <iostream>

#include "wheel_odom.h"

using namespace SLAM_UTILITY;
using namespace SENSOR_MODEL;

void LoadWheelOdomMeasurements(const std::string &file_name, std::vector<Vec3> &velocities) {
    std::ifstream file(file_name.c_str());
    if (!file.is_open()) {
        ReportError("Failed to load data file " << file_name);
        return;
    }

    WheelOdom odom_model;
    odom_model.options().kEncoderCountInOneCircle = 1024.0f;
    odom_model.options().kWheelRadiusInMeter = 0.155f;
    odom_model.options().kEncoderSamplePeriodInSecond = 0.1f;
    odom_model.options().kVelocityNoiseSigma = 0.5f;

    // Publish each line of data file.
    std::string one_line;
    std::string type;
    while (std::getline(file, one_line) && !one_line.empty()) {
        std::istringstream data(one_line);

        WheelOdomMeasurement odom;
        data >> type >> odom.time_stamp_s >> odom.encoder_left_cnt >> odom.encoder_right_cnt;
        odom.is_wheel_velocity_valid = true;

        const Vec3 velocity = odom_model.ConvertEncoderCountToVelocity(odom);
        velocities.emplace_back(velocity);
        ReportInfo("velocity " << LogVec(velocity) << " m/s.");
    }

    file.close();
}

int main(int argc, char **argv) {
    std::string odom_file = "../examples/odom.txt";
    if (argc == 2) {
        odom_file = argv[1];
    }

    ReportInfo(YELLOW ">> Test wheel odom model." RESET_COLOR);

    std::vector<Vec3> velocities;
    LoadWheelOdomMeasurements(odom_file, velocities);

    return 0;
}
