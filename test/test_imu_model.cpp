#include "log_api.h"
#include "imu_state.h"
#include "imu.h"
#include "imu_preintegrate.h"

#include <fstream>

using namespace SENSOR_MODEL;

bool LoadImuMeasurements(const std::string &imu_file,
                         std::vector<ImuMeasurement> &measurements,
                         std::vector<Vec3> &position,
                         std::vector<Quat> &rotation) {
    LogInfo(">> Load imu data from " << imu_file);

    measurements.clear();
    position.clear();
    rotation.clear();

    std::ifstream fsIMU;
    fsIMU.open(imu_file.c_str());
    if (!fsIMU.is_open()) {
        std::cout << "   failed." << std::endl;
        return false;
    }

    std::string oneLine;
    double time_stamp;
    TVec3<double> acc, gyr, pos;
    TQuat<double> q;
    uint32_t cnt = 0;
    while (std::getline(fsIMU, oneLine) && !oneLine.empty()) {
        std::istringstream imuData(oneLine);
        imuData >> time_stamp >> q.w() >> q.x() >> q.y() >> q.z() >> pos.x() >> pos.y() >> pos.z()
			>> gyr.x() >> gyr.y() >> gyr.z() >> acc.x() >> acc.y() >> acc.z();

        ImuMeasurement meas;
        meas.accel = acc.cast<float>();
        meas.gyro = gyr.cast<float>();
        meas.time_stamp = time_stamp;
        measurements.emplace_back(meas);
        position.emplace_back(pos.cast<float>());
        rotation.emplace_back(q.cast<float>());

        ++cnt;
        if (cnt > 20) {
            break;
        }
    }

    LogInfo(GREEN << cnt << RESET_COLOR " imu raw data loaded.");
    return true;
}

void TestImuPreintegration(std::vector<ImuMeasurement> &measurements,
                           std::vector<Vec3> &position,
                           std::vector<Quat> &rotation) {
    // Preintegrate all imu measurements.
    ImuPreintegrateBlock block;
    const int32_t meas_size = measurements.size();
    for (int32_t i = 2; i < meas_size - 1; ++i) {
        block.Propagate(measurements[i - 1], measurements[i]);
    }

    // Compute residual.
    const float dt = measurements[meas_size - 2].time_stamp - measurements[1].time_stamp;
    const Mat3 R_wi_i = rotation[1].matrix();
    const Quat q_wi_i = rotation[1];
    const Quat q_wi_j = rotation[meas_size - 2];
    const Vec3 p_wi_i = position[1];
    const Vec3 p_wi_j = position[meas_size - 2];
    const Vec3 v_wi_i = (position[2] - position[0]) / (measurements[2].time_stamp - measurements[0].time_stamp);
    const Vec3 v_wi_j = (position[meas_size - 1] - position[meas_size - 3]) / (measurements[meas_size - 1].time_stamp - measurements[meas_size - 3].time_stamp);
    const Vec3 g_w = Vec3(0, 0, 9.8);

    Vec9 residual = Vec9::Zero();
    residual.segment<3>(0) = R_wi_i.transpose() * (p_wi_j - p_wi_i - v_wi_i * dt + 0.5f * g_w * dt * dt) - block.p_ij();
    residual.segment<3>(3) = 2.0f * (block.q_ij().inverse() * (q_wi_i.inverse() * q_wi_j)).vec();
    residual.segment<3>(6) = R_wi_i.transpose() * (v_wi_j - v_wi_i + g_w * dt) - block.v_ij();

    LogInfo("Residual of imu preintegration is " << LogVec(residual));

}

void TestImuIntegration() {

}

int main(int argc, char **argv) {
    std::string imu_file;
    if (argc == 2) {
        imu_file = argv[1];
    }

    LogInfo(YELLOW ">> Test imu model." RESET_COLOR);

    std::vector<ImuMeasurement> measurements;
    std::vector<Vec3> position;
    std::vector<Quat> rotation;
    if (LoadImuMeasurements(imu_file, measurements, position, rotation) == false) {
        return 0;
    }

    TestImuPreintegration(measurements, position, rotation);

    return 0;
}
