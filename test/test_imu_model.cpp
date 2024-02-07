#include "log_report.h"
#include "imu.h"
#include "imu_state.h"
#include "imu_measurement.h"
#include "imu_preintegrate.h"

#include "imu_preintegration.hpp"

#include <fstream>

using namespace SENSOR_MODEL;

bool LoadImuMeasurements(const std::string &imu_file,
                         std::vector<ImuMeasurement> &measurements,
                         std::vector<Vec3> &position,
                         std::vector<Quat> &rotation) {
    ReportInfo(">> Load imu data from " << imu_file);

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
    double time_stamp_s;
    TVec3<double> acc, gyr, pos;
    TQuat<double> q;
    uint32_t cnt = 0;
    while (std::getline(fsIMU, oneLine) && !oneLine.empty()) {
        std::istringstream imuData(oneLine);
        imuData >> time_stamp_s >> q.w() >> q.x() >> q.y() >> q.z() >> pos.x() >> pos.y() >> pos.z()
            >> gyr.x() >> gyr.y() >> gyr.z() >> acc.x() >> acc.y() >> acc.z();

        ImuMeasurement meas;
        meas.accel = acc.cast<float>();
        meas.gyro = gyr.cast<float>();
        meas.time_stamp_s = time_stamp_s;
        measurements.emplace_back(meas);
        position.emplace_back(pos.cast<float>());
        rotation.emplace_back(q.cast<float>());

        ++cnt;
    }

    ReportInfo(GREEN << cnt << RESET_COLOR " imu raw data loaded.");
    return true;
}

void TestImuPreintegration(std::vector<ImuMeasurement> &measurements,
                           std::vector<Vec3> &position,
                           std::vector<Quat> &rotation) {
    ReportInfo(YELLOW ">> Test imu preintegration." RESET_COLOR);

    // Define range.
    const int32_t start_index = 1;
    const int32_t end_index = measurements.size() - 2;

    // Compute state in selected range.
    const float dt = measurements[end_index].time_stamp_s - measurements[start_index].time_stamp_s;
    const Mat3 R_wi_i = rotation[start_index].matrix();
    const Quat q_wi_i = rotation[start_index];
    const Quat q_wi_j = rotation[end_index];
    const Vec3 p_wi_i = position[start_index];
    const Vec3 p_wi_j = position[end_index];
    const Vec3 v_wi_i = (position[start_index + 1] - position[start_index - 1])
        / (measurements[start_index + 1].time_stamp_s - measurements[start_index - 1].time_stamp_s);
    const Vec3 v_wi_j = (position[end_index + 1] - position[end_index - 1])
        / (measurements[end_index + 1].time_stamp_s - measurements[end_index - 1].time_stamp_s);
    const Vec3 g_w = Vec3(0, 0, 9.81);

    // Preintegrate all imu measurements.
    ImuPreintegrateBlock<DorF> block;
    block.SetImuNoiseSigma(1e-2f, 1e-2f, 1e-2f, 1e-2f);
    for (int32_t i = start_index + 1; i < end_index + 1; ++i) {
        block.Propagate(measurements[i - 1], measurements[i]);
    }

    block.Information();
    ReportInfo("dr_dbg\n" << block.dr_dbg());
    ReportInfo("dv_dba\n" << block.dv_dba());
    ReportInfo("dv_dbg\n" << block.dv_dbg());
    ReportInfo("dp_dba\n" << block.dp_dba());
    ReportInfo("dp_dbg\n" << block.dp_dbg());
}

void TestOldImuPreintegration(std::vector<ImuMeasurement> &measurements,
                              std::vector<Vec3> &position,
                              std::vector<Quat> &rotation) {
    ReportInfo(YELLOW ">> Test old imu preintegration." RESET_COLOR);

    // Define range.
    const int32_t start_index = 1;
    const int32_t end_index = measurements.size() - 2;

    // Compute state in selected range.
    const float dt = measurements[end_index].time_stamp_s - measurements[start_index].time_stamp_s;
    const Mat3 R_wi_i = rotation[start_index].matrix();
    const Quat q_wi_i = rotation[start_index];
    const Quat q_wi_j = rotation[end_index];
    const Vec3 p_wi_i = position[start_index];
    const Vec3 p_wi_j = position[end_index];
    const Vec3 v_wi_i = (position[start_index + 1] - position[start_index - 1])
        / (measurements[start_index + 1].time_stamp_s - measurements[start_index - 1].time_stamp_s);
    const Vec3 v_wi_j = (position[end_index + 1] - position[end_index - 1])
        / (measurements[end_index + 1].time_stamp_s - measurements[end_index - 1].time_stamp_s);
    const Vec3 g_w = Vec3(0, 0, 9.81);

    // Preintegrate all imu measurements.
    IMUPreintegration block(Vec3::Zero(), Vec3::Zero());
    IMUPreintegration::Q = Eigen::Matrix<float, 18, 18>::Identity() * 1e-4f;
    for (int32_t i = start_index; i < end_index + 1; ++i) {
        block.Propagate(measurements[i].time_stamp_s, measurements[i].accel, measurements[i].gyro);
    }

    block.PrintContent();
    ReportInfo("dr_dbg\n" << block.GetDrDbg());
    ReportInfo("dv_dba\n" << block.GetDvDba());
    ReportInfo("dv_dbg\n" << block.GetDvDbg());
    ReportInfo("dp_dba\n" << block.GetDpDba());
    ReportInfo("dp_dbg\n" << block.GetDpDbg());
}

void TestImuIntegration(std::vector<ImuMeasurement> &measurements,
                        std::vector<Vec3> &position,
                        std::vector<Quat> &rotation) {
    ReportInfo(YELLOW ">> Test imu integration." RESET_COLOR);

    // Define range.
    const int32_t start_index = 1;
    const int32_t end_index = measurements.size() - 2;

    // Compute state in selected range.
    const Quat q_wi_i = rotation[start_index];
    const Quat q_wi_j = rotation[end_index];
    const Vec3 p_wi_i = position[start_index];
    const Vec3 p_wi_j = position[end_index];
    const Vec3 v_wi_i = (position[start_index + 1] - position[start_index - 1])
        / (measurements[start_index + 1].time_stamp_s - measurements[start_index - 1].time_stamp_s);
    const Vec3 v_wi_j = (position[end_index + 1] - position[end_index - 1])
        / (measurements[end_index + 1].time_stamp_s - measurements[end_index - 1].time_stamp_s);
    const Vec3 g_w = Vec3(0, 0, 9.81);

    // Prepare for integration.
    ImuState state_i(p_wi_i, q_wi_i, v_wi_i, Vec3::Zero(), Vec3::Zero(), g_w, measurements[start_index].time_stamp_s);
    ImuState state_j = state_i;
    Mat15 cov_i = Mat15::Zero();
    Mat15 cov_j = cov_i;
    Vec3 mid_gyro = Vec3::Zero();
    Vec3 mid_accel = Vec3::Zero();

    // Integrate state and covariance.
    Imu imu_model;
    imu_model.options().kAccelNoise = 1e-2f;
    imu_model.options().kGyroNoise = 1e-2f;
    imu_model.options().kAccelRandomWalk = 1e-2f;
    imu_model.options().kGyroRandomWalk = 1e-2f;
    for (int32_t i = start_index + 1; i < end_index + 1; ++i) {
        imu_model.PropagateNominalState(measurements[i - 1], measurements[i], state_i, state_j, mid_accel, mid_gyro);
        imu_model.PropagateResidualStateCovariance(measurements[i - 1], measurements[i], mid_accel, mid_gyro, state_i, state_j, cov_i, cov_j);
        state_i = state_j;
        cov_i = cov_j;
    }

    // Compute integration residual.
    Vec9 residual = Vec9::Zero();
    residual.segment<3>(ImuIndex::kPosition) = p_wi_j - state_j.p_wi;
    residual.segment<3>(ImuIndex::kVelocity) = v_wi_j - state_j.v_wi;
    residual.segment<3>(ImuIndex::kRotation) = 2.0f * (state_j.q_wi.inverse() * q_wi_j).vec();

    ReportInfo("Residual of imu integration is " << LogVec(residual));
    ReportInfo("Covariance of imu integration is\n" << cov_j);
}

int main(int argc, char **argv) {
    std::string imu_file;
    if (argc == 2) {
        imu_file = argv[1];
    }

    ReportInfo(YELLOW ">> Test imu model." RESET_COLOR);
    LogFixPercision(5);

    std::vector<ImuMeasurement> measurements;
    std::vector<Vec3> position;
    std::vector<Quat> rotation;
    if (LoadImuMeasurements(imu_file, measurements, position, rotation) == false) {
        return 0;
    }

    TestImuIntegration(measurements, position, rotation);

    TestImuPreintegration(measurements, position, rotation);

    TestOldImuPreintegration(measurements, position, rotation);

    return 0;
}
