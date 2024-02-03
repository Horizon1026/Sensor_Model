#ifndef _SENSOR_MODEL_IMU_PREINTEGRATE_H_
#define _SENSOR_MODEL_IMU_PREINTEGRATE_H_

#include "datatype_basic.h"
#include "math_kinematics.h"
#include "imu_measurement.h"
#include "imu_state.h"

namespace SENSOR_MODEL {

/* Imu preintegrate block. */
template <typename Scalar = float>
class ImuPreintegrateBlock {

public:
    ImuPreintegrateBlock() = default;
    virtual ~ImuPreintegrateBlock() = default;

    // Reset all or integrated states.
    void Reset();
    void ResetIntegratedStates();

    // Print all states.
    void Information() const;
    void SimpleInformation() const;

    // Propagate integrate block.
    bool Propagate(const ImuMeasurement &measure_i,
                   const ImuMeasurement &measure_j);

    // Correct integrate block with new bias_a and bias_g.
    void Correct(const TVec3<Scalar> &new_ba,
                 const TVec3<Scalar> &new_bg,
                 TVec3<Scalar> &corr_p_ij,
                 TQuat<Scalar> &corr_q_ij,
                 TVec3<Scalar> &corr_v_ij);

    // Set noise sigma vector.
    void SetImuNoiseSigma(const Scalar accel_noise,
                          const Scalar gyro_noise,
                          const Scalar accel_random_walk,
                          const Scalar gyro_random_walk);

    // Reference for member variables.
    TVec3<Scalar> &p_ij() { return p_ij_; }
    TQuat<Scalar> &q_ij() { return q_ij_; }
    TVec3<Scalar> &v_ij() { return v_ij_; }
    TVec3<Scalar> &bias_accel() { return bias_accel_; }
    TVec3<Scalar> &bias_gyro() { return bias_gyro_; }
    TMat15<Scalar> &jacobian() { return jacobian_; }
    TMat15<Scalar> &covariance() { return covariance_; }
    TVec18<Scalar> &noise_sigma() { return noise_sigma_; }
    Scalar &integrate_time_s() { return integrate_time_s_; }

    // Const reference for member variables.
    const TVec3<Scalar> &p_ij() const { return p_ij_; }
    const TQuat<Scalar> &q_ij() const { return q_ij_; }
    const TVec3<Scalar> &v_ij() const { return v_ij_; }
    const TVec3<Scalar> &bias_accel() const { return bias_accel_; }
    const TVec3<Scalar> &bias_gyro() const { return bias_gyro_; }
    const TMat15<Scalar> &jacobian() const { return jacobian_; }
    const TMat15<Scalar> &covariance() const { return covariance_; }
    const TVec18<Scalar> &noise_sigma() const { return noise_sigma_; }
    const Scalar &integrate_time_s() const { return integrate_time_s_; }

    inline TMat3<Scalar> dr_dbg() const { return jacobian_.template block<3, 3>(ImuIndex::kRotation, ImuIndex::kBiasGyro); }
    inline TMat3<Scalar> dv_dba() const { return jacobian_.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kBiasAccel); }
    inline TMat3<Scalar> dv_dbg() const { return jacobian_.template block<3, 3>(ImuIndex::kVelocity, ImuIndex::kBiasGyro); }
    inline TMat3<Scalar> dp_dba() const { return jacobian_.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kBiasAccel); }
    inline TMat3<Scalar> dp_dbg() const { return jacobian_.template block<3, 3>(ImuIndex::kPosition, ImuIndex::kBiasGyro); }

private:
    TVec3<Scalar> p_ij_ = TVec3<Scalar>::Zero();
    TQuat<Scalar> q_ij_ = TQuat<Scalar>::Identity();
    TVec3<Scalar> v_ij_ = TVec3<Scalar>::Zero();

    TVec3<Scalar> bias_accel_ = TVec3<Scalar>::Zero();
    TVec3<Scalar> bias_gyro_ = TVec3<Scalar>::Zero();

    TMat15<Scalar> jacobian_ = TMat15<Scalar>::Identity();
    TMat15<Scalar> covariance_ = TMat15<Scalar>::Zero();

    // Sequence is na_i, ng_i, na_j, ng_j, nwa, nwg.
    TVec18<Scalar> noise_sigma_ = TVec18<Scalar>::Ones() * static_cast<Scalar>(1e-6);

    Scalar integrate_time_s_ = static_cast<Scalar>(0);

};

}

#endif // end of _SENSOR_MODEL_IMU_PREINTEGRATE_H_
