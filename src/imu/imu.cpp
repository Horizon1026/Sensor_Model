#include "imu.h"

namespace SENSOR_MODEL {

template <>
void Imu::PropagateNominalState<ImuState15>(const ImuMeasurement &measurement,
                                            const ImuState15 &state_i,
                                            ImuState15 &state_j) {

}

template <>
void Imu::PropagateNominalState<ImuState18>(const ImuMeasurement &measurement,
                                            const ImuState18 &state_i,
                                            ImuState18 &state_j) {

}

template <>
void Imu::PropagateNominalStateCovariance<Mat15>(const ImuMeasurement &measurement,
                                                 const Mat15 &covariance_i,
                                                 Mat15 &covariance_j) {

}

template <>
void Imu::PropagateNominalStateCovariance<Mat18>(const ImuMeasurement &measurement,
                                                 const Mat18 &covariance_i,
                                                 Mat18 &covariance_j) {

}

}
