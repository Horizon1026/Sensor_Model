#include "imu.h"

namespace SENSOR_MODEL {

bool Imu::PropagateNominalState(const ImuMeasurement &measurement,
                                const ImuState &state_i,
                                ImuState &state_j) {

    return true;
}

bool Imu::PropagateNominalStateCovariance(const ImuMeasurement &measurement,
                                          const Mat &covariance_i,
                                          Mat &covariance_j) {

    return true;
}

}
