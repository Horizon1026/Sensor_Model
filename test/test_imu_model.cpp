#include "log_api.h"
#include "imu.h"

int main(int argc, char **argv) {
    LogInfo(YELLOW ">> Test imu model." RESET_COLOR);

    SENSOR_MODEL::Imu imu;

    return 0;
}
