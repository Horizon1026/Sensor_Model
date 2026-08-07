#include "basic_type.h"
#include "slam_basic_math.h"
#include "slam_log_reporter.h"

#include "baro_measurement.h"
#include "barometer.h"

#include "cmath"

using namespace slam_utility;
using namespace sensor_model;

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test barometer model." RESET_COLOR);

    Barometer barometer;

    // Sea level pressure maps to ~0 m altitude for both methods.
    {
        BaroMeasurement measure;
        measure.pressure_Kpa = 101.325f;
        measure.temperature_degC = 25.0f;
        const float standard = barometer.ConvertBaroToAltitude(measure);
        const float compensated = barometer.ConvertBaroWithTemperatureToAltitude(measure);
        ReportInfo("  - At sea level pressure, standard altitude = " << standard
                   << " m, compensated altitude = " << compensated << " m.");
        if (std::abs(standard) > 0.1f || std::abs(compensated) > 0.1f) {
            ReportError("    Failed: sea level pressure should map to ~0 m.");
            return -1;
        }
    }

    // When the measured temperature equals the ISA reference (15 degC), the
    // compensated altitude collapses to the standard one.
    {
        BaroMeasurement measure;
        measure.pressure_Kpa = 95.0f;
        measure.temperature_degC = 15.0f;
        const float standard = barometer.ConvertBaroToAltitude(measure);
        const float compensated = barometer.ConvertBaroWithTemperatureToAltitude(measure);
        ReportInfo("  - At 15 degC, standard altitude = " << standard
                   << " m, compensated altitude = " << compensated << " m.");
        if (std::abs(standard - compensated) > 1.0f) {
            ReportError("    Failed: compensated altitude should match standard altitude at 15 degC.");
            return -1;
        }
    }

    // Warmer air stretches the atmosphere, so the same pressure maps to a
    // higher altitude; colder air does the reverse.
    {
        BaroMeasurement measure;
        measure.pressure_Kpa = 95.0f;
        measure.temperature_degC = 35.0f;
        const float hot = barometer.ConvertBaroWithTemperatureToAltitude(measure);
        measure.temperature_degC = -5.0f;
        const float cold = barometer.ConvertBaroWithTemperatureToAltitude(measure);
        ReportInfo("  - At 35 degC, compensated altitude = " << hot << " m; at -5 degC, compensated altitude = " << cold << " m.");
        if (hot <= cold) {
            ReportError("    Failed: warmer air should yield a higher altitude for the same pressure.");
            return -1;
        }
    }

    // Sanity: the ISA pressure at 500 m should round-trip to ~500 m.
    {
        BaroMeasurement measure;
        measure.pressure_Kpa = 95.461f;  // kPa, ISA pressure at 500 m.
        measure.temperature_degC = 15.0f;
        const float altitude = barometer.ConvertBaroToAltitude(measure);
        const float compensated = barometer.ConvertBaroWithTemperatureToAltitude(measure);
        ReportInfo("  - ISA pressure at 500 m maps back to " << altitude
                   << " m (standard), " << compensated << " m (compensated).");
        if (std::abs(altitude - 500.0f) > 2.0f) {
            ReportError("    Failed: ISA round-trip should recover ~500 m.");
            return -1;
        }
    }

    ReportInfo(GREEN ">> Test barometer model passed." RESET_COLOR);
    return 0;
}
