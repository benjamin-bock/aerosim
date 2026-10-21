/**
 * @file main.cpp
 * @brief AeroSim entry point: parse configuration, run the flight
 *        simulation, write the telemetry CSV, and print a mission summary.
 *
 * PIPELINE
 * --------
 *   argv -> Config (config.h)
 *        -> Atmosphere (US Standard 1976, atmosphere.h)
 *        -> Rocket + semi-implicit Euler loop (rocket.h)
 *        -> CSV telemetry (consumed by scripts/plot_results.py)
 *        -> console mission summary (apogee, max Mach/q, burnout, ...)
 *
 * TERMINATION CONDITIONS
 * ----------------------
 * The loop stops when, in order:
 *   1. the rocket lands (altitude returns to 0 with downward velocity),
 *   2. the mission clock exceeds the safety cutoff (default 600 s),
 *      which guards against parameter sets that never come down.
 */

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>

#include "config.h"
#include "rocket.h"

namespace {

/// Create the parent directory of the output CSV if it does not exist.
/// Uses POSIX mkdir; failure to create is not fatal (maybe stdout paths
/// or an already-existing directory), the ofstream will tell us later.
void ensureOutputDirectory(const std::string& csvPath) {
    const size_t slash = csvPath.find_last_of('/');
    if (slash == std::string::npos) {
        return;
    }
    ::mkdir(csvPath.substr(0, slash).c_str(), 0755);
}

/// Write the CSV header. Column order matches scripts/plot_results.py.
void writeCsvHeader(std::ofstream& out) {
    out << "Time_s,Altitude_m,Velocity_mps,Acceleration_mps2,Mass_kg,Fuel_kg,"
           "Mach,Drag_N,Thrust_N,DynamicPressure_Pa,Density_kgm3,DragCoefficient,"
           "SpeedOfSound_mps,Phase\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::cout << "--- AeroSim ---" << std::endl;
    std::cout << "Vertical rocket flight simulator. Physics documented in docs/PHYSICS.md."
              << std::endl;

    const aerosim::ParseResult parsed = aerosim::parseCommandLine(argc, argv);
    if (parsed.status == aerosim::ParseStatus::HelpShown) {
        return 0;
    }
    if (parsed.status == aerosim::ParseStatus::Error) {
        return 1;
    }
    const aerosim::Config& cfg = parsed.config;

    std::cout << "Configuration:\n"
              << "  Dry mass                    : " << cfg.dryMass << " kg\n"
              << "  Engine thrust               : " << cfg.thrust << " N\n"
              << "  Initial propellant mass     : " << cfg.initialFuel << " kg\n"
              << "  Specific fuel consumption   : " << cfg.specificFuelConsumption
              << " kg/(N*s)  (effective exhaust velocity v_e = 1/sfc = "
              << (cfg.specificFuelConsumption > 0.0 ? 1.0 / cfg.specificFuelConsumption : 0.0)
              << " m/s)\n"
              << "  Frontal reference area      : " << cfg.frontalArea << " m^2\n"
              << "  Subsonic drag coefficient   : " << cfg.dragCoefficient << "\n"
              << "  Gravity                     : " << cfg.gravity << " m/s^2\n"
              << "  Integration time step      : " << cfg.timeStep << " s (semi-implicit Euler)\n"
              << "  Atmosphere                  : US Standard Atmosphere 1976, layers 0-86 km\n"
              << std::endl;

    // Check the most basic feasibility criterion before simulating:
    // thrust-to-weight ratio at ignition. Below 1 the rocket cannot
    // lift off at all; we warn (and still run) because fuel drains and
    // the ratio can cross 1 later only for unrealistic engines.
    const double liftoffMass = cfg.dryMass + cfg.initialFuel;
    const double thrustToWeight = cfg.thrust / (liftoffMass * cfg.gravity);
    if (thrustToWeight <= 1.0) {
        std::cerr << "Warning: thrust-to-weight ratio at ignition is " << thrustToWeight
                  << " (<= 1). The rocket will not lift off." << std::endl;
    }

    const aerosim::Atmosphere atmosphere;
    aerosim::Rocket rocket(cfg.dryMass, cfg.thrust, cfg.initialFuel, cfg.frontalArea,
                           cfg.dragCoefficient, cfg.gravity, cfg.specificFuelConsumption);

    ensureOutputDirectory(cfg.outputCsvPath);
    std::ofstream csv(cfg.outputCsvPath);
    if (!csv.is_open()) {
        std::cerr << "Error: could not open '" << cfg.outputCsvPath << "' for writing."
                  << std::endl;
        return 1;
    }
    writeCsvHeader(csv);

    // --- Simulation loop -------------------------------------------------
    // Mission statistics tracked on the fly for the final summary.
    double apogee = 0.0;
    double apogeeTime = 0.0;
    double maxSpeed = 0.0;
    double maxMach = 0.0;
    double maxDynamicPressure = 0.0;  // "max Q", the classic rocketry metric
    double maxAcceleration = 0.0;
    double touchdownTime = -1.0;

    std::cout << "Ignition. Running simulation..." << std::endl;

    double t = 0.0;
    aerosim::Telemetry lastTelemetry = rocket.telemetry(t, atmosphere);
    csv << lastTelemetry.time << "," << lastTelemetry.altitude << "," << lastTelemetry.velocity
        << "," << lastTelemetry.acceleration << "," << lastTelemetry.mass << ","
        << lastTelemetry.fuel << "," << lastTelemetry.mach << "," << lastTelemetry.drag << ","
        << lastTelemetry.thrust << "," << lastTelemetry.dynamicPressure << ","
        << lastTelemetry.density << "," << lastTelemetry.dragCoefficient << ","
        << lastTelemetry.speedOfSound << ","
        << static_cast<int>(aerosim::FlightPhase::PreLaunch) << "\n";

    while (t < cfg.maxSimulationTime) {
        rocket.step(cfg.timeStep, atmosphere);
        t += cfg.timeStep;

        lastTelemetry = rocket.telemetry(t, atmosphere);

        // Update mission statistics.
        if (lastTelemetry.altitude > apogee) {
            apogee = lastTelemetry.altitude;
            apogeeTime = t;
        }
        maxSpeed = std::max(maxSpeed, std::abs(lastTelemetry.velocity));
        maxMach = std::max(maxMach, lastTelemetry.mach);
        maxDynamicPressure = std::max(maxDynamicPressure, lastTelemetry.dynamicPressure);
        maxAcceleration = std::max(maxAcceleration, lastTelemetry.acceleration);

        csv << lastTelemetry.time << "," << lastTelemetry.altitude << ","
            << lastTelemetry.velocity << "," << lastTelemetry.acceleration << ","
            << lastTelemetry.mass << "," << lastTelemetry.fuel << "," << lastTelemetry.mach
            << "," << lastTelemetry.drag << "," << lastTelemetry.thrust << ","
            << lastTelemetry.dynamicPressure << "," << lastTelemetry.density << ","
            << lastTelemetry.dragCoefficient << "," << lastTelemetry.speedOfSound << ","
            << static_cast<int>(rocket.phase()) << "\n";

        // Landing: altitude back at zero with downward motion arrested
        // by the ground clamp inside Rocket::step.
        if (rocket.altitude() <= 0.0 && rocket.velocity() == 0.0 && t > cfg.timeStep) {
            touchdownTime = t;
            break;
        }
    }

    csv.close();

    // --- Mission summary -------------------------------------------------
    const double burnout = rocket.burnoutTime();
    std::cout << "\n--- Mission summary ---" << std::endl;
    std::cout << "  Apogee altitude           : " << apogee << " m at t = " << apogeeTime
              << " s" << std::endl;
    std::cout << "  Maximum speed             : " << maxSpeed << " m/s" << std::endl;
    std::cout << "  Maximum Mach number       : " << maxMach << std::endl;
    std::cout << "  Maximum dynamic pressure  : " << maxDynamicPressure << " Pa (max Q)"
              << std::endl;
    std::cout << "  Maximum acceleration      : " << maxAcceleration << " m/s^2 ("
              << maxAcceleration / cfg.gravity << " g)" << std::endl;
    if (burnout >= 0.0) {
        std::cout << "  Propellant burnout        : t = " << burnout << " s" << std::endl;
    } else {
        std::cout << "  Propellant burnout        : not reached within the simulation" << std::endl;
    }
    if (touchdownTime > 0.0) {
        std::cout << "  Touchdown                 : t = " << touchdownTime << " s" << std::endl;
    } else {
        std::cout << "  Touchdown                 : not reached before the safety cutoff ("
                  << cfg.maxSimulationTime << " s)" << std::endl;
    }
    std::cout << "Telemetry written to '" << cfg.outputCsvPath << "'." << std::endl;
    std::cout << "Plot it with: python3 scripts/plot_results.py <csv> <png>" << std::endl;

    return 0;
}
