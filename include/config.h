#ifndef AEROSIM_CONFIG_H
#define AEROSIM_CONFIG_H

#include <string>

#include "constants.h"

/**
 * @file config.h
 * @brief Command-line configuration for AeroSim.
 *
 * AeroSim is fully configurable from the command line. This header
 * defines the parameter set and its parsing/validation rules. Every
 * parameter maps to a physical quantity documented in constants.h and
 * docs/PHYSICS.md; nothing here is a unitless knob.
 */

namespace aerosim {

/// Complete simulation configuration. All fields in SI units.
struct Config {
    double dryMass = kDefaultDryMass;                    ///< kg
    double thrust = kDefaultThrust;                      ///< N
    double initialFuel = kDefaultFuelMass;               ///< kg
    double timeStep = kDefaultTimeStep;                  ///< s
    double gravity = kSeaLevelGravity;                   ///< m/s^2
    double specificFuelConsumption =                     ///< kg/(N*s)
        kDefaultSpecificFuelConsumption;
    double frontalArea = kDefaultFrontalArea;             ///< m^2
    double dragCoefficient = kDefaultDragCoefficient;    ///< subsonic, -
    double maxSimulationTime = kMaxSimulationTime;       ///< s
    std::string outputCsvPath = "output/flight_data.csv"; ///< file path
};

/// Parse result of parseCommandLine().
enum class ParseStatus {
    Ok,         ///< Run the simulation with the returned config.
    HelpShown,  ///< -h/--help was requested; exit successfully.
    Error,      ///< A message was printed; exit with failure.
};

/// Outcome of parseCommandLine(): status plus the parsed config.
struct ParseResult {
    ParseStatus status;
    Config config;
};

/**
 * @brief Parse argv into a Config, printing errors to stderr.
 *
 * Accepted flags (each followed by its value):
 *   -m  <kg>    dry (structural) mass,      > 0
 *   -t  <N>     engine thrust,             >= 0
 *   -f  <kg>    initial propellant mass,   >= 0
 *   -dt <s>     integration time step,     > 0
 *   -g  <m/s^2> gravitational acceleration, > 0
 *   -fc <kg/(N*s)> specific fuel consumption, >= 0
 *   -a  <m^2>   reference (frontal) area,  > 0
 *   -cd <->     subsonic drag coefficient, >= 0
 *   -o  <path>  output CSV path
 *   -h / --help usage text
 * Unknown flags are reported as errors (they almost always mean a
 * typo silently producing a wrong simulation, which is the worst
 * failure mode for a physics tool).
 */
ParseResult parseCommandLine(int argc, char** argv);

/// Human-readable usage text, also printed by -h/--help.
std::string usageText();

}  // namespace aerosim

#endif  // AEROSIM_CONFIG_H
