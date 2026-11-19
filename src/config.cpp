#include "config.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace aerosim {

std::string usageText() {
    return
        "AeroSim - vertical rocket flight simulator\n"
        "Usage: aerosim [flags]\n"
        "\n"
        "Physics: US Standard Atmosphere 1976 (COESA) up to 86 km,\n"
        "Mach-dependent drag, semi-implicit (symplectic) Euler integration.\n"
        "See docs/PHYSICS.md for every equation and its source.\n"
        "\n"
        "Flags:\n"
        "  -m  <value>  Dry (structural) mass [kg], must be > 0\n"
        "  -t  <value>  Engine thrust [N], >= 0\n"
        "  -f  <value>  Initial propellant mass [kg], >= 0\n"
        "  -dt <value>  Integration time step [s], > 0\n"
        "  -g  <value>  Gravitational acceleration [m/s^2], > 0\n"
        "  -fc <value>  Specific fuel consumption [kg/(N*s)], >= 0\n"
        "  -a  <value>  Reference (frontal) area [m^2], > 0\n"
        "  -cd <value>  Subsonic drag coefficient [-], >= 0\n"
        "  -o  <path>   Output CSV path (default output/flight_data.csv)\n"
        "  -h, --help   Show this help and exit\n";
}

namespace {

/// Parse a double, or fail with a clear message.
bool parseDouble(const std::string& flag, const std::string& value, double* out) {
    try {
        *out = std::stod(value);
    } catch (const std::exception&) {
        std::cerr << "Error: value of " << flag << " is not a number: '"
                  << value << "'" << std::endl;
        return false;
    }
    return true;
}

}  // namespace

ParseResult parseCommandLine(int argc, char** argv) {
    ParseResult result;
    result.status = ParseStatus::Ok;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            std::cout << usageText();
            result.status = ParseStatus::HelpShown;
            return result;
        }

        // Every remaining flag takes exactly one value.
        if (i + 1 >= argc) {
            std::cerr << "Error: missing value for " << arg << std::endl;
            result.status = ParseStatus::Error;
            return result;
        }
        const std::string value = argv[++i];
        double v = 0.0;

        if (arg == "-m") {
            if (!parseDouble(arg, value, &v) || v <= 0.0) {
                std::cerr << "Error: dry mass (-m) must be a positive number." << std::endl;
                result.status = ParseStatus::Error;
                return result;
            }
            result.config.dryMass = v;
        } else if (arg == "-t") {
            if (!parseDouble(arg, value, &v) || v < 0.0) {
                std::cerr << "Error: thrust (-t) must be a non-negative number." << std::endl;
                result.status = ParseStatus::Error;
                return result;
            }
            result.config.thrust = v;
        } else if (arg == "-f") {
            if (!parseDouble(arg, value, &v) || v < 0.0) {
                std::cerr << "Error: initial fuel mass (-f) must be a non-negative number."
                          << std::endl;
                result.status = ParseStatus::Error;
                return result;
            }
            result.config.initialFuel = v;
        } else if (arg == "-dt") {
            if (!parseDouble(arg, value, &v) || v <= 0.0) {
                std::cerr << "Error: time step (-dt) must be a positive number." << std::endl;
                result.status = ParseStatus::Error;
                return result;
            }
            result.config.timeStep = v;
        } else if (arg == "-g") {
            if (!parseDouble(arg, value, &v) || v <= 0.0) {
                std::cerr << "Error: gravity (-g) must be a positive number." << std::endl;
                result.status = ParseStatus::Error;
                return result;
            }
            result.config.gravity = v;
        } else if (arg == "-fc") {
            if (!parseDouble(arg, value, &v) || v < 0.0) {
                std::cerr << "Error: specific fuel consumption (-fc) must be a non-negative "
                             "number." << std::endl;
                result.status = ParseStatus::Error;
                return result;
            }
            result.config.specificFuelConsumption = v;
        } else if (arg == "-a") {
            if (!parseDouble(arg, value, &v) || v <= 0.0) {
                std::cerr << "Error: frontal area (-a) must be a positive number." << std::endl;
                result.status = ParseStatus::Error;
                return result;
            }
            result.config.frontalArea = v;
        } else if (arg == "-cd") {
            if (!parseDouble(arg, value, &v) || v < 0.0) {
                std::cerr << "Error: drag coefficient (-cd) must be a non-negative number."
                          << std::endl;
                result.status = ParseStatus::Error;
                return result;
            }
            result.config.dragCoefficient = v;
        } else if (arg == "-o") {
            result.config.outputCsvPath = value;
        } else {
            std::cerr << "Error: unknown argument '" << arg << "'." << std::endl
                      << "Run with -h to see the available flags." << std::endl;
            result.status = ParseStatus::Error;
            return result;
        }
    }

    return result;
}

}  // namespace aerosim
