#include <cmath>
#include "../include/atmosphere.h"

Atmosphere isaAt(double altM) {
    Atmosphere a;
    const double R = 287.05, gamma = 1.4;
    const double T0 = 288.15, P0 = 101325.0, rho0 = 1.225;
    if (altM < 11000.0) {
        a.temperature = T0 - 6.5e-3 * altM;
        a.pressure = P0 * std::pow(a.temperature / T0, 5.2559);
    } else if (altM < 20000.0) {
        a.temperature = 216.65;
        a.pressure = 22632.1 * std::exp(-9.80665 * (altM - 11000.0) / (R * a.temperature));
    } else {
        a.temperature = 216.65 + 1.0e-3 * (altM - 20000.0);
        a.pressure = 5474.9 * std::pow(a.temperature / 216.65, -34.16);
    }
    a.density = a.pressure / (R * a.temperature);
    a.soundSpeed = std::sqrt(gamma * R * a.temperature);
    // Sutherland's law
    a.viscosity = 1.458e-6 * std::pow(a.temperature, 1.5) / (a.temperature + 110.4);
    (void)rho0;
    return a;
}

double isaDensity(double altM) { return isaAt(altM).density; }
double isaPressure(double altM) { return isaAt(altM).pressure; }
double isaTemperature(double altM) { return isaAt(altM).temperature; }
double isaSoundSpeed(double altM) { return isaAt(altM).soundSpeed; }
