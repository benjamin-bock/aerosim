#include <cmath>
#include <algorithm>
#include "../include/propeller.h"
#include "../include/atmosphere.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double Propeller::efficiency(double J) const {
    // Empirical fit to NACA Clark-Y propeller charts (variable pitch):
    // eta rises ~linearly to a peak near J~0.85 then falls off.
    // Peak efficiency and peak location scale with activity factor/blades.
    if (J <= 0.02) return 0.0;
    double jPeak = 0.85 * (1.0 + (activityFactor - 100.0) * 0.0015);
    double etaPeak = variablePitch ? 0.85 : 0.75;
    etaPeak -= (nBlades > 2 ? 0.02 * (nBlades - 2) : 0.0);
    double x = J / jPeak;
    if (x <= 1.0) {
        // smooth rise: eta = etaPeak * (x^0.85) with slight S-curve
        return etaPeak * std::pow(x, 0.85) * (1.0 - 0.12 * x * x) / 0.88;
    }
    // beyond peak: quadratic decay, zero near J = 1.6 * jPeak
    double over = (J - jPeak) / (0.75 * jPeak);
    return etaPeak * std::max(0.0, 1.0 - over * over);
}

double Propeller::staticThrust(double powerW) const {
    // Momentum theory: T = (pi D^2/2 * rho * P^2)^(1/3) with figure of merit
    double rho = isaDensity(0.0);
    double area = M_PI * std::pow(diameterM / 2.0, 2);
    double fom = 0.70; // figure of merit, typical cruise prop at static
    double tIdeal = std::pow(2.0 * area * rho * powerW * powerW, 1.0 / 3.0);
    return fom * tIdeal;
}

double Propeller::thrust(double powerW, double speedMs, double rpm) const {
    double n = std::max(rpm / 60.0, 1.0);
    double J = speedMs / (n * diameterM);
    if (speedMs < 8.0)
        return std::min(staticThrust(powerW), powerW / 5.0); // blend to static
    double eta = efficiency(J);
    return powerW * eta / speedMs;
}

double Propeller::recommendedDiameter(double powerKW, bool variablePitch) {
    // Fit to real aircraft: C172 120kW->1.87m, SR22 230kW->2.03m, PT6A-42 635kW->2.4m
    double d = 0.91 * std::pow(std::max(powerKW, 30.0), 0.163);
    if (!variablePitch) d *= 1.05;
    return std::min(std::max(d, 1.2), 3.2);
}
