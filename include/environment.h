#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

/**
 * @struct Environment
 * @brief Represents the physical world conditions.
 */
struct Environment {
    double gravity;      // m/s^2
    double rho0;         // Sea-level air density (kg/m^3)
    double scaleHeight;  // Scale height for atmospheric model (m)
    double getDensityAtAltitude(double altitude) const;

    // Constructor with Earth defaults
    Environment(double g = 9.81, double r = 1.225, double h = 8500.0)
        : gravity(g), rho0(r), scaleHeight(h) {}
};

#endif