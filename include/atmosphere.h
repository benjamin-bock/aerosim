#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

/**
 * @file atmosphere.h
 * @brief Full ISA atmosphere (troposphere + stratosphere) up to 20 km.
 */
struct Atmosphere {
    double temperature;  // K
    double pressure;     // Pa
    double density;      // kg/m^3
    double soundSpeed;   // m/s
    double viscosity;    // Pa.s (Sutherland)
};

Atmosphere isaAt(double altitudeM);

double isaDensity(double altitudeM);
double isaPressure(double altitudeM);
double isaTemperature(double altitudeM);
double isaSoundSpeed(double altitudeM);

#endif
