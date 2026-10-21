#ifndef WEIGHTS_H
#define WEIGHTS_H

/**
 * @file weights.h
 * @brief Component weight buildup (Raymer ch. 6/15, general-aviation
 * piston singles) with Table 3.1 empty-weight regression as cross-check.
 */
#include "geometry.h"

struct WeightResult {
    double wingKg = 0;
    double fuselageKg = 0;
    double htKg = 0;
    double vtKg = 0;
    double engineKg = 0;      // installed, dry
    double gearKg = 0;
    double fuelSystemKg = 0;
    double flightControlsKg = 0;
    double electricalKg = 0;
    double avionicsKg = 0;
    double FurnishingsKg = 0;
    double paintKg = 0;
    double totalEmptyKg = 0;
    double table31Frac = 0;   // Raymer Table 3.1 regression We/W0
    double buildupFrac = 0;   // We/W0 from component buildup
    double nUltimate = 3.8;
};

struct WeightInputs {
    double w0Kg = 1000;
    double powerW = 100000;     // rated shaft power
    double engineMassKg = 90;   // dry engine mass per unit (from engine db)
    double fuelKg = 100;
    double fuelInWingKg = 0;    // fuel carried in wing tanks
    double cruiseLD = 12;       // cruise L/D (fuselage eq.)
    double cruiseDynamicPressure = 3000; // Pa
    int nSeats = 2;
    int nEngines = 1;
    bool retractGear = false;
    bool composite = false;     // composite airframe
    double loadFactorUlt = 5.7; // GA: n_ult = 1.5 * 3.8
};

WeightResult componentWeights(const Geometry& g, const WeightInputs& in);

// Raymer Table 3.1 regression (W0 in kg)
double raymerWeFrac(double w0Kg, double A, double c);

#endif
