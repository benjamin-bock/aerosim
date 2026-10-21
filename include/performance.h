#ifndef PERFORMANCE_H
#define PERFORMANCE_H

/**
 * @file performance.h
 * @brief Performance analysis of a converged design: range, ceiling,
 * rate of climb, takeoff/landing distances, V-n diagram.
 */
#include "aero.h"

struct PerformanceResult {
    double rangeKm = 0;          // achievable with available fuel
    double ceilingM = 0;         // service ceiling (ROC = 0.508 m/s)
    double rocSLms = 0;          // rate of climb at best climb speed, SL
    double vBestClimb = 0;
    double vStallClean = 0;
    double vStallLanding = 0;
    double vMaxLevel = 0;        // at SL
    double takeoffDistM = 0;     // to clear obstacle
    double landingDistM = 0;
    double cruiseSpeedKmh = 0;
    double enduranceH = 0;
    double vDive = 0;            // V-n dive speed
    double nMaxPos = 3.8, nMaxNeg = -1.52;
    double vsPositive = 0, vsNegative = 0; // corner speeds (m/s)
};

struct PerformanceInputs {
    double w0Kg = 1000;
    double fuelKg = 120;
    double powerW = 150000;     // available shaft power
    double propEff = 0.8;
    double sfcKgPerkWh = 0.30;
    bool propDriven = true;
    double thrustN = 0;          // if jet
    double jetTsfc = 0.55;       // 1/h
    double cruiseAltM = 2500;
    double cruiseSpeedMs = 70;
    double obstacleM = 15.2;
    double muRoll = 0.04;
    double muBrake = 0.5;
    double clmaxLanding = 2.0;
    double flapCdLanding = 0.06; // extra CD0 in landing config
    // filled by caller
    double clmaxTO = 1.9;
};

PerformanceResult analyzePerformance(const AeroResult& a, const Geometry& g,
                                     const PerformanceInputs& in);

#endif
