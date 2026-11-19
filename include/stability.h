#ifndef STABILITY_H
#define STABILITY_H

/**
 * @file stability.h
 * @brief Longitudinal static stability: component CG locations, aircraft CG,
 * static margin from tail volume coefficient (Raymer ch. 6 & 16).
 */
#include <vector>
#include <string>
#include "geometry.h"
#include "weights.h"

struct CgItem {
    const char* name;
    double massKg;
    double xM;    // station from nose (m)
};

struct StabilityResult {
    double cgM = 0;            // aircraft CG from nose (m)
    double npM = 0;            // neutral point from nose (m)
    double staticMargin = 0;   // (np - cg)/MAC, dimensionless
    double cgFwdM = 0;         // most-forward CG (no payload)
    double cgAftM = 0;         // most-aft CG (full payload aft)
    double staticMarginFwd = 0;
    double staticMarginAft = 0;
    bool cgWithinLimits = false;
    double wingRootM = 0;   // wing root LE station (m, adjusted for CG target)
    std::vector<CgItem> items;
};

struct StabilityInputs {
    double payloadKg = 180;
    double crewKg = 80;
    double fuelKg = 100;
    double w0Kg = 1000;
    double htVolumeCoeff = 0.7;
    double htLiftSlopeRatio = 0.9;  // tail lift slope / wing lift slope
    double acFracWing = 0.25;       // wing AC at 25% MAC
    double xWingRootM = 1.6;        // wing root LE station from nose (m)
    double clAlphaWing = 5.0;       // per rad
};

StabilityResult stabilityAnalysis(const Geometry& g, const WeightResult& w,
                                  const StabilityInputs& in);

#endif
