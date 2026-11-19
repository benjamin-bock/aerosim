#ifndef AERO_H
#define AERO_H

/**
 * @file aero.h
 * @brief Aerodynamic buildup: component drag buildup (Raymer ch. 12),
 * high-lift devices, ground effects, drag polar & CLmax estimation.
 */
#include "geometry.h"
#include "atmosphere.h"

enum class FlapType { None, Plain, Split, SingleSlot, Fowler, DoubleSlot };

struct AeroResult {
    double cd0 = 0;
    double oswald = 0.8;
    double k = 0.05;
    double clmaxClean = 1.4;
    double clmaxTakeoff = 1.6;
    double clmaxLanding = 2.0;
    double ldMax = 0;
    double cdCruise = 0;      // including compressibility/misc
    double compressibilityRise = 0; // extra CD from Mach
    double clAlpha3D = 0;     // per rad
    double cl0 = 0;
    double alpha0 = 0;
};

struct ComponentDrag {
    const char* name;
    double swet;
    double cf;
    double ff;      // form factor
    double qInterf = 1.0; // interference factor
    double cdContrib;
};

AeroResult aeroAnalysis(const Geometry& g, const NacaAirfoil& foil,
                        double cruiseSpeed, double cruiseAlt,
                        FlapType flap, double flapChordFrac = 0.3,
                        double flapSpanFrac = 0.6, bool fixedGear = true);

// Flap CLmax increment (Raymer Table 12.5 / ch. 12)
double flapDeltaCLmax(FlapType t, double chordFrac, double spanFrac);
double flapDeltaCd(FlapType t);

// Sweep-corrected CLmax (Raymer 12.49-ish)
double clmaxSweepCorrected(double clmax2d, double sweepQC);

// Landing gear drag (Raymer 12.30)
double gearDragArea(bool retract, double w0N);

#endif
