#include <cmath>
#include <algorithm>
#include "../include/weights.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Raymer ch.15.3.3 General-Aviation weight equations (as implemented in the
// 5th-ed. relationships; all unit conversions handled here, results in kg).

static const double LB_PER_KG = 2.20462;
static const double FT2_PER_M2 = 10.7639;
static const double FT_PER_M = 3.28084;

double raymerWeFrac(double w0Kg, double A, double c) {
    return A * std::pow(std::max(w0Kg, 1.0), c);
}

WeightResult componentWeights(const Geometry& g, const WeightInputs& in) {
    WeightResult w;
    double W0lb = in.w0Kg * LB_PER_KG;
    double nUlt = in.loadFactorUlt;
    double lb2kg = 0.453592;

    // Wing (Raymer 15.46): 0.036 * Sw^0.758 * fuel^0.0035 * (AR/cos^2)^0.6
    //   * q^0.006 * taper^0.04 * (100tc)^-0.3 * (W0*Nult)^0.49
    double Swlb = g.wingArea * FT2_PER_M2;
    double fuelFactor = in.fuelInWingKg > 0
        ? std::pow(in.fuelInWingKg * LB_PER_KG, 0.0035) : 1.0;
    double cosSweep = std::cos(g.sweepQC);
    double qCruise = in.cruiseDynamicPressure / 47.88; // Pa -> psf
    double wingLb = 0.036 * std::pow(Swlb, 0.758) * fuelFactor
                  * std::pow(g.aspectRatio / (cosSweep * cosSweep), 0.6)
                  * std::pow(std::max(qCruise, 1.0), 0.006)
                  * std::pow(std::max(g.taper, 0.2), 0.04)
                  * std::pow(100.0 * g.thicknessRatio / cosSweep, -0.3)
                  * std::pow(W0lb * nUlt, 0.49);
    if (in.composite) wingLb *= 0.85;
    w.wingKg = wingLb * lb2kg;

    // Fuselage (Raymer 15.47): 0.052 * Swet^1.086 * (W0*Nult)^0.177 * Lt^-0.051
    //   * (L/D)^-0.072 * q^0.241
    double swetFuslb2 = g.swetFus * FT2_PER_M2;
    double Ltft = g.htArm * FT_PER_M;
    double fusLb = 0.052 * std::pow(std::max(swetFuslb2, 10.0), 1.086)
                 * std::pow(W0lb * nUlt, 0.177)
                 * std::pow(std::max(Ltft, 1.0), -0.051)
                 * std::pow(std::max(in.cruiseLD, 5.0), -0.072)
                 * std::pow(std::max(qCruise, 1.0), 0.241);
    if (in.composite) fusLb *= 0.85;
    w.fuselageKg = fusLb * lb2kg;

    // Horizontal tail (Raymer 15.48): 0.016*(W0*Nult)^0.414*q^0.168*S^0.896*
    //   (100tc)^-0.12*(AR)^0.043*taper^-0.02
    double htlb = 0.016 * std::pow(W0lb * nUlt, 0.414)
                 * std::pow(std::max(qCruise, 1.0), 0.168)
                 * std::pow(g.htArea * FT2_PER_M2, 0.896)
                 * std::pow(100.0 * 0.10, -0.12)
                 * std::pow(4.0, 0.043)
                 * std::pow(0.6, -0.02);
    w.htKg = htlb * lb2kg;

    // Vertical tail (Raymer 15.49): 0.073*(W0*Nult)^0.376*q^0.122*S^0.876*
    //   (100tc)^-0.49*AR^0.357*taper^0.039
    double vtlb = 0.073 * std::pow(W0lb * nUlt, 0.376)
                 * std::pow(std::max(qCruise, 1.0), 0.122)
                 * std::pow(g.vtArea * FT2_PER_M2, 0.876)
                 * std::pow(100.0 * 0.10, -0.49)
                 * std::pow(1.8, 0.357)
                 * std::pow(0.6, 0.039);
    w.vtKg = vtlb * lb2kg;

    // Landing gear (Raymer 15.50/15.51): main 0.095*(Nl*W0)^0.768*Lm^0.409
    double gearLenFt = std::max(0.8 * g.fusMaxDiameter, 0.5) * FT_PER_M; // gear length approx
    double nl = 1.5 * 2; // ultimate landing load factor (2 main gear)
    double mainGearLb = 0.095 * std::pow(nl * W0lb, 0.768) * std::pow(gearLenFt, 0.409);
    double noseGearLb = 0.125 * std::pow(1.5 * W0lb, 0.566) * std::pow(gearLenFt * 0.8, 0.845);
    if (!in.retractGear) { mainGearLb *= (5.7 - 0.7) / 5.7; noseGearLb *= (5.7 - 0.7) / 5.7; }
    w.gearKg = (mainGearLb + noseGearLb) * lb2kg;

    // Engines installed (Raymer 15.54): 2.575 * massPerEngine^0.922 * N
    // mass per engine from selected engine db handled by caller (wi.engineMassKg)
    w.engineKg = 2.575 * std::pow(std::max(in.engineMassKg, 20.0) * LB_PER_KG, 0.922)
               * in.nEngines * lb2kg;

    // Fuel system (Raymer 15.55): 2.49 * Vgal^0.726 * (1+fint)^-0.363 * Ntanks^0.242 * Neng^0.157
    double volGal = in.fuelKg / 0.72 / 3.785; // avgas 0.72 kg/L
    double fuelSysLb = 2.49 * std::pow(std::max(volGal, 1.0), 0.726)
                     * std::pow(1.5, -0.363) * std::pow(2, 0.242)
                     * std::pow(std::max(1, in.nEngines), 0.157);
    w.fuelSystemKg = fuelSysLb * lb2kg;

    // Flight controls (Raymer 15.56): 0.053*L^1.536*b^0.371*(W0*Nult*1e-4)^0.80
    double fclb = 0.053 * std::pow(g.fusLength * FT_PER_M, 1.536)
                * std::pow(g.span * FT_PER_M, 0.371)
                * std::pow(W0lb * nUlt * 1e-4, 0.80);
    w.flightControlsKg = fclb * lb2kg;

    // Avionics installed (Raymer 15.57): 2.117 * Wav^0.933 (avionics ~ 40 lb light GA)
    w.avionicsKg = 2.117 * std::pow(40.0, 0.933) * lb2kg;

    // Electrical (Raymer 15.58): 12.57*(Wfsys+Wav)^0.51
    double elecLb = 12.57 * std::pow((fuelSysLb + 40.0), 0.51);
    w.electricalKg = elecLb * lb2kg;

    // Furnishings (Raymer 15.60): max(0.0582*W0 - 65 lb, 0)
    w.FurnishingsKg = std::max(0.0582 * W0lb - 65.0, 0.0) * lb2kg;

    // Paint & finish: ~0.015 kg per m^2 wetted (rule of thumb)
    w.paintKg = 0.015 * g.swetTotal * 0.35;

    w.totalEmptyKg = w.wingKg + w.fuselageKg + w.htKg + w.vtKg + w.engineKg
                   + w.gearKg + w.fuelSystemKg + w.flightControlsKg
                   + w.avionicsKg + w.electricalKg + w.FurnishingsKg + w.paintKg;
    w.buildupFrac = w.totalEmptyKg / std::max(in.w0Kg, 1.0);
    w.nUltimate = nUlt;
    w.table31Frac = raymerWeFrac(in.w0Kg, 2.05, -0.18); // GA single
    return w;
}
