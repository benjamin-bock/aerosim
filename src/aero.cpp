#include <cmath>
#include <vector>
#include <algorithm>
#include "../include/aero.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double flapDeltaCLmax(FlapType t, double chordFrac, double spanFrac) {
    double base;
    switch (t) {
        case FlapType::None:       base = 0.00; break;
        case FlapType::Plain:      base = 0.90; break;
        case FlapType::Split:      base = 0.95; break;
        case FlapType::SingleSlot: base = 1.30; break;
        case FlapType::Fowler:     base = 1.60; break;
        case FlapType::DoubleSlot: base = 2.00; break;
        default: base = 0.0;
    }
    // scale by flap chord/span fraction (Raymer ch.12 high-lift discussion)
    return base * std::pow(std::max(0.1, chordFrac) / 0.30, 0.5)
               * std::max(0.2, spanFrac);
}

double flapDeltaCd(FlapType t) {
    switch (t) {
        case FlapType::None:       return 0.000;
        case FlapType::Plain:      return 0.010;
        case FlapType::Split:      return 0.020;
        case FlapType::SingleSlot: return 0.018;
        case FlapType::Fowler:      return 0.030;
        case FlapType::DoubleSlot: return 0.035;
    }
    return 0.0;
}

double clmaxSweepCorrected(double clmax2d, double sweepQC) {
    return clmax2d * std::cos(sweepQC);
}

double gearDragArea(bool retract, double w0N) {
    if (retract) return 0.0;
    // Raymer: fixed gear drag area ~ 0.0031 ft^2 per lb... in m^2: f ~ 1.0e-4 * W(N)/... use simpler:
    // typical fixed gear f = 0.5 m^2 * (W/10000N)
    return 0.35 * w0N / 10000.0;
}

AeroResult aeroAnalysis(const Geometry& g, const NacaAirfoil& foil,
                        double vCruise, double altCruise,
                        FlapType flap, double flapChordFrac, double flapSpanFrac,
                        bool fixedGear) {
    AeroResult a;

    Atmosphere atm = isaAt(altCruise);
    double v = std::max(vCruise, 10.0);
    double mac = std::max(g.mac, 0.3);
    double re = atm.density * v * mac / atm.viscosity;

    // Turbulent flat-plate skin friction (Raymer 12.27 cutoff method simplified)
    double cf = 0.455 / std::pow(std::log10(re), 2.58) * std::pow(1.0 + 0.144 * 0.3 * 0.3, -0.65);

    std::vector<ComponentDrag> parts;
    auto add = [&](const char* n, double swet, double ff, double q) {
        ComponentDrag c{n, swet, cf, ff, q, 0.0};
        c.cdContrib = cf * ff * q * swet / g.wingArea;
        parts.push_back(c);
    };

    // Wing form factor (Raymer 12.36)
    double tcSweep = g.thicknessRatio / std::cos(g.sweepQC);
    double ffWing = 1.0 + 2.0 * tcSweep + 60.0 * std::pow(tcSweep, 4);
    double qWing = 1.0;
    add("wing", g.swetWing, ffWing, qWing);

    // Fuselage form factor (Raymer 12.32): FF = 1 + 60/f^3 + f/400
    double f = std::max(g.fusFineness, 1.5);
    double ffFus = (1.0 + 60.0 / std::pow(f, 3.0) + f / 400.0);
    ffFus *= 1.0 + 0.15 * std::pow(std::max(0.0, g.fusFineness - 6.0) / 4.0, 2.0); // upsweep/canopy penalty
    add("fuselage", g.swetFus, ffFus, 1.0);

    // Tails: like wing with t/c = 0.10
    double ffTail = 1.0 + 2.0 * 0.10 + 60.0 * std::pow(0.10, 4);
    add("htail", g.swetHT, ffTail, 1.0);
    add("vtail", g.swetVT, ffTail, 1.0);

    double cd0 = 0;
    for (auto& p : parts) cd0 += p.cdContrib;

    // Miscellaneous: antennas, gaps, protrusions (Raymer: +2-5%)
    cd0 *= 1.03;
    // Nacelle/engine cooling & misc drag
    cd0 += 0.003;
    // Fixed landing gear (Raymer 12.30): drag AREA f_gear ∝ sqrt(W) (tire/strut
    // size grows slowly with weight), so CD0_gear = f/S decays like W^-0.5/S.
    // For a GA wing loading band this is ~0.006-0.010; use sqrt(S) form but
    // normalized to stay bounded: CD0_gear = 0.006 * (S/16)^-0.15.
    if (fixedGear)
        cd0 += 0.0065 * std::pow(std::max(g.wingArea, 4.0) / 16.0, -0.15);
    // Wing struts & cabane (braced high-wing GA)
    cd0 += 0.0025;
    // Cowling/engine-cooling exit, wheel fairing imperfections
    cd0 += 0.0010;
    // Compressibility (Mdd based)
    double mach = v / atm.soundSpeed;
    double mdd = 0.93 * (1.0 / std::sqrt(std::cos(g.sweepQC))) - 0.032 * g.thicknessRatio / std::cos(g.sweepQC) - 0.10; // Korn
    double mcrit = std::max(0.3, mdd - 0.07);
    if (mach > mcrit)
        a.compressibilityRise = 20.0 * std::pow(mach - mcrit, 3.0);
    cd0 += a.compressibilityRise;

    a.cd0 = cd0;

    // Oswald: straight wing Raymer 12.48
    double ar = g.aspectRatio;
    if (g.sweepQC < 0.1)
        a.oswald = 1.78 * (1.0 - 0.045 * std::pow(ar, 0.68)) - 0.64;
    else // swept (Raymer 12.47)
        a.oswald = 4.61 * (1.0 - 0.045 * std::pow(ar, 0.68))
                     * std::pow(std::cos(g.sweepQC), 0.15) - 3.1;
    a.oswald = std::max(0.55, std::min(0.95, a.oswald));
    a.k = 1.0 / (M_PI * ar * a.oswald);
    a.ldMax = 0.5 * std::sqrt(1.0 / (cd0 * a.k));

    // CLmax: 2D section (sweep-corrected) * 3D finite-AR factor, + flaps
    double cl2d = foil.clMax();
    double clmaxClean = clmaxSweepCorrected(cl2d, g.sweepQC)
                          * std::max(0.72, std::min(0.95, 0.95 - 0.02 * std::max(0.0, 10.0 - ar)));
    a.clmaxClean = clmaxClean;
    double dCl = flapDeltaCLmax(flap, flapChordFrac, flapSpanFrac);
    a.clmaxTakeoff = clmaxClean + 0.55 * dCl;   // partial deflection
    a.clmaxLanding = clmaxClean + dCl;

    // 3D lift slope (DATCOM): CLa = 2*pi*AR / (2 + sqrt(AR^2*(1+tan^2)-4))
    double t = std::tan(g.sweepQC / 2.0);
    double denom = 2.0 + std::sqrt(std::max(1.0, ar * ar * (1 + t * t) - 4.0));
    double clAlphaSub = 2.0 * M_PI * ar / denom;
    double prandtl = std::sqrt(1.0 - std::pow(mach, 2.0));
    a.clAlpha3D = clAlphaSub / std::max(prandtl, 0.5);
    a.alpha0 = foil.alpha0();
    a.cl0 = a.clAlpha3D * (-a.alpha0);

    a.cdCruise = cd0; // + lift-dependent evaluated by caller
    return a;
}
