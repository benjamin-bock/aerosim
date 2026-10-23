#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>
#include "../include/stability.h"

StabilityResult stabilityAnalysis(const Geometry& g, const WeightResult& w,
                                  const StabilityInputs& in) {
    StabilityResult r;
    double fusL = g.fusLength;

    // Component stations (fraction of fuselage length, conceptual layout):
    // nose 0 -> tail 1
    auto add = [&](const char* n, double kg, double frac) {
        r.items.push_back({n, kg, frac * fusL});
    };
    double xWingFrac = in.xWingRootM / std::max(fusL, 0.1);
    add("wing", w.wingKg, xWingFrac + 0.12); // wing structure CG ~ near MAC/4 of root
    add("fuselage", w.fuselageKg, 0.42);   // distributed structure, CG mid
    add("htail", w.htKg, 0.92);
    add("vtail", w.vtKg, 0.92);
    add("engine", w.engineKg, 0.12);        // nose engine
    add("gear", w.gearKg, 0.16);
    add("fuel system", w.fuelSystemKg, 0.32);
    add("flight controls", w.flightControlsKg, 0.45);
    add("avionics", w.avionicsKg, 0.10);
    add("electrical", w.electricalKg, 0.25);
    add("furnishings", w.FurnishingsKg, 0.35);
    add("paint", w.paintKg, 0.45);
    // occupants & baggage: crew at 30%, payload at 38% (cabin)
    r.items.push_back({"crew", in.crewKg, 0.32 * fusL});
    r.items.push_back({"payload", in.payloadKg, 0.40 * fusL});
    // fuel at wing CG
    r.items.push_back({"fuel", in.fuelKg, (xWingFrac + 0.12) * fusL});

    double sumMx = 0, sumM = 0;
    for (const auto& it : r.items) {
        sumMx += it.massKg * it.xM;
        sumM += it.massKg;
    }
    r.cgM = sumMx / std::max(sumM, 1.0);

    // Neutral point (strip / volume-coefficient method, Raymer ch.16):
    // x_np = x_ac_wing + (CLa_t/CLa_w) * Vh * (dE/dA ~ 0.9) * MAC
    double mac = g.mac;
    double xWingLE = in.xWingRootM;
    double xAcWing = xWingLE + in.acFracWing * mac;
    double vh = g.htArea * g.htArm / (g.wingArea * mac);
    // downwash gradient at tail: dEps/dA ~ 2/(pi*AR)
    double dEpsDAlpha = std::min(0.35, 2.0 / (M_PI * std::max(g.aspectRatio, 1.0)));
    double npShift = in.htLiftSlopeRatio * vh * (1.0 - dEpsDAlpha) * mac;
    r.npM = xAcWing + npShift;

    r.staticMargin = (r.npM - r.cgM) / std::max(mac, 0.1);

    // Forward/aft excursions: no payload & full fuel vs full payload
    {
        double fwd = 0, fwdM = 0, aft = 0, aftM = 0;
        for (const auto& it : r.items) {
            if (it.name != std::string("payload")) {
                fwd += it.massKg; fwdM += it.massKg * it.xM;
                aft += it.massKg; aftM += it.massKg * it.xM;
            }
        }
        fwd += in.crewKg * 0 + 0; // crew always aboard
        fwd += in.payloadKg * 0;  // no payload for fwd case
        aft += in.payloadKg * (0.40 * fusL);
        r.cgFwdM = fwdM / std::max(fwd, 1.0);
        r.cgAftM = aftM / std::max(aft, 1.0);
        r.staticMarginFwd = (r.npM - r.cgFwdM) / std::max(mac, 0.1);
        r.staticMarginAft = (r.npM - r.cgAftM) / std::max(mac, 0.1);
    }

    // Acceptable: static margin between 5% and 30% MAC (GA practice)
    r.cgWithinLimits = (r.staticMargin > 0.05 && r.staticMargin < 0.30);
    return r;
}
