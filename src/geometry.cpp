#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>
#include <algorithm>
#include "../include/geometry.h"

void Geometry::computeFromLayout() {
    // MAC = (2/3) * C_root * (1 + l + l^2)/(1 + l); C_root = 2S/(b*(1+l))
    double cRoot = 2.0 * wingArea / (std::max(span, 0.1) * (1.0 + std::max(taper, 0.05)));
    mac = (2.0 / 3.0) * cRoot * (1 + taper + taper * taper) / (1 + taper);
    // Wing wetted area: ~2.02 * exposed planform (both sides), no fuselage buried
    double exposedFrac = std::max(0.55, 1.0 - fusMaxDiameter / std::max(span, 1.0));
    double exposedArea = wingArea * exposedFrac;
    double tc = std::max(0.06, thicknessRatio);
    // Raymer: Swet_wing ~ 2.0*S_exposed*(1 + 0.25*t/c) for straight wings
    swetWing = 2.0 * exposedArea * (1.0 + 0.25 * tc);
    // Fuselage: cylinder + nose/tail approximation -> Swet ~ pi*d*l*(fineness correction)
    swetFus = M_PI * fusMaxDiameter * fusLength * 0.90; // 10% cut for nose taper
    // Tails
    swetHT = 2.0 * htArea * (1.0 + 0.25 * 0.10);
    swetVT = 2.0 * vtArea * (1.0 + 0.25 * 0.10);
    swetTotal = swetWing + swetFus + swetHT + swetVT + swetNacelle;
}

Geometry layoutAircraft(double wingArea, double aspectRatio, double taper,
                        double sweepDeg, const NacaAirfoil& foil,
                        const LayoutInputs& in) {
    Geometry g;
    g.wingArea = wingArea;
    g.aspectRatio = aspectRatio;
    g.taper = taper;
    g.sweepQC = sweepDeg * M_PI / 180.0;
    g.span = std::sqrt(aspectRatio * wingArea);
    g.thicknessRatio = foil.tc();

    // Fuselage sized from payload/crew: seats side-by-side, cabin + engine + tail
    int seats = std::max(1, in.nSeats);
    g.fusMaxDiameter = 0.55 + 0.25 * (seats > 2 ? 2 : seats > 1 ? 1 : 0);
    if (in.isJet) g.fusMaxDiameter += 0.15;
    g.fusLength = std::max(4.5, 1.4 + 1.1 * seats + wingArea / 12.0);
    g.fusFineness = g.fusLength / g.fusMaxDiameter;

    // Tail moment arm: ~60% of fuselage length
    g.htArm = 0.60 * g.fusLength;
    g.vtArm = g.htArm;

    // Tail areas from volume coefficients: S_ht = (V_h * S * MAC) / l_h
    double cRoot = 2.0 * wingArea / (std::max(g.span, 0.1) * (1.0 + std::max(taper, 0.05)));
    g.mac = (2.0 / 3.0) * cRoot * (1 + taper + taper * taper) / (1 + taper);
    g.htArea = std::max(0.35, in.htVolumeCoeff * wingArea * g.mac / g.htArm);
    g.vtArea = std::max(0.25, in.vtVolumeCoeff * wingArea * g.span / g.vtArm);

    g.retractGear = false;
    g.computeFromLayout();
    return g;
}
