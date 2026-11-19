#ifndef GEOMETRY_H
#define GEOMETRY_H

/**
 * @file geometry.h
 * @brief Airframe geometry layout: fuselage, wing, tails, wetted areas.
 * Methods follow Raymer ch. 4-7 (tail volume coefficients, Swet estimates).
 */
#include "naca.h"

struct Geometry {
    // Wing
    double wingArea = 10.0;       // S (m^2)
    double span = 9.0;            // b (m)
    double aspectRatio = 8.0;
    double taper = 0.5;
    double sweepQC = 0.0;         // quarter-chord sweep (rad)
    double mac = 1.25;            // mean aerodynamic chord (m)
    double thicknessRatio = 0.12;

    // Fuselage
    double fusLength = 7.0;       // (m)
    double fusMaxDiameter = 1.1;  // (m)
    double fusFineness = 7.0;     // l/d

    // Tails (from volume coefficients)
    double htArea = 1.5;          // horizontal tail (m^2)
    double htArm = 4.0;           // tail moment arm (m)
    double vtArea = 1.0;          // vertical tail (m^2)
    double vtArm = 4.0;

    // Wetted areas
    double swetWing = 0;
    double swetFus = 0;
    double swetHT = 0;
    double swetVT = 0;
    double swetNacelle = 0;
    double swetTotal = 0;

    // Landing gear
    bool retractGear = false;

    void computeFromLayout();
};

struct LayoutInputs {
    double payloadKg = 180.0;
    double crewKg = 80.0;
    int nSeats = 2;
    bool isJet = false;
    double htVolumeCoeff = 0.7;   // Raymer Table 6.4: GA 0.3-0.7... prop GA ~0.48
    double vtVolumeCoeff = 0.04;
    double htAspectRatio = 4.0;
    double vtAspectRatio = 1.8;
};

Geometry layoutAircraft(double wingArea, double aspectRatio, double taper,
                        double sweepDeg, const NacaAirfoil& foil,
                        const LayoutInputs& in);

#endif
