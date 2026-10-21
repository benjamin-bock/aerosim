#ifndef REQUIREMENTS2_H
#define REQUIREMENTS2_H

/**
 * @file requirements2.h
 * @brief Top-level design specification ("the guesses").
 */
#include <string>
#include "aero.h"

struct DesignSpec {
    // Mission
    double payloadKg = 180;
    double crewKg = 80;
    double rangeKm = 1000;
    double cruiseSpeedMs = 70;
    double cruiseAltM = 2500;
    double enduranceH = 0;
    double reserveFrac = 0.06;

    // Performance
    double stallSpeedMs = 22;
    double takeoffDistM = 400;
    double obstacleM = 15.2;
    double rocMs = 5;
    double climbSpeedMs = 40;
    double maxSpeedMs = 90;
    double nTurn = 0;           // 0 = skip
    double ceilingM = 0;         // 0 = skip
    double loadFactor = 3.8;

    // Configuration
    std::string airfoil = "2412";
    double aspectRatio = 8;
    double taper = 0.5;
    double sweepDeg = 0;
    FlapType flap = FlapType::Plain;
    double flapChordFrac = 0.25;
    double flapSpanFrac = 0.6;
    bool propDriven = true;
    double propEff = 0.8;
    bool retractGear = false;
    bool composite = false;
    int nSeats = 2;
    int nEngines = 1;
    double htVolumeCoeff = 0.7;
    double vtVolumeCoeff = 0.04;
    double muRoll = 0.04;
    double muBrake = 0.5;
};

#endif
