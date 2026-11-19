#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

/**
 * @file constraints.h
 * @brief Constraint analysis (Raymer ch. 5.3 / 18.4): for each requirement,
 * required T/W (jet) or P/W (prop) as a function of wing loading W/S.
 */
#include <vector>
#include "aero.h"

enum class ConstraintKind {
    Stall,        // caps W/S (no T/W line)
    Takeoff,      // ground roll + obstacle
    Climb,        // specified ROC at speed
    Cruise,       // level flight at speed/alt
    MaxSpeed,     // max level speed
    Turn,         // sustained turn load factor
    Ceiling       // service ceiling ROC capability
};

struct ConstraintLine {
    ConstraintKind kind;
    const char* label;
    std::vector<double> ws;       // N/m^2
    std::vector<double> tw;       // required T/W (jet convention)
    std::vector<double> pw;        // required P/W (W per N of weight, prop)
    bool isUpperBoundOnWs = false; // stall
    double wsLimit = 0;             // for stall
};

struct ConstraintSet {
    double rhoSL;          // density at eval altitude for performance items
    double altitude = 0;
    double vStall = 22, vTO = 30, vClimb = 40, vMax = 90, vCruise = 70;
    double roc = 5;         // m/s
    double altCruise = 2500;
    double nTurn = 0;       // sustained load factor (0 = skip)
    double ceilingM = 0;    // service ceiling requirement (0 = skip)
    double takeoffDist = 400, obstacle = 15.2;
    double muRoll = 0.04;
    double propEff = 0.8;
    bool propDriven = true;
    double clmaxTO = 1.9;
};

class ConstraintAnalysis {
public:
    ConstraintAnalysis(const AeroResult& aero, const ConstraintSet& set);
    // Compute all constraint lines over [wsMin, wsMax] with n points
    std::vector<ConstraintLine> compute(double wsMin, double wsMax, int n) const;

    // For a candidate W/S: the max required T/W (or P/W) across constraints
    // ignoring the stall cap; returns false if W/S violates stall cap.
    bool requirement(double ws, double& tw, double& pw, const char*& govLabel) const;

    const AeroResult& aero() const { return a_; }
    const ConstraintSet& set() const { return s_; }

private:
    AeroResult a_;
    ConstraintSet s_;
    Atmosphere atmSL_;
    Atmosphere atmCruise_;

    double thrustRequiredJet(ConstraintKind k, double ws) const;
    double powerRequiredProp(ConstraintKind k, double ws) const;
    // generic drag at a flight condition
    double dragAt(double ws, double v, double rho, double nLoad = 1.0) const;
    // takeoff: ground roll + obstacle via energy model
    double thrustTakeoff(double ws) const;
    double powerTakeoff(double ws) const;
};

#endif
