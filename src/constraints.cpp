#include <cmath>
#include <vector>
#include <algorithm>
#include "../include/constraints.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double G = 9.81;
static const double RHO0 = 1.225;

ConstraintAnalysis::ConstraintAnalysis(const AeroResult& a, const ConstraintSet& s)
    : a_(a), s_(s), atmSL_(isaAt(0.0)), atmCruise_(isaAt(s.altCruise)) {}

double ConstraintAnalysis::dragAt(double ws, double v, double rho, double nLoad) const {
    // drag per unit weight (dimensionless D/W) at flight condition
    // q*S/W = 0.5*rho*V^2/(W/S)
    double qSoverW = 0.5 * rho * v * v / ws;
    double cl = nLoad / qSoverW;
    return qSoverW * (a_.cd0 + a_.k * cl * cl);  // D/W
}

double ConstraintAnalysis::thrustRequiredJet(ConstraintKind k, double ws) const {
    switch (k) {
        case ConstraintKind::Climb: {
            double v = s_.vClimb;
            double gamma = std::asin(std::min(0.9, s_.roc / v));
            return std::sin(gamma) + dragAt(ws, v, RHO0, 1.0);
        }
        case ConstraintKind::Cruise: {
            double rho = atmCruise_.density;
            return dragAt(ws, s_.vCruise, rho, 1.0);
        }
        case ConstraintKind::MaxSpeed:
            return dragAt(ws, s_.vMax, RHO0, 1.0);
        case ConstraintKind::Turn: {
            if (s_.nTurn <= 1.0) return 0.0;
            double v = s_.vMax * 0.7;
            return dragAt(ws, v, RHO0, s_.nTurn) + 0.0; // sustained: T = D(n)
        }
        case ConstraintKind::Ceiling: {
            if (s_.ceilingM <= 0) return 0.0;
            double rho = isaDensity(s_.ceilingM);
            double v = s_.vClimb; // climb speed near ceiling
            double roc = 0.508;  // 100 ft/min
            double gamma = std::asin(roc / v);
            return std::sin(gamma) + dragAt(ws, v, rho, 1.0);
        }
        case ConstraintKind::Takeoff:
            return thrustTakeoff(ws);
        default:
            return 0.0;
    }
}

double ConstraintAnalysis::powerRequiredProp(ConstraintKind k, double ws) const {
    // P/W = T/W * V / eta evaluated at the condition speed
    double v;
    switch (k) {
        case ConstraintKind::Climb:   v = s_.vClimb;  break;
        case ConstraintKind::Cruise:  v = s_.vCruise; break;
        case ConstraintKind::MaxSpeed:v = s_.vMax;   break;
        case ConstraintKind::Turn:    v = s_.vMax * 0.7; break;
        case ConstraintKind::Ceiling: v = s_.vClimb; break;
        case ConstraintKind::Takeoff: return powerTakeoff(ws);
        default: return 0.0;
    }
    double tw = thrustRequiredJet(k, ws);
    return tw * v / s_.propEff;
}

double ConstraintAnalysis::thrustTakeoff(double ws) const {
    // Raymer 17.86-style parametric + obstacle energy
    double clTO = s_.clmaxTO;
    double vTO = 1.1 * std::sqrt(2.0 * ws / (RHO0 * clTO)); // rotate speed
    double qLO = 0.5 * RHO0 * vTO * vTO;
    double cl = clTO * 0.9;
    double dOverW = (qLO / ws) * (a_.cd0 + a_.k * cl * cl);
    double sGround = std::max(0.55 * s_.takeoffDist, 30.0);
    double gammaObs = std::atan2(s_.obstacle, std::max(0.55 * s_.takeoffDist, 1.0));
    // ground roll mean (using average speed vMean)
    double vMean = 0.7 * vTO;
    double qMean = 0.5 * RHO0 * vMean * vMean;
    double dMeanOverW = (qMean / ws) * (a_.cd0 + a_.k * cl * cl);
    double fricOverW = s_.muRoll * 0.5; // mean lift ~ 50% W
    double accelNeeded = 0.5 * vTO * vTO / sGround;      // m/s^2 (mean)
    double twRoll = accelNeeded / G + fricOverW + dMeanOverW;
    double twObs = std::sin(gammaObs) + dOverW;
    return std::max(twRoll, twObs);
}

double ConstraintAnalysis::powerTakeoff(double ws) const {
    double tw = thrustTakeoff(ws);
    double clTO = s_.clmaxTO;
    double vTO = 1.1 * std::sqrt(2.0 * ws / (RHO0 * clTO));
    double vMean = 0.7 * vTO;
    return tw * vMean / s_.propEff;
}

std::vector<ConstraintLine> ConstraintAnalysis::compute(double wsMin, double wsMax, int n) const {
    std::vector<ConstraintLine> lines;
    struct Def { ConstraintKind k; const char* label; bool line; };
    const Def defs[] = {
        {ConstraintKind::Takeoff,  "takeoff",  true},
        {ConstraintKind::Climb,    "climb",    true},
        {ConstraintKind::Cruise,   "cruise",   true},
        {ConstraintKind::MaxSpeed, "max speed",true},
        {ConstraintKind::Turn,     "turn",     s_.nTurn > 1.0},
        {ConstraintKind::Ceiling,  "ceiling",  s_.ceilingM > 0},
    };
    for (const Def& d : defs) {
        if (d.k == ConstraintKind::Turn && s_.nTurn <= 1.0) continue;
        if (d.k == ConstraintKind::Ceiling && s_.ceilingM <= 0) continue;
        ConstraintLine l;
        l.kind = d.k;
        l.label = d.label;
        for (int i = 0; i <= n; ++i) {
            double ws = wsMin + (wsMax - wsMin) * i / n;
            l.ws.push_back(ws);
            l.tw.push_back(thrustRequiredJet(d.k, ws));
            l.pw.push_back(powerRequiredProp(d.k, ws));
        }
        lines.push_back(l);
    }
    // Stall cap
    ConstraintLine stall;
    stall.kind = ConstraintKind::Stall;
    stall.label = "stall";
    stall.isUpperBoundOnWs = true;
    stall.wsLimit = 0.5 * RHO0 * s_.vStall * s_.vStall * s_.clmaxTO;
    lines.push_back(stall);
    return lines;
}

bool ConstraintAnalysis::requirement(double ws, double& tw, double& pw, const char*& govLabel) const {
    double wsStallMax = 0.5 * RHO0 * s_.vStall * s_.vStall * s_.clmaxTO;
    if (ws > wsStallMax) {
        govLabel = "stall";
        tw = pw = 0;
        return false;
    }
    struct Def { ConstraintKind k; const char* label; };
    const Def defs[] = {
        {ConstraintKind::Takeoff, "takeoff"},
        {ConstraintKind::Climb, "climb"},
        {ConstraintKind::Cruise, "cruise"},
        {ConstraintKind::MaxSpeed, "max speed"},
        {ConstraintKind::Turn, "turn"},
        {ConstraintKind::Ceiling, "ceiling"},
    };
    double best = 0;
    pw = tw = 0;
    for (const Def& d : defs) {
        if (d.k == ConstraintKind::Turn && s_.nTurn <= 1.0) continue;
        if (d.k == ConstraintKind::Ceiling && s_.ceilingM <= 0) continue;
        double t = thrustRequiredJet(d.k, ws);
        double p = powerRequiredProp(d.k, ws);
        double measure = s_.propDriven ? p : t;
        if (measure > best) {
            best = measure;
            tw = t;
            pw = p;
            govLabel = d.label;
        }
    }
    return true;
}
