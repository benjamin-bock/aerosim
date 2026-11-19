#include <cmath>
#include <cstdio>
#include <algorithm>
#include "../include/synthesis.h"
#include "../include/stability.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double G = 9.81;
static const double RHO0 = 1.225;

SynthesisResult synthesizeDesign(const DesignSpec& spec) {
    SynthesisResult res;
    NacaAirfoil foil(spec.airfoil);

    double w0 = 4.0 * (spec.payloadKg + spec.crewKg);
    double fuelKg = 0.10 * w0;
    double powerKW = 0.0;

    const int maxIter = 80;
    double relax = 0.35;
    // secant (Aitken) acceleration on the scalar fixed point w0 = F(w0),
    // with bracketing: if the residual sign flips we know the root is between
    // the last two iterates, so we bisect instead of overshooting again.
    bool havePrev = false;
    double prevW0 = 0.0, prevResidual = 0.0;
    double bracketLo = 0.0, bracketHi = 0.0;
    bool haveBracket = false;
    double bestW0 = 1e18;
    SynthesisResult bestRes;
    double lastW0 = w0, lastDelta = 0.0;
    for (int it = 0; it < maxIter; ++it) {
        // 1) Geometry from W0 and stall requirement
        double clmaxTO; // computed below from aero of previous pass; first pass estimate
        clmaxTO = res.aero.clmaxTakeoff > 0.1 ? res.aero.clmaxTakeoff : 1.7;
        double wsMaxStall = 0.5 * RHO0 * spec.stallSpeedMs * spec.stallSpeedMs * clmaxTO;
        // Choose wing loading: 85% of stall cap (margin) but reasonable range
        double ws = std::max(60.0, 0.85 * wsMaxStall);
        double S = w0 * G / ws;
        double ar = spec.aspectRatio;

        LayoutInputs li;
        li.payloadKg = spec.payloadKg;
        li.crewKg = spec.crewKg;
        li.nSeats = spec.nSeats;
        li.isJet = !spec.propDriven;
        li.htVolumeCoeff = spec.htVolumeCoeff;
        li.vtVolumeCoeff = spec.vtVolumeCoeff;
        Geometry geo = layoutAircraft(S, ar, spec.taper, spec.sweepDeg, foil, li);
        geo.retractGear = spec.retractGear;

        // 2) Aerodynamics
        AeroResult aero = aeroAnalysis(geo, foil, spec.cruiseSpeedMs, spec.cruiseAltM,
                                        spec.flap, spec.flapChordFrac, spec.flapSpanFrac,
                                        !spec.retractGear);

        // 3) Constraint set -> required power/thrust at this wing loading
        ConstraintSet cs;
        cs.vStall = spec.stallSpeedMs;
        cs.takeoffDist = spec.takeoffDistM;
        cs.obstacle = spec.obstacleM;
        cs.roc = spec.rocMs;
        cs.vClimb = spec.climbSpeedMs;
        cs.vMax = spec.maxSpeedMs;
        cs.vCruise = spec.cruiseSpeedMs;
        cs.altCruise = spec.cruiseAltM;
        cs.nTurn = spec.nTurn;
        cs.ceilingM = spec.ceilingM;
        cs.muRoll = spec.muRoll;
        cs.propEff = spec.propEff;
        cs.propDriven = spec.propDriven;
        cs.clmaxTO = aero.clmaxTakeoff;
        ConstraintAnalysis ca(aero, cs);

        double twReq = 0, pwReq = 0;
        const char* gov = "";
        bool feasible = ca.requirement(ws, twReq, pwReq, gov);
        double powerW = 0;
        if (spec.propDriven) {
            // pwReq is P/W in W per newton of weight -> multiply by weight (N)
            powerW = pwReq * w0 * G;
            powerKW = powerW / 1000.0;
        } else {
            powerW = twReq * w0 * G; // thrust N
            powerKW = powerW / 1000.0;
        }
        if (!feasible) { /* stall cap violated -> handled via ws choice */ }

        // 4) Fuel from Breguet with current L/D
        double q = 0.5 * isaDensity(spec.cruiseAltM) * spec.cruiseSpeedMs * spec.cruiseSpeedMs;
        double clCruise = (w0 * G) / (q * S);
        double ldCruise = clCruise / (aero.cd0 + aero.k * clCruise * clCruise);
        double sfcEngine = 0.30;
        int engIdx = -1;
        if (spec.propDriven) { engIdx = selectEngine(powerKW, 1.0); if (engIdx >= 0) sfcEngine = engineDatabase()[engIdx].sfcKgPerkWh; }
        double wfFrac;
        {
            double mission = 0.97 * 0.985 * 0.992 * 0.995;
            if (spec.propDriven) {
                double c = sfcEngine / 3.6e6; // kg/(W.s)
                double ldBest = aero.ldMax;
                double ld = std::max(ldCruise, 6.0); // actual cruise-point L/D (Breguet at constant V,CL)
                mission *= std::exp(-spec.rangeKm * 1000.0 * G * c / (spec.propEff * ld));
                if (spec.enduranceH > 0)
                    mission *= std::exp(-spec.enduranceH * 3600.0 * G * c
                                        * spec.cruiseSpeedMs / (spec.propEff * 0.866 * ldBest));
            } else {
                double cj = 0.55 / 3600.0;
                mission *= std::exp(-spec.rangeKm * 1000.0 * cj
                                    / (spec.cruiseSpeedMs * 0.866 * aero.ldMax));
                if (spec.enduranceH > 0)
                    mission *= std::exp(-spec.enduranceH * 3600.0 * cj / aero.ldMax);
            }
            wfFrac = (1.0 + spec.reserveFrac) * (1.0 - mission);
        }
        double fuelNew = wfFrac * w0;

        // 5) Component weights
        WeightInputs wi;
        wi.w0Kg = w0;
        wi.engineMassKg = engIdx >= 0 ? engineDatabase()[engIdx].massKg
                          : std::max(40.0, powerKW * 0.85);
        wi.fuelKg = fuelNew;
        wi.fuelInWingKg = 0.5 * fuelNew;
        wi.cruiseLD = std::max(ldCruise, 8.0);
        wi.cruiseDynamicPressure = q;
        wi.nSeats = spec.nSeats;
        wi.nEngines = spec.nEngines;
        wi.retractGear = spec.retractGear;
        wi.composite = spec.composite;
        wi.loadFactorUlt = 1.5 * spec.loadFactor;
        WeightResult wr = componentWeights(geo, wi);

        // 6) Convergence check on W0
        double w0New = wr.totalEmptyKg + spec.crewKg + spec.payloadKg + fuelNew;
        res.w0History.push_back(w0);
        if (!std::isfinite(w0New) || w0New <= 0.0) {
            // divergence guard: relax harder and reset to a sane value
            w0 = 0.5 * (w0 + std::max(100.0, 0.5 * w0));
            relax = std::max(0.2, relax * 0.5);
            continue;
        }
        double err = std::fabs(w0New - w0) / std::max(w0, 1.0);
        // fill result with this iteration's state BEFORE any bookkeeping
        fuelKg = fuelNew;
        res.wfKg = fuelNew;
        res.w0Kg = w0New;
        res.geo = geo;
        res.aero = aero;
        res.weights = wr;
        res.cset = cs;
        res.wingArea = S;
        res.span = geo.span;
        res.tw = twReq;
        res.pw = pwReq;
        res.powerKW = powerKW;
        res.engineIdx = engIdx;
        res.governingConstraint = gov;
        res.iterations = it + 1;

#ifdef SYNTH_DEBUG
        std::fprintf(stderr, "it%d w0=%.1f -> w0New=%.1f powerKW=%.1f fuel=%.1f gov=%s\n", it, w0, w0New, powerKW, fuelNew, gov ? gov : "?");
#endif
        if (w0New < bestW0) {
            bestW0 = w0New;
            bestRes = res;
        }
        if (err < 0.001) { res.converged = true; break; }

        // Divergence detection: accelerating growth means a runaway loop
        // (e.g. requirements physically impossible) — bail out with best iterate
        double delta = w0New - w0;
        if (it > 5 && w0New > 1.2 * w0 && delta > 1.1 * lastDelta) {
            res = bestRes;
            res.converged = false;
            res.iterations = it + 1;
            break;
        }
        lastDelta = delta;

        double residual = w0New - w0;
        double next;
        bool signFlip = havePrev && (residual > 0) != (prevResidual > 0)
                        && std::fabs(residual) > 1e-6;
        if (signFlip) {
            // root bracketed between prevW0 and w0
            if (!haveBracket) { bracketLo = std::min(prevW0, w0); bracketHi = std::max(prevW0, w0); haveBracket = true; }
            bracketLo = std::min(bracketLo, std::min(prevW0, w0));
            bracketHi = std::max(bracketHi, std::max(prevW0, w0));
            next = 0.5 * (bracketLo + bracketHi);
        } else if (havePrev && std::fabs(w0 - prevW0) > 1e-9) {
            double slope = (w0New - (prevW0 + prevResidual)) / (w0 - prevW0);
            if (slope > -0.5 && slope < 1.5 && std::fabs(slope - 1.0) > 0.05) {
                next = w0 - residual / (slope - 1.0);
                double step = next - w0;
                if (step > 0.15 * w0) next = w0 + 0.15 * w0;
                if (step < -0.15 * w0) next = w0 - 0.15 * w0;
                if (haveBracket) next = std::min(std::max(next, bracketLo), bracketHi);
            } else {
                next = relax * w0New + (1.0 - relax) * w0;
            }
        } else {
            next = relax * w0New + (1.0 - relax) * w0;
        }
        prevW0 = w0; prevResidual = residual; havePrev = true;
        w0 = std::max(50.0, next);
    }

    // On divergence we returned bestRes inside the loop; on convergence res is
    // already the final state. Sync the scalar summary from the stored fields.
    res.w0Kg = res.weights.totalEmptyKg + spec.crewKg + spec.payloadKg + res.wfKg;
    if (!std::isfinite(res.w0Kg) || res.w0Kg <= 0.0) res.w0Kg = w0;
    res.weKg = res.weights.totalEmptyKg;
    res.aspectRatio = spec.aspectRatio;

    // Final performance analysis with converged numbers
    PerformanceInputs pi;
    pi.w0Kg = w0;
    pi.fuelKg = fuelKg;
    pi.powerW = res.powerKW * 1000.0;
    pi.propEff = spec.propEff;
    pi.propDriven = spec.propDriven;
    pi.thrustN = res.tw * w0 * G;
    pi.cruiseAltM = spec.cruiseAltM;
    pi.cruiseSpeedMs = spec.cruiseSpeedMs;
    pi.obstacleM = spec.obstacleM;
    pi.muRoll = spec.muRoll;
    pi.muBrake = spec.muBrake;
    pi.clmaxTO = res.aero.clmaxTakeoff;
    pi.clmaxLanding = res.aero.clmaxLanding;
    if (spec.propDriven) {
        pi.usePropModel = true;
        pi.rpm = 2500;
        pi.prop.diameterM = Propeller::recommendedDiameter(res.powerKW, true);
        pi.prop.variablePitch = res.powerKW > 100;
    }
    if (spec.propDriven) {
        int idx = selectEngine(res.powerKW, 1.0);
        if (idx >= 0) {
            res.engineIdx = idx;
            pi.sfcKgPerkWh = engineDatabase()[idx].sfcKgPerkWh;
        }
    }
    res.perf = analyzePerformance(res.aero, res.geo, pi);

    // Static stability & CG — position the wing so CG lands at ~28% MAC
    StabilityInputs si;
    si.payloadKg = spec.payloadKg;
    si.crewKg = spec.crewKg;
    si.fuelKg = fuelKg;
    si.w0Kg = w0;
    si.htVolumeCoeff = spec.htVolumeCoeff;
    si.clAlphaWing = res.aero.clAlpha3D;
    // Bisection on wing station for a target static margin of 12% MAC.
    double lo = 0.15 * res.geo.fusLength, hi = 0.50 * res.geo.fusLength;
    double smTarget = 0.12;
    for (int k = 0; k < 25; ++k) {
        double mid = 0.5 * (lo + hi);
        si.xWingRootM = mid;
        res.stab = stabilityAnalysis(res.geo, res.weights, si);
        // static margin increases with wing station (np moves faster than cg)
        if (res.stab.staticMargin < smTarget) lo = mid; else hi = mid;
    }
    si.xWingRootM = 0.5 * (lo + hi);
    res.stab = stabilityAnalysis(res.geo, res.weights, si);
    res.stab.wingRootM = si.xWingRootM;
    return res;
}
