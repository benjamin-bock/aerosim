#include <cmath>
#include <algorithm>
#include "../include/performance.h"
#include "../include/atmosphere.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double G = 9.81;
static const double RHO0 = 1.225;

// Shaft power available (before propeller): NA piston scales ~ sigma
static double shaftPower(const PerformanceInputs& in, double altM) {
    double sigma = isaDensity(altM) / RHO0;
    return in.powerW * std::max(0.25, sigma);
}

// Thrust available: constant-eta model or propeller eta = f(J) model
static double thrustAvailable(const PerformanceInputs& in, double v, double altM) {
    double sigma = isaDensity(altM) / RHO0;
    if (in.propDriven) {
        double pAlt = shaftPower(in, altM);
        if (in.usePropModel) {
            double n = std::max(in.rpm / 60.0, 5.0);
            double J = std::max(v, 0.1) / (n * in.prop.diameterM);
            double eta = in.prop.efficiency(J);
            if (v < 8.0) return std::min(in.prop.staticThrust(pAlt), pAlt / 5.0);
            return pAlt * eta / v;
        }
        double vUse = std::max(v, 12.0);
        return pAlt * in.propEff / vUse;
    }
    return in.thrustN * std::max(0.3, std::pow(sigma, 0.7));
}

PerformanceResult analyzePerformance(const AeroResult& a, const Geometry& g,
                                     const PerformanceInputs& in) {
    PerformanceResult r;
    double W = in.w0Kg * G;
    double S = g.wingArea;

    // Stall speeds
    double ws = W / S;
    r.vStallClean = std::sqrt(2.0 * ws / (RHO0 * a.clmaxClean));
    r.vStallLanding = std::sqrt(2.0 * ws / (RHO0 * in.clmaxLanding));

    // Best climb speed & ROC at SL: maximize (P_avail - P_req)*V / W
    double bestROC = -1e9, vBest = 1.0;
    for (double v = std::max(r.vStallClean * 1.05, 15.0); v < 120.0; v += 0.5) {
        double q = 0.5 * RHO0 * v * v;
        double cl = W / (q * S);
        if (cl > a.clmaxClean) continue;
        double cd = a.cd0 + a.k * cl * cl;
        double d = q * S * cd;
        double tAvail = thrustAvailable(in, v, 0.0);
        double excess = tAvail - d;
        double roc = excess * v / W;
        if (roc > bestROC) { bestROC = roc; vBest = v; }
    }
    r.rocSLms = std::max(bestROC, 0.0);
    r.vBestClimb = vBest;

    // Max level speed at SL: solve T_avail(V) = D(V)
    double vMax = 0;
    for (double v = 20.0; v < 250.0; v += 0.5) {
        double q = 0.5 * RHO0 * v * v;
        double cl = W / (q * S);
        double cd = a.cd0 + a.k * cl * cl;
        double d = q * S * cd;
        if (thrustAvailable(in, v, 0.0) >= d) vMax = v; else break;
    }
    r.vMaxLevel = vMax;

    // Service ceiling: find altitude where ROC drops to 100 ft/min (0.508 m/s)
    double alt = 0, rocAt = 1e9;
    for (alt = 0; alt <= 15000.0; alt += 50.0) {
        double rho = isaDensity(alt);
        double v = vBest; // approx constant Vbest
        double q = 0.5 * rho * v * v;
        double cl = W / (q * S);
        double cd = a.cd0 + a.k * cl * cl;
        double d = q * S * cd;
        double tAvail = thrustAvailable(in, v, alt);
        rocAt = (tAvail - d) * v / W;
        if (rocAt <= 0.508) break;
    }
    r.ceilingM = alt;

    // Range & endurance (Breguet)
    if (in.propDriven) {
        // range (km): R = eta/c * (L/D) * ln(W0/W1) * (1/g)
        double c = in.sfcKgPerkWh / 3.6e6; // kg/kWh -> kg/(W.s)
        double q = 0.5 * isaDensity(in.cruiseAltM) * in.cruiseSpeedMs * in.cruiseSpeedMs;
        double cl = W / (q * S);
        double cd = a.cd0 + a.k * cl * cl;
        double ld = cl / cd;
        double w1 = W - in.fuelKg * G * 0.94; // 6% reserve/trapped
        double rangeM = (in.propEff / (c * G)) * ld * std::log(W / std::max(w1, 0.2 * W));
        r.rangeKm = rangeM / 1000.0;
        r.enduranceH = rangeM / std::max(in.cruiseSpeedMs, 1.0) / 3600.0;
    } else {
        double cj = in.jetTsfc / 3600.0;
        double q = 0.5 * isaDensity(in.cruiseAltM) * in.cruiseSpeedMs * in.cruiseSpeedMs;
        double cl = W / (q * S);
        double cd = a.cd0 + a.k * cl * cl;
        double ld = cl / cd;
        double w1 = W - in.fuelKg * G * 0.94;
        double rangeM = (in.cruiseSpeedMs / (cj * G)) * ld * std::log(W / std::max(w1, 0.2 * W));
        r.rangeKm = rangeM / 1000.0;
        r.enduranceH = rangeM / in.cruiseSpeedMs / 3600.0;
    }
    r.cruiseSpeedKmh = in.cruiseSpeedMs * 3.6;

    // Takeoff distance to obstacle (prop)
    {
        double clTO = in.clmaxTO;
        double vLO = 1.1 * std::sqrt(2.0 * ws / (RHO0 * clTO));
        double qLO = 0.5 * RHO0 * vLO * vLO;
        double cl = clTO * 0.9;
        double cd = a.cd0 + 0.01 + a.k * cl * cl; // flaps partial
        double d = qLO * S * cd;
        // Numerical integration of ground roll with T = P*eta/V
        double v = 0.5, x = 0, dt = 0.02;
        double m = in.w0Kg;
        int steps = 0, maxSteps = 200000;
        bool reached = false;
        while (v < vLO && x < 5000.0 && steps < maxSteps) {
            double tAvail = thrustAvailable(in, std::max(v, 5.0), 0.0);
            double q = 0.5 * RHO0 * v * v;
            double lift = q * S * cl;
            double dGround = q * S * cd;
            double fric = in.muRoll * std::max(W - lift, 0.0);
            double acc = (tAvail - dGround - fric) / m;
            if (acc <= 0.0) break; // underpowered: cannot accelerate
            v += acc * dt;
            x += v * dt;
            ++steps;
        }
        reached = v >= vLO;
        if (!reached) { r.takeoffDistM = 1e9; } // cannot take off with this power
        // climb to obstacle: gamma from excess thrust at vLO
        double tAvailLO = thrustAvailable(in, vLO, 0.0);
        double gamma = std::max(0.02, (tAvailLO - d) / W);
        double sClimb = in.obstacleM / std::tan(std::asin(std::min(gamma, 0.9)));
        r.takeoffDistM = x + sClimb;
    }

    // Landing distance: approach over obstacle + flare + ground roll w/ braking
    {
        double vApp = 1.3 * r.vStallLanding;
        double gammaApp = std::atan2(3.0, std::max(vApp, 1.0)); // 3 deg glideslope
        double sApproach = in.obstacleM / std::tan(gammaApp);
        // ground roll: v^2/(2*a), a = mu_brake*g (conservative, no reverse)
        double roll = vApp * vApp / (2.0 * in.muBrake * G * 0.60); // 60% braking efficiency
        r.landingDistM = sApproach + roll;
    }

    // V-n diagram corner points (FAR-23-like: nmax = 3.8 or 4.4, nmin = -0.4*nmax)
    r.nMaxPos = 3.8;
    r.nMaxNeg = -0.4 * 3.8;
    double q = 0.5 * RHO0 * r.vMaxLevel * r.vMaxLevel;
    // VC ~ 0.8*VD (cruise), VD = 1.4*VC typical; corner speed from CLmax:
    double vsCornerPos = std::sqrt(2.0 * ws * r.nMaxPos / (RHO0 * a.clmaxClean));
    r.vsPositive = vsCornerPos;
    r.vsNegative = std::sqrt(2.0 * ws * (-r.nMaxNeg) / (RHO0 * a.clmaxClean * 0.6));
    // dive speed from n at VC: VD such that q_max = n*W/(CLmin*? ) use empirical VD = 1.25 VC
    r.vDive = std::max(r.vMaxLevel * 1.25, vsCornerPos * 1.4);
    (void)q;
    return r;
}
